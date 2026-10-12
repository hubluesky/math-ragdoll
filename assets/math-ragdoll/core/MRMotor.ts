/**
 * MRMotor —— PD 电机（L3 主动控制层，core 层可测，禁止 import 'cc'）
 * 见设计文档 §5.2：速度冲量式，每子步开始（积分前）施加。
 *
 * 受击变软 = 全局 strength 衰减（MRActiveController 控制），不改求解器。
 */
import { MRVec3, MRQuat, clamp } from './MRMath';
import { MRBody } from './MRBody';

/** 单子步 |Δω| 钳制（防过冲）。20 rad/s/子步 对手（invI≈4700）等轻小肢体
 * 等于每秒 9600 rad/s² 的加速能力，毫无约束意义（目标错位时恒打满 → bang-bang）；
 * 躯干典型需求 ~0.1 rad/s/子步，2 已留 20 倍余量。 */
export const MOTOR_DW_CLAMP = 2; // rad/s

export class MRMotor {
  /** 父体 */
  A: MRBody;
  /** 子体 */
  B: MRBody;
  /** 目标相对旋转（A 局部系）：qTarget_rel = conj(qA_target) ⊗ qB_target */
  targetRelLocal = new MRQuat();
  /** 比例增益 */
  kp = 100;
  /** 微分增益（临界阻尼起点 kd = 2√kp） */
  kd = 20;
  /** 最大力矩幅值（N·m） */
  tauMax = 50;
  /** 全局强度 0~1（受击变软接口） */
  strength = 1;
  name = '';
  // ---- 诊断缓存（apply 时记录，联调采集用）----
  /** 上次施加的力矩幅值（钳制后，N·m） */
  tauMagLast = 0;
  /** 上次的姿态误差角（rad） */
  errAngleLast = 0;

  constructor(A: MRBody, B: MRBody) {
    this.A = A;
    this.B = B;
  }

  /**
   * 施加 PD 力矩（速度冲量式）：w_B += invI_B·τ·h；w_A −= invI_A·τ·h（牛三）。
   * 在子步积分前调用；效果通过位置/旋转积分体现，最终由速度更新回读（XPBD 惯例）。
   *
   * kp 项与 kd 项分开施加：kd 阻尼项的显式欧拉增益 invI·kd·h 在轻小肢体上
   * 远超离散稳定界（手 invI≈4700 → 每子步角速度放大 ~215 倍并反号），被
   * tauMax/DW_CLAMP 钳制后变成 ±20 rad/s 的逐帧翻转（bang-bang，实测双手
   * 抽搐 52 次/秒）。钳制 |Δω_kd| ≤ 0.25·|ω_rel|（两体合计每子步最多消一半
   * 相对角速度）后阻尼回归平滑指数衰减 —— 撞击 → 反弹 → 衰减 → 静止。
   * 大惯量体（躯干/大腿 invI·kd·h ≈ 0.07）不受钳制影响，行为不变。
   */
  apply(h: number): void {
    const A = this.A;
    const B = this.B;
    const s = clamp(this.strength, 0, 1);
    if (s <= 0) return;

    // q_rel = conj(qA) ⊗ qB；q_err = qTarget ⊗ conj(q_rel)
    tQ1.copy(A.q).conjugateInPlace();
    const qRel = tQ2.multiply(tQ1, B.q);
    const qErr = tQ3.copy(this.targetRelLocal).multiply(tQ3, tQ4.copy(qRel).conjugateInPlace());
    if (qErr.w < 0) qErr.set(-qErr.x, -qErr.y, -qErr.z, -qErr.w); // 短弧

    const axisLocal = tA1;
    const theta = qErr.toAxisAngle(axisLocal);
    this.errAngleLast = theta;
    const axisWorld = A.rot.applyToVec3(axisLocal, tA2);

    // kp 位置项：τ_p = kp·θ·axis_world（tauMax 钳制）
    const tauP = tT.copy(axisWorld).scale(this.kp * theta * s);
    const tauCap = this.tauMax * s;
    const tauPMag = tauP.length();
    if (tauPMag > tauCap) tauP.scale(tauCap / tauPMag);
    this.tauMagLast = Math.min(tauPMag, tauCap);

    // w_rel = w_B − w_A（kd 项的衰减对象）
    const wRel = tW.copy(B.w).sub(A.w);

    // kp 项冲量（牛三）
    applyAngImpulse(B, tauP, h);
    applyAngImpulse(A, tauP, -h);

    // kd 阻尼项：Δω = ∓invI·(kd·w_rel)·h，带 |Δω| ≤ 0.25·|w_rel| 稳定钳制
    const kdEff = this.kd * s;
    if (kdEff > 0 && wRel.lengthSqr() > 1e-24) {
      applyDampingImpulse(B, wRel, kdEff, h, 1);
      applyDampingImpulse(A, wRel, kdEff, h, -1);
    }
  }
}

function applyAngImpulse(body: MRBody, tau: Readonly<MRVec3>, h: number): void {
  if (body.isStatic) return;
  const I = body.invIworld.m;
  tDw.set(
    (I[0] * tau.x + I[1] * tau.y + I[2] * tau.z) * h,
    (I[3] * tau.x + I[4] * tau.y + I[5] * tau.z) * h,
    (I[6] * tau.x + I[7] * tau.y + I[8] * tau.z) * h,
  );
  const dwMag = tDw.length();
  if (dwMag > MOTOR_DW_CLAMP) tDw.scale(MOTOR_DW_CLAMP / dwMag);
  body.w.add(tDw);
}

/**
 * kd 项冲量：Δω = −sign·invI·(kd·w_rel)·h。
 * 钳制 |Δω| ≤ 0.25·|w_rel|：B、A 两体合计每子步最多消除一半相对角速度，
 * 保证显式阻尼不翻转（invI·kd·h > 1 的轻小体上这是唯一的稳定保障）。
 */
function applyDampingImpulse(body: MRBody, wRel: Readonly<MRVec3>, kdEff: number, h: number, sign: number): void {
  if (body.isStatic) return;
  const I = body.invIworld.m;
  tDw2.set(
    (I[0] * wRel.x + I[1] * wRel.y + I[2] * wRel.z) * (-sign * kdEff * h),
    (I[3] * wRel.x + I[4] * wRel.y + I[5] * wRel.z) * (-sign * kdEff * h),
    (I[6] * wRel.x + I[7] * wRel.y + I[8] * wRel.z) * (-sign * kdEff * h),
  );
  const cap = 0.25 * wRel.length();
  const dwMag = tDw2.length();
  if (dwMag > cap) tDw2.scale(cap / dwMag);
  body.w.add(tDw2);
}

const tQ1 = new MRQuat();
const tQ2 = new MRQuat();
const tQ3 = new MRQuat();
const tQ4 = new MRQuat();
const tA1 = new MRVec3();
const tA2 = new MRVec3();
const tT = new MRVec3();
const tW = new MRVec3();
const tDw = new MRVec3();
const tDw2 = new MRVec3();
