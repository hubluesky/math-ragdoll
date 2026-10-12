/**
 * MRMotor —— PD 电机（L3 主动控制层，core 层可测，禁止 import 'cc'）
 * 见设计文档 §5.2：速度冲量式，每子步开始（积分前）施加。
 *
 * 受击变软 = 全局 strength 衰减（MRActiveController 控制），不改求解器。
 */
import { MRVec3, MRQuat, clamp } from './MRMath';
import { MRBody } from './MRBody';

/** 单子步 |Δω| 钳制（防过冲） */
export const MOTOR_DW_CLAMP = 20; // rad/s

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

  constructor(A: MRBody, B: MRBody) {
    this.A = A;
    this.B = B;
  }

  /**
   * 施加 PD 力矩（速度冲量式）：w_B += invI_B·τ·h；w_A −= invI_A·τ·h（牛三）。
   * 在子步积分前调用；效果通过位置/旋转积分体现，最终由速度更新回读（XPBD 惯例）。
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
    const axisWorld = A.rot.applyToVec3(axisLocal, tA2);

    // τ = kp·θ·axis_world − kd·(w_B − w_A)
    const tau = tT.copy(axisWorld).scale(this.kp * theta * s);
    tau.addScaled(tW.copy(B.w).sub(A.w), -this.kd * s);
    const tauMag = tau.length();
    const tauCap = this.tauMax * s;
    if (tauMag > tauCap) tau.scale(tauCap / tauMag);

    // w += invI·τ·h，带 |Δω| 钳制
    applyAngImpulse(B, tau, h);
    applyAngImpulse(A, tau, -h);
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

const tQ1 = new MRQuat();
const tQ2 = new MRQuat();
const tQ3 = new MRQuat();
const tQ4 = new MRQuat();
const tA1 = new MRVec3();
const tA2 = new MRVec3();
const tT = new MRVec3();
const tW = new MRVec3();
const tDw = new MRVec3();
