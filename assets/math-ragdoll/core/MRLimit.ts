/**
 * MRLimit —— 关节限位（core 层，禁止 import 'cc'）
 *
 * 实现方式与 Müller 2020 / Ten Minute Physics 第 25 课（25-joints.html，MIT）的
 * 参考实现对齐（查漏补缺：文档 §3.5.1 的"绕锚点旋转修正"在长杠杆下会把 COM
 * 平移量经速度回读变成能量注入源，实测全身布娃娃爆炸；参考实现用**纯角修正**
 * ——COM 不动，仅转姿态，锚点由球关节位置约束维护，从根本上消除该注入通道）。
 *
 * 统一修正原语（applyAngularPair，见 MRJoint）：
 *   corr = 旋转差向量（方向 = 修正轴，大小 = sin(角差)），两体各施 0.5·invI·corr。
 *
 * 测角原语（getAngle，同参考实现）：绕轴 n 从 a 到 b 的有符号角。
 */
import { MRVec3, MRQuat, clamp } from './MRMath';
import { MRBody } from './MRBody';
import { MRHingeJoint, applyAngularPair, projectRelativeAngularVelocity, angularInvMassAlong, applyInvI } from './MRJoint';

/**
 * 限位软边界：修正目标点是外推 LIMIT_SLOP 的软边界，而非硬边界。
 * 肘/膝等 aMin=0 的关节静置时贴死边界，重力每子步把关节压过界一丝、
 * 限位又修回来，修正经速度回读变成角速度 —— 边界上永不衰减的 chatter。
 * 软边界让贴边静默（超界 < LIMIT_SLOP 不修），超软边界的部分才修，
 * 稳态超硬边界 ≈ LIMIT_SLOP + 欠松弛收敛残余（M3 验收容差 2°、
 * 符号实证容差 0.5° 之内）。
 * 注意不能做成"超界 < slop 时整段不修"：那样违反会积累到 slop 再一次
 * 全量爆发，脉冲反而更大。
 */
export const LIMIT_SLOP = 0.25 * Math.PI / 180;

/**
 * 球关节被动扭转阻尼默认时间常数（秒）。
 *
 * 物理依据：关节囊是预张紧的黏弹性膜，骨骼绕关节轴拧转时囊膜被绞紧，
 * 产生全范围（不限位边界内也生效）的黏性阻力矩——放松人体的被动轴向转动
 * 在 0.3~0.5s 内自然停止，绝不会像光滑圆柱一样自由旋转。现有模型只有
 * 韧带"硬墙"（锥面/扭转限位）而缺这一黏性项，落地冲击注入的滚转角速度
 * 只能靠全局 angDamp（τ≈4s）耗散 —— 实测手部以 ~10 rad/s 自由滚转 4 秒
 * 以上、限位边界拍打放大到 30~55 rad/s（"圆柱式自转"的直接根因）。
 *
 * 数学形式：I_r·dω_t/dt = −c·ω_t，c = I_r/τ ⟹ ω_t(t) = ω_t(0)·e^(−t/τ)；
 * 每子步消除比例 f = 1 − e^(−h/τ)（≤0.5 钳制，同电机 kd 稳定模式），
 * 按广义逆质量分摊 —— 复用 projectRelativeAngularVelocity（relax=f）。
 * 只作用扭转轴 n̂2，不阻尼锥面摆动（布娃娃"瘫软"观感本体 + 弯曲阻力
 * 确实小），τ 与模型尺度无关。逐关节可覆盖（JointDef.limit.twistDamp），
 * 显式 0 = 关闭。
 *
 * [v1.6] 0.15 → 0.05：0.15s 下冲击注入的扭转角速率（~15 rad/s）在衰减前
 * 已把扭角扫满 ±45° 限位（渐近扫角 = ω0·τ_d = 129° >> 45°），用户观感
 * "自转幅度太大"（限位全幅来回）。0.05s 时渐近扫角 = ω0·τ_d ≈ 43°，
 * 配合刚度项把击打后扭转峰值压到 ~27°（实测）。
 */
export const DEFAULT_TWIST_DAMP = 0.05;

/**
 * 球关节被动扭转刚度默认回中时间常数（秒）——关节囊黏弹性模型的弹性项。
 *
 * 物理依据：真实关节囊是**预张紧**的膜，轴向拧转会绞紧囊膜产生指向中性位
 * 的回复力矩——放松的肢体被拧转后会缓慢回中，而不是带着扭转角继续"流"。
 * 仅黏性阻尼（twistDamp）的数学极限：静置承重时接触/摩擦/速度回读每子步
 * 持续注入微小滚转，阻尼只能把它平衡成**恒定滚转速率**（ω_ss = 注入率×τ，
 * 实测 0.2~0.5 rad/s）——肢体恒速绕轴自转，直到撞上 ±45°/±15° 扭转限位
 * 才停（0.35 rad/s 转满 45° ≈ 2s，即用户看到的"自转一两秒后停止"）。
 * 弹性项把渗漏平衡成**静态扭转角**（θ_ss = 注入率×τ_s，限位内），
 * 速度归零——肢体静止，与真实关节一致。
 *
 * 实现：每子步将当前扭角向 0 修正比例 g = h/τ_s（≤0.25 钳制），轴 = 子体
 * 关节轴 a1（与锥面摆动正交，同 twistDamp 的轴选择依据），修正量经
 * applyAngularPair 按广义逆质量分摊。0 = 关闭。逐关节可覆盖
 * （JointDef.limit.twistStiffness）。
 *
 * [v1.6] 0.3 → 0.08：配合阻尼加强（v1.6），把击打后扭转峰值进一步压低
 * （实测静置击打 27° → 9°），且回中更快（e^(-t/0.08)）。
 */
export const DEFAULT_TWIST_STIFFNESS = 0.08;

/**
 * 铰链弯曲被动阻尼默认时间常数（秒）——[v1.8] 关节被动矩模型的黏性项推广到
 * 弯曲自由度（肘/膝）。
 *
 * 现实依据：生物力学被动关节矩标准模型（Audu & Davy 1985、Riener & Edrich
 * 1999）——真实关节被动阻力矩 = 弹性项（限位端，由关节囊/韧带提供）+ 黏性项
 * （软组织黏弹性，−b·ω）。静息肢体受扰后 0.5~1s 内停止。本项目此前的模型中
 * 弯曲自由度限位内零阻力：击打/落地后重力经零阻力关节持续塌陷，膝角以
 * τ ≈ 5~15s 的指数长尾缓慢伸直（实测 10s 内 13°→5°、maxV 恒 1~2.4cm/s），
 * 比真实慢 3~10 倍——用户观感"击打后手臂/关节缓缓伸直、像光滑表面一样轻微位移"。
 *
 * 数学形式：Δω_rel = −f·ω_rel，f = 1 − e^(−h/τ_b)（≤0.5 钳制），沿铰链轴、
 * 按广义逆质量分摊（与扭转阻尼同一投影原语）。沉降时间常数由长尾的
 * τ ≈ 5~15s 压到 ≈ τ_b 量级（默认 0.2s → 沉降 ~0.5~1s，符合真实）。
 * 甩动不受损：击打甩动力矩（5~20 N·m）远大于残余重力矩（<1 N·m），阻尼
 * 只压缩沉降尾段，初始幅度由冲量决定。
 * 0 = 关闭。逐关节可覆盖（JointDef.limit.flexDamp）。
 *
 * [v1.8] 取值扫掠（孪生落地 5s 后速度地板）：0→0.025、0.8→0.0119（最优）、
 * 0.4→0.0141、0.3→0.0202、0.2→0.0681 m/s。弱阻尼压制接触循环扰动；过强
 * （f 每子步 >1%）在持续载荷下泵入速度地板（与位置级弹簧同族的稳态伪影）。
 * 甜点 f ≈ 0.26%/子步。
 */
export const DEFAULT_FLEX_DAMP = 0.8;

/** 绕轴 n 从 a 到 b 的有符号角（参考实现 getAngle） */
function getAngle(n: Readonly<MRVec3>, a: Readonly<MRVec3>, b: Readonly<MRVec3>, tmpCross: MRVec3): number {
  const c = tmpCross.cross(a, b);
  let phi = Math.asin(clamp(c.dot(n), -1, 1));
  if (a.dot(b) < 0) phi = Math.PI - phi;
  if (phi > Math.PI) phi -= 2 * Math.PI;
  if (phi < -Math.PI) phi += 2 * Math.PI;
  return phi;
}

/** 参考实现 limitAngle：角超界时构造旋转差 corr 并施加纯角对修正 */
function limitAngle(
  n: Readonly<MRVec3>, a: Readonly<MRVec3>, b: Readonly<MRVec3>,
  minAngle: number, maxAngle: number,
  A: MRBody, B: MRBody,
  tCross: MRVec3, tRa: MRVec3, tCorr: MRVec3,
): void {
  if (measureViolation(n, a, b, minAngle, maxAngle, tCross, tRa, tCorr) === 0) return;
  applyAngularPair(A, B, tCorr);
}

/**
 * 测量 a 绕 n 相对 b 超出 [min−SLOP, max+SLOP] 软边界的违反量。
 * 返回 0 = 无违反；>0 时 tCorr = 修正旋转向量（方向 = 修正轴，
 * 大小 = sin(到软边界的角差)），与 applyAngularPair 的约定一致，
 * 位置修正与速度松弛共用同一测量。
 */
function measureViolation(
  n: Readonly<MRVec3>, a: Readonly<MRVec3>, b: Readonly<MRVec3>,
  minAngle: number, maxAngle: number,
  tCross: MRVec3, tRa: MRVec3, tCorr: MRVec3,
): number {
  const phi = getAngle(n, a, b, tCross);
  const phiC = clamp(phi, minAngle - LIMIT_SLOP, maxAngle + LIMIT_SLOP);
  if (phi === phiC) return 0;
  // ra = a 绕 n 转 phiC（Rodrigues）
  const cos = Math.cos(phiC), sin = Math.sin(phiC), one = 1 - cos;
  const kdot = n.x * a.x + n.y * a.y + n.z * a.z;
  const cx = n.y * a.z - n.z * a.y;
  const cy = n.z * a.x - n.x * a.z;
  const cz = n.x * a.y - n.y * a.x;
  tRa.set(
    a.x * cos + cx * sin + n.x * kdot * one,
    a.y * cos + cy * sin + n.y * kdot * one,
    a.z * cos + cz * sin + n.z * kdot * one,
  );
  tCorr.cross(tRa, b);
  return Math.abs(phi - phiC);
}

/**
 * 限位速度松弛（XPBD 速度阶段，单边约束 restitution=0 语义）：
 * tCorr 为 measureViolation 输出的修正轴；两体相对角速度沿该轴的分量
 * 若朝"加深违反"方向，按广义逆质量消除之（压到 0，不反向助推）。
 * 没有这一步，位置法限位的修正经速度回读变成反向角速度，与持续压入源
 * （重力/电机/约束耦合）形成固定步级方波 —— 贴边关节原地高频抽搐。
 * 投影后稳态 = 恒力矩压在软边界上（零速度），不再弹跳。
 */
function solveLimitVelocity(A: MRBody, B: MRBody, corr: Readonly<MRVec3>): void {
  projectRelativeAngularVelocity(A, B, corr, true);
}

// ---------------------------------------------------------------------------
// 锥面 + 扭转限位
// ---------------------------------------------------------------------------

export class MRConeTwistLimit {
  A: MRBody;
  B: MRBody;
  /** 锚点（两体局部系；构建/诊断用，修正本身不使用锚点） */
  rALocal = new MRVec3();
  rBLocal = new MRVec3();
  /** 关节主轴（A/B 局部系，bind 时世界对齐；对应参考实现的 x 轴） */
  axisALocal = new MRVec3(0, 1, 0);
  axisBLocal = new MRVec3(0, 1, 0);
  /** 扭转测量垂直参考轴（A/B 局部系，bind 时世界对齐；对应参考实现的 y 轴） */
  twistRefALocal = new MRVec3(1, 0, 0);
  twistRefBLocal = new MRVec3(1, 0, 0);
  /** bind 相对旋转 q0 = conj(qA_bind) ⊗ qB_bind（诊断用） */
  q0Bind = new MRQuat();
  coneMax = 0;
  twistMin = 0;
  twistMax = 0;
  /**
   * 被动扭转阻尼时间常数（秒）：关节囊黏性项，见 DEFAULT_TWIST_DAMP。
   * 0 = 关闭（限位边界投影仍生效）。
   */
  twistDamp = 0;
  /**
   * 被动扭转刚度回中时间常数（秒）：关节囊弹性项，见 DEFAULT_TWIST_STIFFNESS。
   * 0 = 关闭。
   */
  twistStiffness = 0;
  name = '';

  constructor(A: MRBody, B: MRBody) {
    this.A = A;
    this.B = B;
  }

  solve(): void {
    const A = this.A;
    const B = this.B;
    const a0 = A.rot.applyToVec3(this.axisALocal, tA0);
    const a1 = B.rot.applyToVec3(this.axisBLocal, tA1);

    // ---- 锥面（swing）：n = 主轴叉积，单边 [0, coneMax] ----
    const n = tN.cross(a0, a1);
    const nLen = n.length();
    if (nLen > 1e-9) {
      n.scale(1 / nLen);
      limitAngle(n, a0, a1, 0, this.coneMax, A, B, tCross, tRa, tCorr);
    }
    // 主轴反向平行时锥面方向无定义，跳过（其他约束会推离奇异点）

    // ---- 扭转（twist）：n = 主轴和的角平分线，垂直参考轴投影后测有符号角 ----
    const n2 = tN2.copy(a0).add(a1);
    const n2Len = n2.length();
    if (n2Len > 1e-9) {
      n2.scale(1 / n2Len);
      const vA = A.rot.applyToVec3(this.twistRefALocal, tVA);
      const vB = B.rot.applyToVec3(this.twistRefBLocal, tVB);
      vA.addScaled(n2, -n2.dot(vA)).normalize();
      vB.addScaled(n2, -n2.dot(vB)).normalize();
      limitAngle(n2, vA, vB, this.twistMin, this.twistMax, A, B, tCross, tRa, tCorr);
    }
  }

  /** 诊断（M3 验收用）：当前锥面角与扭转角（弧度） */
  getCurrentDeviation(): { coneAngle: number; twistAngle: number } {
    const a0 = this.A.rot.applyToVec3(this.axisALocal, tA0);
    const a1 = this.B.rot.applyToVec3(this.axisBLocal, tA1);
    const coneAngle = Math.acos(clamp(a0.dot(a1), -1, 1));

    const n2 = tN2.copy(a0).add(a1);
    const n2Len = n2.length();
    if (n2Len < 1e-9) return { coneAngle, twistAngle: 0 };
    n2.scale(1 / n2Len);
    const vA = this.A.rot.applyToVec3(this.twistRefALocal, tVA);
    const vB = this.B.rot.applyToVec3(this.twistRefBLocal, tVB);
    vA.addScaled(n2, -n2.dot(vA)).normalize();
    vB.addScaled(n2, -n2.dot(vB)).normalize();
    return { coneAngle, twistAngle: getAngle(n2, vA, vB, tCross) };
  }

  /**
   * 速度松弛（XPBD 速度阶段，子步速度更新后调用）：
   * 锥面与扭转限界各自独立投影——测量与 solve() 同构，仅把位置修正换成
   * "加深违反方向的相对角速度消除"。
   * 之后施加被动扭转黏弹性（twistDamp/twistStiffness > 0 时恒激活，限位内也作用）：
   * - 黏性：沿子体关节轴 a1 按 f = 1 − e^(−h/τ_d) 消除相对滚转速率（见 DEFAULT_TWIST_DAMP）；
   * - 弹性：沿 a1 施加 Δω_rel = −(h/τ_s²)·θ 的相对角速度冲量（θ = a1 轴扭角，
   *   物理弹簧离散化，见 DEFAULT_TWIST_STIFFNESS）。
   * 轴选 a1（子体关节轴）而非 n̂2 的依据见 DEFAULT_TWIST_DAMP 注释
   * （与锥面摆动正交解耦；n̂2 在大锥角下与锥面边界投影拉锯实测发散）。
   * 弹性曾尝试位置级（每子步角度修正 g·θ），实测经速度回读变成 θ/τ_s 量级的
   * 速度踢，在承重/电机场景与约束打擂台（M4 闭环 |v|=0.21、肘对齐 24 rad/s
   * 抖动）——速度级冲量形式平滑注入、由黏性项吸收，无位置擂台。
   */
  solveVelocity(h: number): void {
    const A = this.A;
    const B = this.B;
    const a0 = A.rot.applyToVec3(this.axisALocal, tA0);
    const a1 = B.rot.applyToVec3(this.axisBLocal, tA1);

    // ---- 锥面 ----
    const n = tN.cross(a0, a1);
    if (n.length() > 1e-9) {
      if (measureViolation(n, a0, a1, 0, this.coneMax, tCross, tRa, tCorr) > 0) {
        solveLimitVelocity(A, B, tCorr);
      }
    }

    // ---- 扭转（限界）----
    const n2 = tN2.copy(a0).add(a1);
    const n2Len = n2.length();
    if (n2Len > 1e-9) {
      n2.scale(1 / n2Len);
      const vA = A.rot.applyToVec3(this.twistRefALocal, tVA);
      const vB = B.rot.applyToVec3(this.twistRefBLocal, tVB);
      vA.addScaled(n2, -n2.dot(vA)).normalize();
      vB.addScaled(n2, -n2.dot(vB)).normalize();
      if (measureViolation(n2, vA, vB, this.twistMin, this.twistMax, tCross, tRa, tCorr) > 0) {
        solveLimitVelocity(A, B, tCorr);
      }
    }

    // ---- 被动扭转黏弹性（关节囊，恒激活，轴 = 子体关节轴 a1）----
    if (this.twistDamp > 0 || this.twistStiffness > 0) {
      // 边界投影可能已转动身体，重算最新 a1
      const ax1 = B.rot.applyToVec3(this.axisBLocal, tA1);
      const vA = A.rot.applyToVec3(this.twistRefALocal, tVA);
      const vB = B.rot.applyToVec3(this.twistRefBLocal, tVB);
      vA.addScaled(ax1, -ax1.dot(vA));
      vB.addScaled(ax1, -ax1.dot(vB));
      if (vA.lengthSqr() > 1e-12 && vB.lengthSqr() > 1e-12) {
        vA.normalize();
        vB.normalize();
        // 黏性：消除 f 比例的相对滚转速率
        if (this.twistDamp > 0) {
          const f = 1 - Math.exp(-h / this.twistDamp);
          projectRelativeAngularVelocity(A, B, ax1, false, Math.min(f, 0.5));
        }
        // 弹性：Δω_rel = −(h/τ_s²)·θ，按广义逆质量分摊（物理弹簧离散化）
        if (this.twistStiffness > 0) {
          const theta = getAngle(ax1, vA, vB, tCross);
          if (Math.abs(theta) > 1e-12) {
            const w = angularInvMassAlong(A, ax1) + angularInvMassAlong(B, ax1);
            if (w > 1e-12) {
              const d = (-(h / (this.twistStiffness * this.twistStiffness)) * theta) / w;
              applyInvI(B, ax1, tSpringB);
              B.w.addScaled(tSpringB, d);
              applyInvI(A, ax1, tSpringA);
              A.w.addScaled(tSpringA, -d);
            }
          }
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 铰链行程限位
// ---------------------------------------------------------------------------

/**
 * 铰链行程限位（参考实现 hinge limits）：n = 铰链轴（父体），垂直参考轴投影后测角。
 * 在 solveAngleAlignment 之后调用。
 */
export function solveHingeRange(hinge: MRHingeJoint): void {
  const A = hinge.A;
  const B = hinge.B;
  const n = A.rot.applyToVec3(hinge.hingeAxisALocal, tH1);
  const vA = A.rot.applyToVec3(hinge.refAxisALocal, tH2);
  const vB = B.rot.applyToVec3(hinge.refAxisBLocal, tH3);
  // 垂直参考轴投影到⊥铰链轴平面（对齐约束保证已垂直，投影保险）
  vA.addScaled(n, -n.dot(vA));
  if (vA.lengthSqr() < 1e-12) return;
  vA.normalize();
  vB.addScaled(n, -n.dot(vB));
  if (vB.lengthSqr() < 1e-12) return;
  vB.normalize();
  limitAngle(n, vA, vB, hinge.aMin, hinge.aMax, A, B, tCross, tRa, tCorr);
}

/**
 * 铰链行程限位的速度松弛（子步速度更新后调用，与 solveHingeRange 配对）。
 * 绕铰链轴的拧转是行程测不到的自由自由度，行程贴边时由这里拦截压入速度。
 */
export function solveHingeRangeVelocity(hinge: MRHingeJoint): void {
  const A = hinge.A;
  const B = hinge.B;
  const n = A.rot.applyToVec3(hinge.hingeAxisALocal, tH1);
  const vA = A.rot.applyToVec3(hinge.refAxisALocal, tH2);
  const vB = B.rot.applyToVec3(hinge.refAxisBLocal, tH3);
  vA.addScaled(n, -n.dot(vA));
  if (vA.lengthSqr() < 1e-12) return;
  vA.normalize();
  vB.addScaled(n, -n.dot(vB));
  if (vB.lengthSqr() < 1e-12) return;
  if (measureViolation(n, vA, vB, hinge.aMin, hinge.aMax, tCross, tRa, tCorr) > 0) {
    solveLimitVelocity(A, B, tCorr);
  }
}

/** 诊断（M3 验收用）：当前行程角（弧度） */
export function getHingeRangeAngle(hinge: MRHingeJoint): number {
  const A = hinge.A;
  const B = hinge.B;
  const n = A.rot.applyToVec3(hinge.hingeAxisALocal, tH1);
  const vA = A.rot.applyToVec3(hinge.refAxisALocal, tH2);
  const vB = B.rot.applyToVec3(hinge.refAxisBLocal, tH3);
  vA.addScaled(n, -n.dot(vA));
  if (vA.lengthSqr() < 1e-12) return 0;
  vA.normalize();
  vB.addScaled(n, -n.dot(vB));
  if (vB.lengthSqr() < 1e-12) return 0;
  vB.normalize();
  return getAngle(n, vA, vB, tCross);
}

// 模块级临时对象
const tA0 = new MRVec3();
const tA1 = new MRVec3();
const tN = new MRVec3();
const tN2 = new MRVec3();
const tVA = new MRVec3();
const tVB = new MRVec3();
const tCross = new MRVec3();
const tRa = new MRVec3();
const tCorr = new MRVec3();
const tH1 = new MRVec3();
const tH2 = new MRVec3();
const tH3 = new MRVec3();
const tSpringA = new MRVec3();
const tSpringB = new MRVec3();
