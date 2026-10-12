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
import { MRHingeJoint, applyAngularPair, projectRelativeAngularVelocity } from './MRJoint';

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
   * 锥面与扭转各自独立投影——测量与 solve() 同构，仅把位置修正换成
   * "加深违反方向的相对角速度消除"。
   */
  solveVelocity(): void {
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

    // ---- 扭转 ----
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
