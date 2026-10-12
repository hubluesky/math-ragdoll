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
import { MRHingeJoint, applyAngularPair } from './MRJoint';

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
  const phi = getAngle(n, a, b, tCross);
  if (phi >= minAngle && phi <= maxAngle) return;
  const phiC = clamp(phi, minAngle, maxAngle);
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
  applyAngularPair(A, B, tCorr);
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
