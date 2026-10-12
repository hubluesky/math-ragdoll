/**
 * MRJoint —— 球关节 / 铰链关节（core 层，禁止 import 'cc'）
 * 见设计文档 §3.4、§3.5.2。
 *
 * 球关节：3 维位置约束，K 矩阵 3x3 求解（Müller 2020）。
 * 铰链关节 = 球关节（位置）+ 2 个角对齐标量约束（消除两自由度）+ 行程限位（MRLimit.solveHingeRange）。
 *
 * 施加约定（§3.3 点脉冲修正，符号推导见交付说明，M2 测试实证）：
 *   C = pA − pB，解 K·λ = −C
 *   x_A += λ·invM_A；Δφ_A = invI_A·(rA × λ)；q ← axisAngle(Δφ)⊗q（世界系左乘）
 *   x_B −= λ·invM_B；Δφ_B = −invI_B·(rB × λ)
 */
import { MRVec3, MRQuat, MRMat3, skewSymmetric, solve3x3, clamp } from './MRMath';
import { MRBody, rotateBodyAboutPoint } from './MRBody';
import { ANGULAR_CORRECTION_CLAMP, LINEAR_CORRECTION_CLAMP } from './MRContact';

// ---------------------------------------------------------------------------
// 球关节位置求解（MRBallJoint 与 MRHingeJoint 共用）
// ---------------------------------------------------------------------------

const pjR1 = new MRVec3();
const pjR2 = new MRVec3();
const pjC = new MRVec3();
const pjL = new MRVec3();
const pjK = new MRMat3();
const pjSkew = new MRMat3();
const pjM1 = new MRMat3();
const pjM2 = new MRMat3();

/**
 * 两体锚点重合约束（§3.4）：
 * K = (invM_A + invM_B)·I − [rA]×·invI_A·[rA]× − [rB]×·invI_B·[rB]×，解 (K+α̃I)·λ = −C。
 */
export function solveBallPosition(
  A: MRBody, B: MRBody,
  rALocal: Readonly<MRVec3>, rBLocal: Readonly<MRVec3>,
  h: number, compliance: number,
): void {
  const rA = A.rot.applyToVec3(rALocal, pjR1);
  const rB = B.rot.applyToVec3(rBLocal, pjR2);

  // C = pA − pB
  const C = pjC.copy(A.x).add(rA).sub(B.x).sub(rB);

  // K = (invM_A + invM_B)·I − [rA]×·invI_A·[rA]× − [rB]×·invI_B·[rB]×
  const K = pjK.identity();
  const wLin = (A.isStatic ? 0 : A.invMass) + (B.isStatic ? 0 : B.invMass);
  K.m[0] += wLin; K.m[4] += wLin; K.m[8] += wLin;
  subSkewBlock(K, rA, A.invIworld);
  subSkewBlock(K, rB, B.invIworld);

  if (compliance > 0) {
    const at = compliance / (h * h);
    K.m[0] += at; K.m[4] += at; K.m[8] += at;
  }

  const lambda = solve3x3(K, C, pjL);
  lambda.negate(); // λ = −K⁻¹·C

  // 单次线修正钳制：|λ|·invMass ≤ LINEAR_CORRECTION_CLAMP（每体各自受限）
  const lambdaLen = lambda.length();
  if (lambdaLen > 1e-14) {
    let maxLambda = Number.POSITIVE_INFINITY;
    if (!A.isStatic && A.invMass > 0) maxLambda = Math.min(maxLambda, LINEAR_CORRECTION_CLAMP / A.invMass);
    if (!B.isStatic && B.invMass > 0) maxLambda = Math.min(maxLambda, LINEAR_CORRECTION_CLAMP / B.invMass);
    if (lambdaLen > maxLambda) lambda.scale(maxLambda / lambdaLen);
  }

  // 施加：x_A += λ·invM_A；Δφ_A = +invI_A·(rA × λ)；x_B −= λ·invM_B；Δφ_B = −invI_B·(rB × λ)
  if (!A.isStatic) {
    A.x.addScaled(lambda, A.invMass);
    applyAngularFromLambda(A, rA, lambda, 1);
  }
  if (!B.isStatic) {
    B.x.addScaled(lambda, -B.invMass);
    applyAngularFromLambda(B, rB, lambda, -1);
  }
}

/** K −= [r]×·invI·[r]× */
function subSkewBlock(K: MRMat3, r: Readonly<MRVec3>, invI: Readonly<MRMat3>): void {
  skewSymmetric(r, pjSkew);
  pjM1.mulMat(pjSkew, invI);
  pjM2.mulMat(pjM1, pjSkew);
  for (let i = 0; i < 9; i++) K.m[i] -= pjM2.m[i];
}

// ---------------------------------------------------------------------------
// MRBallJoint
// ---------------------------------------------------------------------------

export class MRBallJoint {
  A: MRBody;
  B: MRBody;
  /** 锚点在两体局部系（COM→锚点） */
  rALocal = new MRVec3();
  rBLocal = new MRVec3();
  compliance = 0;
  name = '';

  constructor(A: MRBody, B: MRBody) {
    this.A = A;
    this.B = B;
  }

  solve(h: number): void {
    solveBallPosition(this.A, this.B, this.rALocal, this.rBLocal, h, this.compliance);
  }
}

// ---------------------------------------------------------------------------
// MRHingeJoint
// ---------------------------------------------------------------------------

export class MRHingeJoint {
  A: MRBody;
  B: MRBody;
  rALocal = new MRVec3();
  rBLocal = new MRVec3();
  /** 父体铰链轴（局部系，单位向量） */
  hingeAxisALocal = new MRVec3(1, 0, 0);
  /** 子体铰链轴（局部系，单位向量） */
  hingeAxisBLocal = new MRVec3(1, 0, 0);
  /** 父体参考垂直轴（局部系，⊥ 铰链轴，行程零位参考） */
  refAxisALocal = new MRVec3(0, 1, 0);
  /**
   * 子体零位参考方向（局部系，⊥ 铰链轴，bind 时与 refAxisALocal 世界对齐）。
   * 查漏补缺：行程限位测的是此方向相对 refAxisALocal 的角（原文档误写为测铰链轴 uB——
   * uB 已被角对齐钉死在 uA 上，垂直平面投影为零，无法测角）。
   */
  refAxisBLocal = new MRVec3(0, 1, 0);
  /** 行程范围（弧度），由 MRLimit.solveHingeRange 处理 */
  aMin = 0;
  aMax = Math.PI;
  name = '';

  constructor(A: MRBody, B: MRBody) {
    this.A = A;
    this.B = B;
  }

  solve(h: number): void {
    solveBallPosition(this.A, this.B, this.rALocal, this.rBLocal, h, 0);
  }

  /**
   * 角对齐（参考实现方式）：corr = uA × uB 的纯角对修正，一次消除两自由度
   * （叉积向量同时编码偏差角与方向），COM 不动，锚点由位置约束维护。
   */
  solveAngleAlignment(): void {
    const A = this.A;
    const B = this.B;
    const uA = A.rot.applyToVec3(this.hingeAxisALocal, tU1);
    const uB = B.rot.applyToVec3(this.hingeAxisBLocal, tU2);
    const corr = tG1.cross(uA, uB);
    applyAngularPair(A, B, corr);
  }
}

// ---------------------------------------------------------------------------
// 共享工具
// ---------------------------------------------------------------------------

/** 角逆质量：gᵀ·invI·g */
export function angularInvMassAlong(body: MRBody, g: Readonly<MRVec3>): number {
  const I = body.invIworld.m;
  const ix = I[0] * g.x + I[1] * g.y + I[2] * g.z;
  const iy = I[3] * g.x + I[4] * g.y + I[5] * g.z;
  const iz = I[6] * g.x + I[7] * g.y + I[8] * g.z;
  return ix * g.x + iy * g.y + iz * g.z;
}

/**
 * 绕世界点 anchor 旋转的角逆质量（平行轴定理）：n 方向单位角修正的广义逆质量。
 * 纯角逆质量 invI 漏掉 COM 平移动能（等效惯量应为 I + m·d⊥²），
 * 会导致限位修正按过小的惯量分配、转过修正、持续振荡注入能量（实测全身布娃娃抖动不止）。
 * 静态体返回 0。
 */
export function angularInvMassAboutAnchor(body: MRBody, n: Readonly<MRVec3>, anchor: Readonly<MRVec3>): number {
  if (body.isStatic) return 0;
  const invIAngular = angularInvMassAlong(body, n);
  if (invIAngular <= 1e-12) return 0;
  const inertiaAngular = 1 / invIAngular; // 正惯量（n 方向）
  const dx = body.x.x - anchor.x;
  const dy = body.x.y - anchor.y;
  const dz = body.x.z - anchor.z;
  const dn = dx * n.x + dy * n.y + dz * n.z;
  const leverPerp2 = dx * dx + dy * dy + dz * dz - dn * dn; // 垂直于转轴的杠杆分量²
  const mass = body.invMass > 0 ? 1 / body.invMass : 0;
  const inertiaEff = inertiaAngular + leverPerp2 * mass; // 平行轴
  if (inertiaEff <= 1e-12) return 0;
  return 1 / inertiaEff;
}

/** out = invI_world · v */
export function applyInvI(body: MRBody, v: Readonly<MRVec3>, out: MRVec3): MRVec3 {
  const I = body.invIworld.m;
  return out.set(
    I[0] * v.x + I[1] * v.y + I[2] * v.z,
    I[3] * v.x + I[4] * v.y + I[5] * v.z,
    I[6] * v.x + I[7] * v.y + I[8] * v.z,
  );
}

/**
 * 纯角对修正（Müller 2020 / Ten Minute Physics 第 25 课的限位施加方式）：
 * corr = 旋转差向量（方向 = 修正旋转轴，大小 = sin(角差)）。
 * body0 沿 +corr 转、body1 沿 −corr 转，各施 0.5·invI·corr 的角修正（COM 不动！）。
 *
 * COM 不平移是稳定性的关键：绕锚点旋转会平移 COM，位置回读 v = Δx/h 把修正
 * 变成巨大的速度脉冲，约束间拉锯下持续注入能量（实测全身布娃娃爆炸）。
 * 锚点由球关节位置约束在后续求解中维护（与参考实现一致）。
 */
export function applyAngularPair(A: MRBody, B: MRBody, corr: Readonly<MRVec3>): void {
  const C = corr.length();
  if (C < 1e-12 || !Number.isFinite(C)) return;
  const n = tNPair.copy(corr).scale(1 / C);
  const w = angularInvMassAlong(A, n) + angularInvMassAlong(B, n);
  if (w <= 1e-12) return;
  // body0: Δφ = +0.5·invI·(C/w)·n；body1: 反向
  const scale = 0.5 * (C / w);
  applyHalfRotation(A, tD1A.copy(n).scale(scale));
  applyHalfRotation(B, tD1B.copy(n).scale(-scale));
}

/** 单体纯角修正：q ← axisAngle(0.5·invI·dφ)⊗q（0.5 缩放：每子步消除一半约束违反，兼顾收敛与稳定） */
function applyHalfRotation(body: MRBody, dPhi: Readonly<MRVec3>): void {
  if (body.isStatic) return;
  applyInvI(body, dPhi, tD2A);
  const phi = tD2A.length();
  if (phi < 1e-14 || !Number.isFinite(phi)) return;
  const clamped = clamp(phi, 0, ANGULAR_CORRECTION_CLAMP);
  const axis = tAxisPair.copy(tD2A).scale(1 / phi);
  tQuatPair.fromAxisAngle(axis, clamped);
  body.q.multiply(tQuatPair, body.q).normalize();
  // 同步刷新 rot 缓存：同一子步内后续约束/诊断必须看到本次角修正
  body.rot.fromQuat(body.q);
}

const tNPair = new MRVec3();
const tD1A = new MRVec3();
const tD1B = new MRVec3();
const tD2A = new MRVec3();
const tAxisPair = new MRVec3();
const tQuatPair = new MRQuat();

/** 绕世界点 anchor 施加角修正 Δφ（带钳制），静态体跳过 */
export function applyAnchorAngular(body: MRBody, anchor: Readonly<MRVec3>, dPhi: Readonly<MRVec3>): void {
  if (body.isStatic) return;
  const phi = dPhi.length();
  if (phi < 1e-14 || !Number.isFinite(phi)) return;
  // 绕锚点旋转会平移 COM：Δx ≈ φ·|COM−anchor|。钳制必须按 COM 位移而非角本身——
  // 否则长杠杆（限位锚点远离 COM）下 0.2rad 会产生每子步数 cm 的速度跳变，
  // 大幅度翻滚场景中持续注入能量（实测全身布娃娃永不静止并随子步数放大爆炸）。
  const lever = body.x.distanceTo(anchor);
  const maxPhi = Math.min(ANGULAR_CORRECTION_CLAMP, LINEAR_CORRECTION_CLAMP / Math.max(lever, 1e-6));
  const clamped = clamp(phi, 0, maxPhi);
  const axis = tAxisAA.copy(dPhi).scale(1 / phi);
  rotateBodyAboutPoint(body, anchor, axis, clamped);
}

/** 施加角修正 Δφ = sign·invI·(r × λ)（球关节的角部分；绕 COM 转 = 纯旋转） */
export function applyAngularFromLambda(body: MRBody, r: Readonly<MRVec3>, lambda: Readonly<MRVec3>, sign: number): void {
  if (body.isStatic) return;
  tRxL.cross(r, lambda);
  const I = body.invIworld.m;
  tDPhi2.set(
    (I[0] * tRxL.x + I[1] * tRxL.y + I[2] * tRxL.z) * sign,
    (I[3] * tRxL.x + I[4] * tRxL.y + I[5] * tRxL.z) * sign,
    (I[6] * tRxL.x + I[7] * tRxL.y + I[8] * tRxL.z) * sign,
  );
  applyAnchorAngular(body, body.x, tDPhi2);
}

// 模块级临时对象
const tR1 = new MRVec3();
const tU1 = new MRVec3();
const tU2 = new MRVec3();
const tU3 = new MRVec3();
const tU4 = new MRVec3();
const tG1 = new MRVec3();
const tG2 = new MRVec3();
const tG3 = new MRVec3();
const tD1 = new MRVec3();
const tD2 = new MRVec3();
const tDPhi2 = new MRVec3();
const tAxisAA = new MRVec3();
const tRxL = new MRVec3();
const tAnchor = new MRVec3();
