/**
 * MRContact —— 接触约束 + 摩擦（core 层，禁止 import 'cc'）
 * 见设计文档 §3.6.4。
 *
 * 法线约定（勘误澄清）：n 从 A 指向 B；法向修正 A 沿 −n 退、B 沿 +n 退。
 * B = null 表示静态地面（A 为 capsule，n 取地面法线的反向，即从 capsule 指向地面）。
 * restitution = 0（非弹性，ragdoll 需要）。
 */
import { MRVec3, MRQuat, MRMat3, clamp } from './MRMath';
import { MRBody } from './MRBody';

/** 单次修正钳制（§3.3 / §7.3） */
export const LINEAR_CORRECTION_CLAMP = 0.05; // m
export const ANGULAR_CORRECTION_CLAMP = 0.2; // rad

export class MRContact {
  A: MRBody;
  B: MRBody | null; // null = 静态地面
  /** 法线：A → B（世界系，单位向量） */
  n = new MRVec3();
  /** 穿透深度（>0） */
  depth = 0;
  /** 接触点在 A 体局部系的偏移（COM→接触点） */
  rALocal = new MRVec3();
  /** 接触点在 B 体局部系的偏移；B 为 null 时未使用 */
  rBLocal = new MRVec3();
  /** 摩擦系数 */
  friction = 0;

  constructor(A: MRBody, B: MRBody | null) {
    this.A = A;
    this.B = B;
  }

  /**
   * 求解一个接触（法向 + 摩擦）。
   * @param applyFriction 摩擦只在每子步第一轮接触迭代时施加：
   * 后续轮次的 dispT 会被其他接触的修正反复改写，若每轮都施摩擦，
   * 累计消除量会远超 μ·depth 预算（实测 μ 完全失效、斜面钉死）。
   */
  solve(h: number, applyFriction = true): void {
    const A = this.A;
    const B = this.B;
    const n = this.n;

    // ---- 摩擦测量（先于法向求解）----
    // 接触材料点自子步开始以来的切向相对位移。
    // 必须在法向修正前测量：法向修正的角部分会移动接触点，若在其后测量，
    // 该系统性切向分量会被摩擦当作滑动"消除"，形成推动刚体的正反馈（实测爬行 bug）。
    const curA = materialPoint(A, this.rALocal, tMA);
    const startA = materialPointStart(A, this.rALocal, tMC);
    const disp = tDisp.copy(curA).sub(startA);
    if (B !== null && !B.isStatic) {
      const curB = materialPoint(B, this.rBLocal, tMA);
      const startB = materialPointStart(B, this.rBLocal, tMC);
      disp.sub(tD1.copy(curB).sub(startB));
    }
    // 切向分量 dispT = disp − n·dot(disp, n)
    const dispT = tDispT.copy(disp).addScaled(n, -disp.dot(n));
    const dispTLen = dispT.length();

    // ---- 法向 ----
    let w = 0;
    if (!A.isStatic) {
      const rA = applyWorldR(A, this.rALocal, tRA);
      w += A.invMass + generalAngularInvMass(A.invIworld, rA, n, tRxN);
    }
    if (B !== null && !B.isStatic) {
      const rB = applyWorldR(B, this.rBLocal, tRB);
      w += B.invMass + generalAngularInvMass(B.invIworld, rB, n, tRxN);
    }
    if (w <= 1e-12) return;

    let lambda = this.depth / w; // 恒为正
    // 单次线修正钳制（防止深穿透弹飞）
    if (!A.isStatic && A.invMass > 0) lambda = Math.min(lambda, LINEAR_CORRECTION_CLAMP / A.invMass);
    if (B !== null && !B.isStatic && B.invMass > 0) lambda = Math.min(lambda, LINEAR_CORRECTION_CLAMP / B.invMass);

    if (!A.isStatic) {
      const rA = applyWorldR(A, this.rALocal, tRA);
      applyPointImpulse(A, rA, n, -lambda, tDPhi, tAxis, tRxN, tQuat);
    }
    if (B !== null && !B.isStatic) {
      const rB = applyWorldR(B, this.rBLocal, tRB);
      applyPointImpulse(B, rB, n, lambda, tDPhi, tAxis, tRxN, tQuat);
    }

    // ---- 摩擦（Macklin 位置法摩擦，用法向求解前测量的 dispT）----
    if (!applyFriction || dispTLen < 1e-10) return;

    const maxT = this.friction * this.depth;
    if (maxT <= 0) return;
    if (dispTLen > maxT) dispT.scale(maxT / dispTLen);

    // 单位化的切向方向
    const dir = tDir.copy(dispT).scale(1 / dispTLen);

    let wt = 0;
    if (!A.isStatic) {
      const rA = applyWorldR(A, this.rALocal, tRA);
      wt += A.invMass + generalAngularInvMass(A.invIworld, rA, dir, tRxN);
    }
    if (B !== null && !B.isStatic) {
      const rB = applyWorldR(B, this.rBLocal, tRB);
      wt += B.invMass + generalAngularInvMass(B.invIworld, rB, dir, tRxN);
    }
    if (wt <= 1e-12) return;

    // 消除相对滑动：A 沿 −dir，B 沿 +dir，标量 = |dispT| / wt
    const fLambda = dispTLen / wt;
    if (!A.isStatic) {
      const rA = applyWorldR(A, this.rALocal, tRA);
      applyPointImpulse(A, rA, dir, -fLambda, tDPhi, tAxis, tRxN, tQuat);
    }
    if (B !== null && !B.isStatic) {
      const rB = applyWorldR(B, this.rBLocal, tRB);
      applyPointImpulse(B, rB, dir, fLambda, tDPhi, tAxis, tRxN, tQuat);
    }
  }
}

// ---------------------------------------------------------------------------
// 内部工具（模块级临时对象，避免子步内分配）
// ---------------------------------------------------------------------------

const tRA = new MRVec3();
const tRB = new MRVec3();
const tRxN = new MRVec3();
const tDPhi = new MRVec3();
const tAxis = new MRVec3();
const tQuat = new MRQuat();
const tMA = new MRVec3();
const tMC = new MRVec3();
const tD1 = new MRVec3();
const tDisp = new MRVec3();
const tDispT = new MRVec3();
const tDir = new MRVec3();

/** rWorld = R · rLocal（用子步开始缓存的 rot；invIworld 同样是子步开始缓存，与 Müller 实现一致） */
function applyWorldR(body: MRBody, rLocal: Readonly<MRVec3>, out: MRVec3): MRVec3 {
  return body.rot.applyToVec3(rLocal, out);
}

/** 当前世界接触点：x + R·r_local */
function materialPoint(body: MRBody, rLocal: Readonly<MRVec3>, t1: MRVec3): MRVec3 {
  return body.rot.applyToVec3(rLocal, t1).add(body.x);
}

/** 子步开始的世界接触点：prevX + prevR·r_local */
function materialPointStart(body: MRBody, rLocal: Readonly<MRVec3>, t1: MRVec3): MRVec3 {
  return body.prevR.applyToVec3(rLocal, t1).add(body.prevX);
}

/**
 * 单点标量约束的角部分广义逆质量：(r × n)ᵀ · invI · (r × n)
 */
export function generalAngularInvMass(
  invI: Readonly<MRMat3>, r: Readonly<MRVec3>, n: Readonly<MRVec3>, tmpRxN: MRVec3,
): number {
  const rxn = tmpRxN.cross(r, n); // r × n
  const ix = invI.m[0] * rxn.x + invI.m[1] * rxn.y + invI.m[2] * rxn.z;
  const iy = invI.m[3] * rxn.x + invI.m[4] * rxn.y + invI.m[5] * rxn.z;
  const iz = invI.m[6] * rxn.x + invI.m[7] * rxn.y + invI.m[8] * rxn.z;
  return ix * rxn.x + iy * rxn.y + iz * rxn.z;
}

/**
 * 对刚体在锚点 r（COM→点，世界系）沿单位方向 n 施加标量脉冲 λ（可正可负）：
 * x += λ·invMass·n；Δφ = λ·invI·(r×n)（带 0.2 rad 钳制）；q ← axisAngle(Δφ)⊗q（左乘，世界系）。
 */
export function applyPointImpulse(
  body: MRBody,
  r: Readonly<MRVec3>,
  n: Readonly<MRVec3>,
  lambda: number,
  tmpDeltaPhi: MRVec3,
  tmpAxis: MRVec3,
  tmpRxN: MRVec3,
  tmpQuat: MRQuat,
): void {
  body.x.addScaled(n, lambda * body.invMass);

  // Δφ = λ · invI · (r × n)
  tmpRxN.cross(r, n);
  const I = body.invIworld.m;
  tmpDeltaPhi.set(
    (I[0] * tmpRxN.x + I[1] * tmpRxN.y + I[2] * tmpRxN.z) * lambda,
    (I[3] * tmpRxN.x + I[4] * tmpRxN.y + I[5] * tmpRxN.z) * lambda,
    (I[6] * tmpRxN.x + I[7] * tmpRxN.y + I[8] * tmpRxN.z) * lambda,
  );
  const phi = tmpDeltaPhi.length();
  if (phi > 1e-14) {
    const clamped = clamp(phi, 0, ANGULAR_CORRECTION_CLAMP);
    tmpAxis.copy(tmpDeltaPhi).scale(1 / phi);
    tmpQuat.fromAxisAngle(tmpAxis, clamped);
    body.q.multiply(tmpQuat, body.q).normalize();
    // 同步刷新 rot 缓存（同 rotateBodyAboutPoint）：后续约束求解需看到本次角修正
    body.rot.fromQuat(body.q);
  }
}
