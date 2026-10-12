/**
 * MRCollision —— 解析碰撞检测纯函数（core 层，禁止 import 'cc'）
 * 见设计文档 §3.6。
 *
 * 全部闭式解，无加速结构（13 体 ≈ 78 对，每子步全算微秒级）。
 *
 * 法线约定：n 从 A 指向 B（接触修正 A 沿 −n、B 沿 +n，见 MRContact）。
 *
 * 勘误说明（相对设计文档 §3.6.3）：capsule-ground 检测对线段两端点**各自独立**
 * 检测（depth>0 的端点各生成一个接触），而非只取最低者——只取单点时平躺的
 * capsule 会绕单接触点跷跷板抖动，无法满足 M1"静止无抖动"验收。
 * 有符号距离沿线段是线性函数，两端点检测对无限平面是充分的。
 */
import { MRVec3 } from './MRMath';
import { MRBody } from './MRBody';
import { MRCapsule } from './MRShape';
import { MRContact } from './MRContact';
import { IGround } from './MRGround';

// ---------------------------------------------------------------------------
// 线段-线段最近点（Ericson《Real-Time Collision Detection》§5.1.9，含平行退化）
// ---------------------------------------------------------------------------

const ssTmp1 = new MRVec3();
const ssTmp2 = new MRVec3();
const ssTmp3 = new MRVec3();

/**
 * 求线段 (p1,p2) 与 (q1,q2) 的最近点对。
 * 返回最近距离平方；最近点写入 outC1（第一段上）、outC2（第二段上）。
 */
export function closestPtSegmentSegment(
  p1: Readonly<MRVec3>, p2: Readonly<MRVec3>,
  q1: Readonly<MRVec3>, q2: Readonly<MRVec3>,
  outC1: MRVec3, outC2: MRVec3,
): number {
  const d1 = ssTmp1.copy(p2).sub(p1); // 方向向量，线段 1
  const d2 = ssTmp2.copy(q2).sub(q1); // 方向向量，线段 2
  const r = ssTmp3.copy(p1).sub(q1);
  const a = d1.lengthSqr(); // d1·d1
  const e = d2.lengthSqr(); // d2·d2
  const f = d2.dot(r);

  let s: number, t: number;
  const EPS = 1e-12;
  if (a <= EPS && e <= EPS) {
    // 两线段都退化为点
    s = 0; t = 0;
  } else if (a <= EPS) {
    // 线段 1 退化为点
    s = 0;
    t = clamp01(f / e);
  } else {
    const c = d1.dot(r);
    if (e <= EPS) {
      // 线段 2 退化为点
      t = 0;
      s = clamp01(-c / a);
    } else {
      const b = d1.dot(d2);
      const denom = a * e - b * b; // 恒非负
      // 平行退化（denom≈0）时用边界投影
      s = denom > EPS ? clamp01((b * f - c * e) / denom) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = clamp01(-c / a);
      } else if (t > 1) {
        t = 1;
        s = clamp01((b - c) / a);
      }
    }
  }

  outC1.copy(p1).addScaled(d1, s);
  outC2.copy(q1).addScaled(d2, t);
  return outC1.distanceSqrTo(outC2);
}

function clamp01(v: number): number {
  return v < 0 ? 0 : v > 1 ? 1 : v;
}

// ---------------------------------------------------------------------------
// capsule × capsule
// ---------------------------------------------------------------------------

const ccP1 = new MRVec3();
const ccP2 = new MRVec3();
const ccQ1 = new MRVec3();
const ccQ2 = new MRVec3();
const ccC1 = new MRVec3();
const ccC2 = new MRVec3();

/**
 * capsule × capsule 间距查询（负值 = 穿透深度），测试/诊断用。
 */
export function capsuleCapsuleDistance(capA: MRCapsule, capB: MRCapsule): number {
  capA.worldP1(ccP1);
  capA.worldP2(ccP2);
  capB.worldP1(ccQ1);
  capB.worldP2(ccQ2);
  const distSqr = closestPtSegmentSegment(ccP1, ccP2, ccQ1, ccQ2, ccC1, ccC2);
  return Math.sqrt(distSqr) - (capA.radius + capB.radius);
}

/**
 * capsule × capsule 检测。相交时填充 contact 并返回 true。
 * n 从 A 指向 B；接触点取各自线段最近点。
 */
export function capsuleCapsule(capA: MRCapsule, capB: MRCapsule, contact: MRContact): boolean {
  capA.worldP1(ccP1);
  capA.worldP2(ccP2);
  capB.worldP1(ccQ1);
  capB.worldP2(ccQ2);

  const distSqr = closestPtSegmentSegment(ccP1, ccP2, ccQ1, ccQ2, ccC1, ccC2);
  const rSum = capA.radius + capB.radius;
  if (distSqr >= rSum * rSum) return false;

  const dist = Math.sqrt(distSqr);
  contact.A = capA.body;
  contact.B = capB.body;
  if (dist > 1e-9) {
    contact.n.set(ccC2.x - ccC1.x, ccC2.y - ccC1.y, ccC2.z - ccC1.z).scale(1 / dist);
  } else {
    // 线段重合的退化情形：任取一个与 A 轴向正交的轴
    orthoAxisOf(capA, contact.n);
  }
  contact.depth = rSum - dist;
  worldToLocalPoint(capA.body, ccC1, contact.rALocal);
  worldToLocalPoint(capB.body, ccC2, contact.rBLocal);
  contact.friction = Math.min(capA.selfFriction, capB.selfFriction);
  return true;
}

// ---------------------------------------------------------------------------
// capsule × ground
// ---------------------------------------------------------------------------

const cgN = new MRVec3();

/**
 * capsule 端点 × 地面单点检测（真正的 Gauss-Seidel：调用方应在求解一个端点后
 * 用最新位姿检测下一个端点，见 MRSolver）。
 * depth>0 时填充 contact（A=capsule，B=null）并返回 true。
 */
export function detectCapsuleGroundPoint(
  cap: MRCapsule, p: Readonly<MRVec3>, ground: IGround, contact: MRContact,
): boolean {
  const h = ground.sampleHeight(p, cgN); // 相对地面的有符号高度，cgN = 地面法线（朝上）
  const depth = cap.radius - h;
  if (depth <= 0) return false;

  contact.A = cap.body;
  contact.B = null;
  // n 从 A（capsule）指向 B（地面）：取地面法线的反向
  contact.n.copy(cgN).negate();
  contact.depth = depth;
  worldToLocalPoint(cap.body, p, contact.rALocal);
  contact.rBLocal.set(0, 0, 0);
  contact.friction = cap.groundFriction;
  return true;
}

// ---------------------------------------------------------------------------
// 辅助
// ---------------------------------------------------------------------------

const w2lTmp = new MRVec3();

/** 世界点 → body 局部系（COM 为原点）：rLocal = Rᵀ · (p − x) */
export function worldToLocalPoint(body: MRBody, p: Readonly<MRVec3>, out: MRVec3): void {
  w2lTmp.set(p.x - body.x.x, p.y - body.x.y, p.z - body.x.z);
  // Rᵀ·v：行主序矩阵的转置乘向量 = 分量与列点积
  const m = body.rot.m;
  out.set(
    m[0] * w2lTmp.x + m[3] * w2lTmp.y + m[6] * w2lTmp.z,
    m[1] * w2lTmp.x + m[4] * w2lTmp.y + m[7] * w2lTmp.z,
    m[2] * w2lTmp.x + m[5] * w2lTmp.y + m[8] * w2lTmp.z,
  );
}

/** 与单位向量 v 正交的任一单位轴 */
export function orthoAxis(v: Readonly<MRVec3>, out: MRVec3): MRVec3 {
  if (Math.abs(v.x) < 0.9) {
    // 与 x 轴叉乘
    return out.set(0, v.z, -v.y).normalize();
  }
  return out.set(v.y, -v.x, 0).normalize();
}

function orthoAxisOf(cap: MRCapsule, out: MRVec3): MRVec3 {
  // capsule 轴向（局部 p2−p1 经 rot 变换）
  const ax = orthoTmp.copy(cap.p2Local).sub(cap.p1Local);
  cap.body.rot.applyToVec3(ax, ax);
  if (ax.lengthSqr() < 1e-14) return out.set(1, 0, 0);
  ax.normalize();
  return orthoAxis(ax, out);
}
const orthoTmp = new MRVec3();
