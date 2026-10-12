/**
 * M3 全身被动布娃娃验收（设计文档 §8 M3，核心里程碑）
 * - 合成 13 体 SkeletonDef，T-pose 1m 落地：(a) 3s 内静止；(b) 静止时所有限位满足（超差 <2°）；
 *   (c) 全程过滤表内 capsule 对穿透 < 1mm；(d) 60s 无 NaN —— 全部自动断言
 * - 限位符号实证：逐关节超限启动，一步内回到限内
 * - 确定性：同输入跑两遍，位置序列 hash 相同
 * - 性能：60s 模拟（60Hz×8 子步）挂钟 < 3s
 */
import { describe, it, expect } from 'vitest';
import { MRVec3 } from '../assets/math-ragdoll/core/MRMath';
import { MRBody } from '../assets/math-ragdoll/core/MRBody';
import { boxInvInertia } from '../assets/math-ragdoll/core/MRShape';
import { createDefaultHumanoidDef, instantiateSkeleton, MRRagdollInstance } from '../assets/math-ragdoll/core/MRDefaultSkeleton';
import { capsuleCapsuleDistance } from '../assets/math-ragdoll/core/MRCollision';
import { MRSolver } from '../assets/math-ragdoll/core/MRSolver';
import { MRGroundPlane } from '../assets/math-ragdoll/core/MRGround';
import { MRConeTwistLimit, getHingeRangeAngle, solveHingeRange } from '../assets/math-ragdoll/core/MRLimit';
import { MRHingeJoint } from '../assets/math-ragdoll/core/MRJoint';

const FIXED_DT = 1 / 60;
const RAD2DEG = 180 / Math.PI;

/** 构建下落 dropHeight 米的 T-pose 布娃娃场景 */
function makeDropRagdoll(dropHeight = 1): MRRagdollInstance {
  const inst = instantiateSkeleton(createDefaultHumanoidDef(70));
  for (const b of inst.solver.bodies) b.x.y += dropHeight;
  inst.solver.ground = new MRGroundPlane(0);
  return inst;
}

function stepSeconds(solver: MRSolver, seconds: number): void {
  const steps = Math.round(seconds / FIXED_DT);
  for (let i = 0; i < steps; i++) solver.step(FIXED_DT);
}

function allFinite(solver: MRSolver): boolean {
  return solver.bodies.every(
    (b) => b.x.isFinite() && b.q.isFinite() && b.v.isFinite() && b.w.isFinite(),
  );
}

function maxSpeed(solver: MRSolver): number {
  let m = 0;
  for (const b of solver.bodies) m = Math.max(m, b.v.length());
  return m;
}

/** 测试用绕点旋转（与 core 语义一致：x' = a + Rot·(x−a)，q ← axisAngle⊗q） */
function rotateBodyAboutPoint(
  body: { x: MRVec3; q: { x: number; y: number; z: number; w: number } },
  anchor: MRVec3, axis: MRVec3, angle: number,
): void {
  const rx = body.x.x - anchor.x, ry = body.x.y - anchor.y, rz = body.x.z - anchor.z;
  const cos = Math.cos(angle), sin = Math.sin(angle), one = 1 - cos;
  const kdotv = axis.x * rx + axis.y * ry + axis.z * rz;
  const cx = axis.y * rz - axis.z * ry;
  const cy = axis.z * rx - axis.x * rz;
  const cz = axis.x * ry - axis.y * rx;
  body.x.set(
    anchor.x + rx * cos + cx * sin + axis.x * kdotv * one,
    anchor.y + ry * cos + cy * sin + axis.y * kdotv * one,
    anchor.z + rz * cos + cz * sin + axis.z * kdotv * one,
  );
  const half = angle / 2, s = Math.sin(half);
  const qx = axis.x * s, qy = axis.y * s, qz = axis.z * s, qw = Math.cos(half);
  const q = body.q;
  const nx = qw * q.x + qx * q.w + qy * q.z - qz * q.y;
  const ny = qw * q.y + qy * q.w + qz * q.x - qx * q.z;
  const nz = qw * q.z + qz * q.w + qx * q.y - qy * q.x;
  const nw = qw * q.w - qx * q.x - qy * q.y - qz * q.z;
  const len = Math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw) || 1;
  q.x = nx / len; q.y = ny / len; q.z = nz / len; q.w = nw / len;
}

// ---------------------------------------------------------------------------
describe('M3: 13 体 T-pose 1m 落地', () => {
  const inst = makeDropRagdoll(1);
  const solver = inst.solver;

  stepSeconds(solver, 3);

  it('(a) 3s 内静止（所有体 |v| < 0.05 m/s）', () => {
    expect(maxSpeed(solver)).toBeLessThan(0.05);
  });

  it('(b) 静止时所有限位满足（超差 < 2°）', () => {
    for (const lim of inst.limits) {
      const dev = lim.getCurrentDeviation();
      expect(dev.coneAngle * RAD2DEG, `${lim.name} cone`).toBeLessThanOrEqual(lim.coneMax * RAD2DEG + 2);
      expect(dev.twistAngle * RAD2DEG, `${lim.name} twist`).toBeGreaterThanOrEqual(lim.twistMin * RAD2DEG - 2);
      expect(dev.twistAngle * RAD2DEG, `${lim.name} twist`).toBeLessThanOrEqual(lim.twistMax * RAD2DEG + 2);
    }
    for (const hinge of inst.hinges) {
      const am = getHingeRangeAngle(hinge) * RAD2DEG;
      expect(am, `${hinge.name} range`).toBeGreaterThanOrEqual(hinge.aMin * RAD2DEG - 2);
      expect(am, `${hinge.name} range`).toBeLessThanOrEqual(hinge.aMax * RAD2DEG + 2);
    }
  });

  it('(c)(d) 全程过滤表内 capsule 对穿透 < 1mm 且 60s 无 NaN', () => {
    const violations: { t: number; pair: string; pen: number }[] = [];
    const totalSteps = Math.round(57 / FIXED_DT);
    let nan = false;
    for (let i = 0; i < totalSteps; i++) {
      solver.step(FIXED_DT);
      if (!allFinite(solver)) {
        nan = true;
        break;
      }
      if (i % 6 === 0) {
        const t = 3 + i * FIXED_DT;
        for (let p = 0; p < solver.collisionPairs.length; p++) {
          const [ia, ib] = solver.collisionPairs[p];
          const dist = capsuleCapsuleDistance(solver.capsules[ia], solver.capsules[ib]);
          if (dist < -1e-3) {
            violations.push({
              t,
              pair: `${solver.capsules[ia].name}|${solver.capsules[ib].name}`,
              pen: -dist,
            });
          }
        }
      }
    }
    expect(nan).toBe(false);
    expect(violations).toEqual([]);
  });
});

// ---------------------------------------------------------------------------
describe('M3: 限位符号实证（逐关节超限启动）', () => {
  /**
   * 纯限位单元语义：两体 + 单个限位，手动超限后循环调用限位求解
   * （次数 = 1 个固定步的子步数），验证修正方向正确且收敛到限内。
   * 不引入重力/其他关节干扰 —— 本测试验证的是限位修正的方向与收敛性本身。
   */
  const SUBSTEPS = 8;
  const tolerance = 0.5 * Math.PI / 180;

  function makePair(): { A: MRBody; B: MRBody } {
    const A = new MRBody();
    A.name = 'A';
    A.makeStatic();
    A.x.set(0, 1, 0);
    A.refreshDerived();
    const B = new MRBody();
    B.name = 'B';
    B.invMass = 1 / 5;
    boxInvInertia(0.07, 0.21, 0.07, 5, B.invIlocal);
    B.x.set(0, 0.7, 0); // A 下方 0.3m
    B.refreshDerived();
    return { A, B };
  }

  it('锥面限位：超限 coneMax+10° 启动，1 个固定步内回到限内（0.5°）', () => {
    const coneMax = 30 * Math.PI / 180;
    const { A, B } = makePair();
    const lim = new MRConeTwistLimit(A, B);
    lim.axisALocal.set(0, -1, 0); // A 指向 B（下）
    lim.axisBLocal.set(0, -1, 0);
    lim.twistRefALocal.set(1, 0, 0);
    lim.twistRefBLocal.set(1, 0, 0);
    lim.coneMax = coneMax;
    lim.twistMin = -30 * Math.PI / 180;
    lim.twistMax = 30 * Math.PI / 180;

    // B 绕 z 轴转出锥面（coneMax + 10°，锚点 = A 位置）
    const over = coneMax + 10 * Math.PI / 180;
    rotateBodyAboutPoint(B, A.x, new MRVec3(0, 0, 1), over);
    B.refreshDerived();

    let before = lim.getCurrentDeviation();
    expect(before.coneAngle, '应已超限').toBeGreaterThan(coneMax);
    for (let s = 0; s < SUBSTEPS; s++) lim.solve();
    before = lim.getCurrentDeviation();
    expect(before.coneAngle * RAD2DEG, `cone 1 步后 ${before.coneAngle * RAD2DEG}°`)
      .toBeLessThanOrEqual(coneMax * RAD2DEG + tolerance * RAD2DEG);
    // 扭转保持界内
    expect(before.twistAngle * RAD2DEG).toBeLessThanOrEqual(lim.twistMax * RAD2DEG + tolerance * RAD2DEG);
    expect(before.twistAngle * RAD2DEG).toBeGreaterThanOrEqual(lim.twistMin * RAD2DEG - tolerance * RAD2DEG);
  });

  it('扭转限位：超限 twistMax+10° 启动，1 个固定步内回到限内（0.5°）', () => {
    const twistMax = 30 * Math.PI / 180;
    const { A, B } = makePair();
    const lim = new MRConeTwistLimit(A, B);
    lim.axisALocal.set(0, -1, 0);
    lim.axisBLocal.set(0, -1, 0);
    lim.twistRefALocal.set(1, 0, 0);
    lim.twistRefBLocal.set(1, 0, 0);
    lim.coneMax = 60 * Math.PI / 180; // 锥面放宽，专注扭转
    lim.twistMin = -twistMax;
    lim.twistMax = twistMax;

    // B 绕主轴（y）扭转超限
    const over = twistMax + 10 * Math.PI / 180;
    rotateBodyAboutPoint(B, B.x, new MRVec3(0, 1, 0), over); // 绕自身 COM 主轴转
    B.refreshDerived();

    const before = lim.getCurrentDeviation();
    // 符号约定：绕 y 正向旋转在测量中体现为负角，超限判定按两侧界
    expect(
      before.twistAngle > twistMax || before.twistAngle < -twistMax,
      `应已超限（实际 ${before.twistAngle * RAD2DEG}°）`,
    ).toBe(true);
    for (let s = 0; s < SUBSTEPS; s++) lim.solve();
    const after = lim.getCurrentDeviation();
    expect(Math.abs(after.twistAngle) * RAD2DEG, `twist 1 步后 ${after.twistAngle * RAD2DEG}°`)
      .toBeLessThanOrEqual(twistMax * RAD2DEG + tolerance * RAD2DEG);
  });

  it('铰链行程限位：超限启动，1 个固定步内回到限内（0.5°）', () => {
    const aMax = 90 * Math.PI / 180;
    const { A, B } = makePair();
    const hinge = new MRHingeJoint(A, B);
    hinge.rALocal.set(0, -0.3, 0); // A COM→锚点（下 0.3m）
    hinge.rBLocal.set(0, 0.3, 0); // B COM→锚点（上 0.3m）
    hinge.hingeAxisALocal.set(1, 0, 0);
    hinge.hingeAxisBLocal.set(1, 0, 0);
    hinge.refAxisALocal.set(0, -1, 0); // 零位：B 在 A 正下方
    hinge.refAxisBLocal.set(0, -1, 0);
    hinge.aMin = 0;
    hinge.aMax = aMax;

    // B 绕铰链轴（x）转到超限（aMax + 15°）
    const anchor = A.x.clone().add(new MRVec3(0, -0.3, 0));
    const over = aMax + 15 * Math.PI / 180;
    rotateBodyAboutPoint(B, anchor, new MRVec3(1, 0, 0), over);
    B.refreshDerived();

    let before = getHingeRangeAngle(hinge);
    expect(before, '应已超限').toBeGreaterThan(aMax);
    for (let s = 0; s < SUBSTEPS; s++) {
      hinge.solveAngleAlignment();
      solveHingeRange(hinge);
    }
    const after = getHingeRangeAngle(hinge);
    expect(after * RAD2DEG, `range 1 步后 ${after * RAD2DEG}°`)
      .toBeLessThanOrEqual(aMax * RAD2DEG + tolerance * RAD2DEG);
  });
});

// ---------------------------------------------------------------------------
describe('M3: 确定性', () => {
  it('同输入跑两遍，位置序列 hash 相同', () => {
    const run = (): string => {
      const inst = makeDropRagdoll(1);
      let hash = 0x811c9dc5;
      for (let i = 0; i < 600; i++) {
        inst.solver.step(FIXED_DT);
        for (const b of inst.solver.bodies) {
          for (const v of [b.x.x, b.x.y, b.x.z]) {
            const buf = new Float64Array(1);
            buf[0] = v;
            const bytes = new Uint8Array(buf.buffer);
            for (const byte of bytes) {
              hash ^= byte;
              hash = Math.imul(hash, 0x01000193) >>> 0;
            }
          }
        }
      }
      return hash.toString(16);
    };
    expect(run()).toBe(run());
  });
});

// ---------------------------------------------------------------------------
describe('M3: 性能', () => {
  it('60s 模拟（60Hz×8 子步）挂钟 < 3s', () => {
    const inst = makeDropRagdoll(1);
    const start = performance.now();
    stepSeconds(inst.solver, 60);
    const elapsed = performance.now() - start;
    expect(elapsed).toBeLessThan(3000);
    expect(allFinite(inst.solver)).toBe(true);
  });
});
