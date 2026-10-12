/**
 * M1 单刚体落地验收（设计文档 §8 M1）
 * - 单 capsule 从 5m 自由落体到平面：3s 后 |v|<0.01、穿透<1e-3、静止无抖动、无 NaN
 * - 摩擦方向实证：30° 斜面 μ=0.7 不滑 / μ=0.4 滑
 * - M1b：盒惯性竖杆倒下翻滚自然（躺平静止）
 */
import { describe, it, expect } from 'vitest';
import { MRVec3 } from '../assets/math-ragdoll/core/MRMath';
import { MRBody } from '../assets/math-ragdoll/core/MRBody';
import { MRCapsule, boxInvInertia } from '../assets/math-ragdoll/core/MRShape';
import { MRGroundPlane } from '../assets/math-ragdoll/core/MRGround';
import { MRSolver } from '../assets/math-ragdoll/core/MRSolver';

const FIXED_DT = 1 / 60;

interface SingleBodyScene {
  solver: MRSolver;
  body: MRBody;
  capsule: MRCapsule;
}

/** 构建单体 + 地面场景（无自碰撞对） */
function makeSingleBodyCapsule(opts: {
  comY: number;
  radius: number;
  halfLen: number;
  axis: 'x' | 'y' | 'z';
  mass: number;
  ground?: MRGroundPlane;
}): SingleBodyScene {
  const solver = new MRSolver();
  const body = new MRBody();
  body.name = 'cap';
  body.invMass = 1 / opts.mass;
  body.x.set(0, opts.comY, 0);

  const cap = new MRCapsule(body);
  cap.name = 'cap';
  cap.radius = opts.radius;
  const p = new MRVec3();
  if (opts.axis === 'x') {
    cap.p1Local.set(-opts.halfLen, 0, 0);
    cap.p2Local.set(opts.halfLen, 0, 0);
  } else if (opts.axis === 'y') {
    cap.p1Local.set(0, -opts.halfLen, 0);
    cap.p2Local.set(0, opts.halfLen, 0);
  } else {
    cap.p1Local.set(0, 0, -opts.halfLen);
    cap.p2Local.set(0, 0, opts.halfLen);
  }
  // 盒惯性（capsule 包围盒）
  const hx = (opts.axis === 'x' ? opts.halfLen : 0) + opts.radius;
  const hy = (opts.axis === 'y' ? opts.halfLen : 0) + opts.radius;
  const hz = (opts.axis === 'z' ? opts.halfLen : 0) + opts.radius;
  boxInvInertia(hx, hy, hz, opts.mass, body.invIlocal);

  body.refreshDerived();
  body.prevX.copy(body.x);
  body.prevQ.copy(body.q);
  body.prevR.copy(body.rot);

  solver.bodies.push(body);
  solver.capsules.push(cap);
  solver.ground = opts.ground ?? new MRGroundPlane(0);
  return { solver, body, capsule: cap };
}

function runSeconds(solver: MRSolver, seconds: number): void {
  const steps = Math.round(seconds / FIXED_DT);
  for (let i = 0; i < steps; i++) solver.step(FIXED_DT);
}

function allFinite(solver: MRSolver): boolean {
  for (const b of solver.bodies) {
    if (!b.x.isFinite() || !b.q.isFinite() || !b.v.isFinite() || !b.w.isFinite()) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
describe('M1: 单 capsule 5m 自由落体', () => {
  // 水平 capsule（线段沿 x，半长 0.3，r=0.1）：落地后两端点支撑，稳定平躺
  const scene = makeSingleBodyCapsule({ comY: 5, radius: 0.1, halfLen: 0.3, axis: 'x', mass: 10 });
  runSeconds(scene.solver, 3);

  it('3s 后速度趋于零且穿透 < 1e-3', () => {
    expect(scene.body.v.length()).toBeLessThan(0.01);
    // 最低点（两端点）不低于地面 −1e-3
    const p1 = scene.capsule.worldP1(new MRVec3());
    const p2 = scene.capsule.worldP2(new MRVec3());
    const pen = 0.1 - Math.min(p1.y, p2.y);
    expect(pen).toBeLessThan(1e-3);
  });

  it('全程无 NaN', () => {
    expect(allFinite(scene.solver)).toBe(true);
  });

  it('静止 10s 无抖动（后段位移方差 < 1e-6）', () => {
    runSeconds(scene.solver, 10); // 累计 13s
    // 采样最后 3s 的 COM 位移
    const startX = scene.body.x.clone();
    let maxDispSqr = 0;
    for (let i = 0; i < 180; i++) {
      scene.solver.step(FIXED_DT);
      maxDispSqr = Math.max(maxDispSqr, scene.body.x.distanceSqrTo(startX));
    }
    expect(maxDispSqr).toBeLessThan(1e-6);
  });
});

// ---------------------------------------------------------------------------
describe('M1: 摩擦方向实证（30° 斜面）', () => {
  // 斜面：绕 z 轴倾斜 30°，法线朝上偏 −x
  const sin30 = Math.sin(Math.PI / 6);
  const cos30 = Math.cos(Math.PI / 6);
  const slope = MRGroundPlane.fromPointNormal(0, 0, 0, -sin30, cos30, 0);

  // capsule 线段沿 z（平行坡的走向），COM 沿法线抬升 r —— 初始恰好接触
  function makeOnSlope(mu: number): SingleBodyScene {
    const scene = makeSingleBodyCapsule({ comY: 0, radius: 0.1, halfLen: 0.25, axis: 'z', mass: 5, ground: slope });
    scene.capsule.groundFriction = mu;
    scene.body.x.set(-sin30 * 0.1, cos30 * 0.1, 0);
    scene.body.refreshDerived();
    scene.body.prevX.copy(scene.body.x);
    scene.body.prevR.copy(scene.body.rot);
    return scene;
  }

  it('μ=0.7（> tan30°）不滑', () => {
    const scene = makeOnSlope(0.7);
    const x0 = scene.body.x.clone();
    runSeconds(scene.solver, 2);
    const disp = scene.body.x.distanceTo(x0);
    expect(disp).toBeLessThan(0.05);
    expect(allFinite(scene.solver)).toBe(true);
  });

  it('μ=0.4（< tan30°）下滑', () => {
    const scene = makeOnSlope(0.4);
    const x0 = scene.body.x.clone();
    runSeconds(scene.solver, 2);
    const disp = scene.body.x.distanceTo(x0);
    expect(disp).toBeGreaterThan(0.2);
    expect(allFinite(scene.solver)).toBe(true);
  });
});

// ---------------------------------------------------------------------------
describe('M1b: 盒惯性竖杆倒下', () => {
  it('细长竖杆从微倾斜倒下后躺平静止', () => {
    const solver = new MRSolver();
    const body = new MRBody();
    body.name = 'rod';
    body.invMass = 1 / 5;
    // 竖长杆：半长 0.5，r=0.05 → 盒半尺寸 (0.05, 0.55, 0.05)，绕 z 微倾 5°
    boxInvInertia(0.05, 0.55, 0.05, 5, body.invIlocal);
    body.x.set(0, 0.7, 0);
    const tilt = 5 * Math.PI / 180;
    body.q.set(0, 0, Math.sin(tilt / 2), Math.cos(tilt / 2));
    body.refreshDerived();
    body.prevX.copy(body.x);
    body.prevQ.copy(body.q);
    body.prevR.copy(body.rot);

    const cap = new MRCapsule(body);
    cap.radius = 0.05;
    cap.p1Local.set(0, -0.5, 0);
    cap.p2Local.set(0, 0.5, 0);
    cap.groundFriction = 0.7;

    solver.bodies.push(body);
    solver.capsules.push(cap);
    solver.ground = new MRGroundPlane(0);

    runSeconds(solver, 4);

    // 躺平：COM 高度接近 r（允许倒下过程的残余滚动）
    expect(Math.abs(body.x.y - 0.05)).toBeLessThan(0.06);
    // 静止
    expect(body.v.length()).toBeLessThan(0.02);
    expect(body.w.length()).toBeLessThan(0.05);
    // 翻滚自然（角度接近水平：竖轴与地面夹角 < 10°）
    const upW = body.rot.applyToVec3(new MRVec3(0, 1, 0), new MRVec3());
    expect(Math.abs(upW.y)).toBeLessThan(Math.sin(10 * Math.PI / 180));
    expect(allFinite(solver)).toBe(true);
  });
});
