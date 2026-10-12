/**
 * M2 关节链验收（设计文档 §8 M2）
 * - 2 体球关节单摆：锚点距离误差 < 1mm，能量单调衰减（有阻尼）
 * - 铰链关节：只绕铰链轴弯，其他自由度锁死（最大偏移 < 1°）
 * - N=8 链甩动 10s 无爆炸、无 NaN
 */
import { describe, it, expect } from 'vitest';
import { MRVec3 } from '../assets/math-ragdoll/core/MRMath';
import { MRBody } from '../assets/math-ragdoll/core/MRBody';
import { MRCapsule, boxInvInertia } from '../assets/math-ragdoll/core/MRShape';
import { MRBallJoint, MRHingeJoint } from '../assets/math-ragdoll/core/MRJoint';
import { MRSolver } from '../assets/math-ragdoll/core/MRSolver';

const FIXED_DT = 1 / 60;

/** 构造一个 1kg 单体（无碰撞形状，纯关节测试用） */
function makeBody(name: string, x: number, y: number, z: number, mass = 1, isStatic = false): MRBody {
  const b = new MRBody();
  b.name = name;
  if (isStatic) {
    b.makeStatic();
  } else {
    b.invMass = 1 / mass;
    boxInvInertia(0.05, 0.05, 0.05, mass, b.invIlocal);
  }
  b.x.set(x, y, z);
  b.refreshDerived();
  b.prevX.copy(b.x);
  b.prevQ.copy(b.q);
  b.prevR.copy(b.rot);
  return b;
}

/** 锚点距离：joint 两体锚点世界位置的偏差 */
function anchorError(j: MRBallJoint | MRHingeJoint): number {
  const pA = j.A.rot.applyToVec3(j.rALocal, new MRVec3()).add(j.A.x);
  const pB = j.B.rot.applyToVec3(j.rBLocal, new MRVec3()).add(j.B.x);
  return pA.distanceTo(pB);
}

// ---------------------------------------------------------------------------
describe('M2: 2 体球关节单摆', () => {
  // A 静态锚点 (0,2,0)；B 1kg 挂在下方 1m，初始拉到水平 (1,2,0) 释放
  const solver = new MRSolver();
  const A = makeBody('anchor', 0, 2, 0, 1, true);
  const B = makeBody('bob', 1, 2, 0, 1);
  const joint = new MRBallJoint(A, B);
  joint.rALocal.set(0, 0, 0); // A 的 COM 即锚点
  joint.rBLocal.set(-1, 0, 0); // B 的 COM 到锚点（B 局部系 = 世界系，bind q=I）
  solver.bodies.push(A, B);
  solver.ballJoints.push(joint);
  solver.linDamp = 0.05;
  solver.angDamp = 0.05;

  const STEPS = 600; // 10s
  const energies: number[] = [];
  let maxAnchorErr = 0;
  for (let i = 0; i <= STEPS; i++) {
    // E = ½mv² + mg·h（h 相对最低点 y=1）
    const E = 0.5 * B.v.lengthSqr() + 9.81 * (B.x.y - 1);
    energies.push(E);
    maxAnchorErr = Math.max(maxAnchorErr, anchorError(joint));
    solver.step(FIXED_DT);
  }

  it('锚点距离误差 < 1mm（全程最大值）', () => {
    expect(maxAnchorErr).toBeLessThan(1e-3);
  });

  it('能量单调衰减（有阻尼，允许微小数值毛刺）', () => {
    const e0 = energies[0];
    const eEnd = energies[energies.length - 1];
    expect(eEnd).toBeLessThan(e0 * 0.7); // 10s 阻尼显著衰减
    // 逐点检查无显著回升（每步回升 < 初始能量的 0.1%）
    for (let i = 1; i < energies.length; i++) {
      expect(energies[i]).toBeLessThan(energies[i - 1] + e0 * 0.001);
    }
  });

  it('无 NaN', () => {
    for (const b of solver.bodies) {
      expect(b.x.isFinite()).toBe(true);
      expect(b.q.isFinite()).toBe(true);
    }
  });
});

// ---------------------------------------------------------------------------
describe('M2: 铰链关节自由度锁死', () => {
  // A 静态，B 通过铰链连接（铰链轴 = 世界 x 轴），初始 B 绕非铰链轴（y）扭转 20°，
  // 角对齐约束应把扭转拉回，使 B 只能绕 x 轴转。
  function makeHingePivot(): { solver: MRSolver; A: MRBody; B: MRBody; hinge: MRHingeJoint } {
    const solver = new MRSolver();
    const A = makeBody('base', 0, 1, 0, 1, true);
    const B = makeBody('swing', 0, 0, 0, 1); // B COM 在锚点下方 1m
    const hinge = new MRHingeJoint(A, B);
    hinge.rALocal.set(0, 0, 0);
    hinge.rBLocal.set(0, 1, 0);
    hinge.hingeAxisALocal.set(1, 0, 0); // 铰链轴 = x
    hinge.hingeAxisBLocal.set(1, 0, 0);
    hinge.refAxisALocal.set(0, 0, -1); // 参考垂直轴（bind 时 B 的远端方向 −y 与之垂直）
    hinge.refAxisBLocal.set(0, -1, 0); // B 的零位参考 = 摆杆方向（bind 对齐）
    hinge.aMin = -Math.PI;
    hinge.aMax = Math.PI;
    solver.bodies.push(A, B);
    solver.hinges.push(hinge);
    return { solver, A, B, hinge };
  }

  it('初始绕 y 扭转 20°，2s 内被角对齐拉回（末段铰链轴偏差 < 1°）', () => {
    const { solver, B, hinge } = makeHingePivot();
    // B 绕 y 扭 20°（绕其局部 y 轴，含摆杆方向）
    const half = (10 * Math.PI) / 180;
    B.q.set(0, Math.sin(half), 0, Math.cos(half));
    B.refreshDerived();

    // 末段（最后 1s）最大偏差：稳态锁死判定（摆动瞬态允许滞后）
    let maxAxisDev = 0;
    for (let i = 0; i < 240; i++) {
      solver.step(FIXED_DT);
      if (i >= 120) {
        const uA = hinge.A.rot.applyToVec3(hinge.hingeAxisALocal, new MRVec3());
        const uB = hinge.B.rot.applyToVec3(hinge.hingeAxisBLocal, new MRVec3());
        const dev = Math.acos(Math.max(-1, Math.min(1, uA.dot(uB))));
        maxAxisDev = Math.max(maxAxisDev, dev);
      }
    }
    expect(maxAxisDev).toBeLessThan((1 * Math.PI) / 180);
    // B 仍在锚点附近（位置约束成立）
    const pB = hinge.B.rot.applyToVec3(hinge.rBLocal, new MRVec3()).add(hinge.B.x);
    const pA = hinge.A.rot.applyToVec3(hinge.rALocal, new MRVec3()).add(hinge.A.x);
    expect(pA.distanceTo(pB)).toBeLessThan(1e-3);
    expect(B.x.isFinite()).toBe(true);
  });

  it('单摆摆动只绕铰链轴（给垂直于铰链轴的初始角速度，轴偏差保持 < 1°）', () => {
    const { solver, B, hinge } = makeHingePivot();
    B.w.set(0, 2, 0); // 绕 y 的角速度扰动（垂直于铰链轴 x）
    let maxAxisDev = 0;
    for (let i = 0; i < 240; i++) {
      solver.step(FIXED_DT);
      const uA = hinge.A.rot.applyToVec3(hinge.hingeAxisALocal, new MRVec3());
      const uB = hinge.B.rot.applyToVec3(hinge.hingeAxisBLocal, new MRVec3());
      maxAxisDev = Math.max(maxAxisDev, Math.acos(Math.max(-1, Math.min(1, uA.dot(uB)))));
    }
    expect(maxAxisDev).toBeLessThan((1 * Math.PI) / 180);
  });
});

// ---------------------------------------------------------------------------
describe('M2: N=8 链甩动', () => {
  it('10s 无爆炸、无 NaN、关节保持连接', () => {
    const N = 8;
    const solver = new MRSolver();
    const bodies: MRBody[] = [];
    // 首体静态（锚点在 y=3），其余竖直下挂，链节 0.5m
    for (let i = 0; i <= N; i++) {
      const b = makeBody(`link${i}`, 0, 3 - i * 0.5, 0, 1, i === 0);
      bodies.push(b);
      solver.bodies.push(b);
      if (i > 0) {
        const j = new MRBallJoint(bodies[i - 1], bodies[i]);
        j.rALocal.set(0, -0.25, 0);
        j.rBLocal.set(0, 0.25, 0);
        solver.ballJoints.push(j);
      }
    }
    // 甩动：给末端一个初速度
    bodies[N].v.set(5, 0, 0);

    let maxJointErr = 0;
    for (let i = 0; i < 600; i++) {
      // 10s
      solver.step(FIXED_DT);
      for (const j of solver.ballJoints) maxJointErr = Math.max(maxJointErr, anchorError(j));
    }
    expect(maxJointErr).toBeLessThan(5e-3);
    for (const b of bodies) {
      expect(b.x.isFinite() && b.q.isFinite() && b.v.isFinite() && b.w.isFinite()).toBe(true);
      // 无爆炸：速度被钳制在求解器上限内
      expect(b.v.length()).toBeLessThan(30.001);
      expect(b.w.length()).toBeLessThan(60.001);
    }
  });
});
