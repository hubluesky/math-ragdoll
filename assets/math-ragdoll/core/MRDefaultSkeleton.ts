/**
 * MRDefaultSkeleton —— 标准人形 SkeletonDef（13 体）+ 纯 TS 实例化构建器
 *
 * 查漏补缺说明（相对设计文档）：
 * - 文档的 MRRagdollBuilder 在 runtime 层（依赖 cc）；core 层测试（M3）需要
 *   "SkeletonDef → MRBody 图"的纯 TS 实例化，runtime builder 同样复用它。
 * - BodyDef 补充 bindPos（COM 的 bind 世界位置）与锥扭参考轴 axisA/axisB
 *   （原文档 JointDef/BodyDef 缺失，无法从纯拓扑数据构建限位与位姿）。
 * - 体数口径：文档 §3.6 说"13 体 ≈ 78 对"，§4.1/§7.1 又将手/脚并入前臂/小腿
 *   （11 体）。本定义采用 13 体口径：手独立成体、脚并入小腿（含于 shin 质量），
 *   C(13,2)=78 与文档对数一致。
 *
 * 人体几何：1.75m 基准（§7.1），Y-up，面向 +z，T-pose bind，bind 旋转 = identity
 * （body 局部系 = 世界系）。
 */
import { MRVec3, MRQuat } from './MRMath';
import { MRBody } from './MRBody';
import { MRCapsule, boxInvInertia } from './MRShape';
import { MRBallJoint, MRHingeJoint } from './MRJoint';
import { MRConeTwistLimit, DEFAULT_TWIST_DAMP, DEFAULT_TWIST_STIFFNESS } from './MRLimit';
import { MRSolver } from './MRSolver';
import { BodyDef, JointDef, SkeletonDef, buildCollisionPairs } from './MRSkeletonDef';

// ---------------------------------------------------------------------------
// 标准人形定义
// ---------------------------------------------------------------------------

/** 每体的 bind COM 世界位置（T-pose） */
const BIND_POS: Record<string, [number, number, number]> = {
  pelvis: [0, 1.01, 0],
  chest: [0, 1.28, 0],
  head: [0, 1.57, 0],
  upperArmL: [-0.35, 1.42, 0],
  upperArmR: [0.35, 1.42, 0],
  forearmL: [-0.64, 1.42, 0],
  forearmR: [0.64, 1.42, 0],
  handL: [-0.86, 1.42, 0],
  handR: [0.86, 1.42, 0],
  thighL: [-0.09, 0.71, 0],
  thighR: [0.09, 0.71, 0],
  shinL: [-0.09, 0.28, 0],
  shinR: [0.09, 0.28, 0],
};

function body(
  name: string, parent: string | null, massRatio: number,
  p1: [number, number, number], p2: [number, number, number], radius: number,
): BodyDef {
  return {
    name, parent, massRatio,
    capsule: { p1Local: p1, p2Local: p2, radius },
    bindPos: BIND_POS[name],
  };
}

/** 13 体标准人形（T-pose，§7.1 质量比例与 capsule 半径基准） */
export function createDefaultHumanoidDef(totalMass = 70): SkeletonDef {
  const bodies: BodyDef[] = [
    body('pelvis', null, 15, [0, -0.09, 0], [0, 0.09, 0], 0.11),
    body('chest', 'pelvis', 30, [0, -0.18, 0], [0, 0.18, 0], 0.12),
    body('head', 'chest', 8, [0, -0.11, 0], [0, 0.11, 0], 0.11),
    // 左 = −x
    body('upperArmL', 'chest', 3.0, [-0.15, 0, 0], [0.15, 0, 0], 0.05),
    body('forearmL', 'upperArmL', 2.0, [-0.14, 0, 0], [0.14, 0, 0], 0.045),
    body('handL', 'forearmL', 0.8, [-0.08, 0, 0], [0.08, 0, 0], 0.04),
    body('thighL', 'pelvis', 10.0, [0, -0.21, 0], [0, 0.21, 0], 0.075),
    body('shinL', 'thighL', 5.7, [0, -0.22, 0], [0, 0.22, 0], 0.06),
    // 右 = +x
    body('upperArmR', 'chest', 3.0, [-0.15, 0, 0], [0.15, 0, 0], 0.05),
    body('forearmR', 'upperArmR', 2.0, [-0.14, 0, 0], [0.14, 0, 0], 0.045),
    body('handR', 'forearmR', 0.8, [-0.08, 0, 0], [0.08, 0, 0], 0.04),
    body('thighR', 'pelvis', 10.0, [0, -0.21, 0], [0, 0.21, 0], 0.075),
    body('shinR', 'thighR', 5.7, [0, -0.22, 0], [0, 0.22, 0], 0.06),
  ];

  const joints: JointDef[] = [
    // 腰（§7.2：锥扭 30°，twist ±30°）
    { type: 'ball', a: 'pelvis', b: 'chest', limit: { coneMax: 30 * DEG, twistMin: -30 * DEG, twistMax: 30 * DEG, axisA: [0, 1, 0], axisB: [0, 1, 0] } },
    // 颈（锥扭 40°，twist ±60°）
    { type: 'ball', a: 'chest', b: 'head', limit: { coneMax: 40 * DEG, twistMin: -60 * DEG, twistMax: 60 * DEG, axisA: [0, 1, 0], axisB: [0, 1, 0] } },
    // 肩 ×2（锥扭 90°，twist ±45°）
    { type: 'ball', a: 'chest', b: 'upperArmL', limit: { coneMax: 90 * DEG, twistMin: -45 * DEG, twistMax: 45 * DEG, axisA: [-1, 0, 0], axisB: [-1, 0, 0] } },
    { type: 'ball', a: 'chest', b: 'upperArmR', limit: { coneMax: 90 * DEG, twistMin: -45 * DEG, twistMax: 45 * DEG, axisA: [1, 0, 0], axisB: [1, 0, 0] } },
    // 腕 ×2（锥扭 30°，twist ±15°；§7.2 未列，补充默认值）
    { type: 'ball', a: 'forearmL', b: 'handL', limit: { coneMax: 30 * DEG, twistMin: -15 * DEG, twistMax: 15 * DEG, axisA: [-1, 0, 0], axisB: [-1, 0, 0] } },
    { type: 'ball', a: 'forearmR', b: 'handR', limit: { coneMax: 30 * DEG, twistMin: -15 * DEG, twistMax: 15 * DEG, axisA: [1, 0, 0], axisB: [1, 0, 0] } },
    // 髋 ×2（锥扭 60°，twist ±30°）
    { type: 'ball', a: 'pelvis', b: 'thighL', limit: { coneMax: 60 * DEG, twistMin: -30 * DEG, twistMax: 30 * DEG, axisA: [0, -1, 0], axisB: [0, -1, 0] } },
    { type: 'ball', a: 'pelvis', b: 'thighR', limit: { coneMax: 60 * DEG, twistMin: -30 * DEG, twistMax: 30 * DEG, axisA: [0, -1, 0], axisB: [0, -1, 0] } },
    // 肘 ×2（铰链 [0,150°]，弯曲方向朝前 +z）
    { type: 'hinge', a: 'upperArmL', b: 'forearmL', limit: { aMin: 0, aMax: 150 * DEG, hingeAxis: [0, 1, 0], refAxis: [-1, 0, 0] } },
    { type: 'hinge', a: 'upperArmR', b: 'forearmR', limit: { aMin: 0, aMax: 150 * DEG, hingeAxis: [0, -1, 0], refAxis: [1, 0, 0] } },
    // 膝 ×2（铰链 [0,150°]，弯曲方向朝后 −z）
    { type: 'hinge', a: 'thighL', b: 'shinL', limit: { aMin: 0, aMax: 150 * DEG, hingeAxis: [1, 0, 0], refAxis: [0, -1, 0] } },
    { type: 'hinge', a: 'thighR', b: 'shinR', limit: { aMin: 0, aMax: 150 * DEG, hingeAxis: [1, 0, 0], refAxis: [0, -1, 0] } },
  ];

  return {
    bodies,
    joints,
    // 过滤表微调：左右手互斥（T-pose 落地时双手相邻易抖）
    ignorePairs: [['handL', 'handR']],
    forcePairs: [],
    totalMass,
  };
}

const DEG = Math.PI / 180;

// ---------------------------------------------------------------------------
// SkeletonDef → 求解器实例（core 层构建器，runtime 复用）
// ---------------------------------------------------------------------------

/** 实例化结果：solver + 名称映射（供 runtime 回写与诊断） */
export interface MRRagdollInstance {
  solver: MRSolver;
  def: SkeletonDef;
  bodies: Map<string, MRBody>;
  capsules: Map<string, MRCapsule>;
  ballJoints: MRBallJoint[];
  hinges: MRHingeJoint[];
  limits: MRConeTwistLimit[];
}

/**
 * 实例化 SkeletonDef。
 * bindOverrides：可选的逐体 bind 位姿覆盖（世界系 COM 位置 + 旋转）；
 * 缺省时用 BodyDef.bindPos 与 identity 旋转（默认人形即 T-pose）。
 */
export function instantiateSkeleton(
  def: SkeletonDef,
  bindOverrides?: Map<string, { pos: [number, number, number]; quat?: [number, number, number, number] }>,
): MRRagdollInstance {
  // 质量归一化（§7.1 表总和不足 100%，按比例归一）
  let massSum = 0;
  for (const b of def.bodies) massSum += b.massRatio;
  const scale = def.totalMass / massSum;

  const bodyMap = new Map<string, MRBody>();
  const capsules = new Map<string, MRCapsule>();
  const defByName = new Map<string, BodyDef>();
  const solver = new MRSolver();

  for (const bd of def.bodies) {
    defByName.set(bd.name, bd);
    const rb = new MRBody();
    rb.name = bd.name;
    const mass = bd.massRatio * scale;
    rb.invMass = 1 / mass;

    // 盒惯性：capsule 包围盒半尺寸（各轴 = 线段半长 + r）
    const { p1Local, p2Local, radius } = bd.capsule;
    const hx = (Math.abs(p1Local[0]) + Math.abs(p2Local[0])) / 2 + radius;
    const hy = (Math.abs(p1Local[1]) + Math.abs(p2Local[1])) / 2 + radius;
    const hz = (Math.abs(p1Local[2]) + Math.abs(p2Local[2])) / 2 + radius;
    boxInvInertia(hx, hy, hz, mass, rb.invIlocal);

    // bind 位姿
    const override = bindOverrides?.get(bd.name);
    const bp = override?.pos ?? bd.bindPos ?? [0, 0, 0];
    rb.x.set(bp[0], bp[1], bp[2]);
    if (override?.quat) rb.q.set(override.quat[0], override.quat[1], override.quat[2], override.quat[3]);
    else rb.q.identity();
    rb.refreshDerived();
    // 子步快照初始化（首个子步的 prevR/prevX 有效）
    rb.prevX.copy(rb.x);
    rb.prevQ.copy(rb.q);
    rb.prevR.copy(rb.rot);

    const cap = new MRCapsule(rb);
    cap.name = bd.name;
    cap.p1Local.set(p1Local[0], p1Local[1], p1Local[2]);
    cap.p2Local.set(p2Local[0], p2Local[1], p2Local[2]);
    cap.radius = radius;
    cap.groundFriction = bd.groundFriction ?? 0.7;
    cap.selfFriction = bd.selfFriction ?? 0.3;

    bodyMap.set(bd.name, rb);
    capsules.set(bd.name, cap);
    solver.bodies.push(rb);
    solver.capsules.push(cap);
  }

  // 关节与限位
  const ballJoints: MRBallJoint[] = [];
  const hinges: MRHingeJoint[] = [];
  const limits: MRConeTwistLimit[] = [];

  for (const jd of def.joints) {
    const A = bodyMap.get(jd.a);
    const B = bodyMap.get(jd.b);
    const capB = capsules.get(jd.b);
    if (!A || !B || !capB) continue;

    // 锚点世界位置 = 子体 bind COM + R_B·p1Local（近端关节位置）
    const anchorWorld = anchorTmp.set(capB.p1Local.x, capB.p1Local.y, capB.p1Local.z);
    applyQuat(B.q, anchorWorld);
    anchorWorld.add(B.x);

    // rALocal = R_A⁻¹·(anchor − A.x)；rBLocal = capB.p1Local
    const rALocal = new MRVec3().copy(anchorWorld).sub(A.x);
    invApplyQuat(A.q, rALocal);
    const rBLocal = new MRVec3().copy(capB.p1Local);

    const lim = jd.limit;
    if (jd.type === 'ball') {
      const joint = new MRBallJoint(A, B);
      joint.rALocal.copy(rALocal);
      joint.rBLocal.copy(rBLocal);
      joint.name = `${jd.b} joint`;
      ballJoints.push(joint);
      solver.ballJoints.push(joint);

      if (lim) {
        const limit = new MRConeTwistLimit(A, B);
        limit.rALocal.copy(rALocal);
        limit.rBLocal.copy(rBLocal);
        const la = lim.axisA ?? [0, 1, 0];
        const lb = lim.axisB ?? la;
        limit.axisALocal.set(la[0], la[1], la[2]);
        limit.axisBLocal.set(lb[0], lb[1], lb[2]);
        // 扭转测量垂直参考轴：bind 时与主轴世界对齐，取主轴的正交轴
        orthoAxisOfLocal(limit.axisALocal, limit.twistRefALocal);
        orthoAxisOfLocal(limit.axisBLocal, limit.twistRefBLocal);
        limit.q0Bind.copy(A.q).conjugateInPlace().multiply(limit.q0Bind, B.q); // conj(qA)⊗qB
        limit.coneMax = lim.coneMax ?? 0;
        limit.twistMin = lim.twistMin ?? 0;
        limit.twistMax = lim.twistMax ?? 0;
        // [v1.4] 被动扭转黏性阻尼：缺省引擎默认（关节囊黏性项），显式 0 关闭
        limit.twistDamp = lim.twistDamp ?? DEFAULT_TWIST_DAMP;
        // [v1.5] 被动扭转刚度（回中）：缺省引擎默认（关节囊弹性项），显式 0 关闭
        limit.twistStiffness = lim.twistStiffness ?? DEFAULT_TWIST_STIFFNESS;
        limit.name = `${jd.b} limit`;
        limits.push(limit);
        solver.limits.push(limit);
      }
    } else {
      const joint = new MRHingeJoint(A, B);
      joint.rALocal.copy(rALocal);
      joint.rBLocal.copy(rBLocal);
      const ha = lim?.hingeAxis ?? [1, 0, 0];
      const ra = lim?.refAxis ?? [0, 1, 0];
      const rb = lim?.refAxisB ?? ra;
      // 限位参考系随两体各自的 bind 旋转（bind 时 A/B 侧世界对齐）
      joint.hingeAxisALocal.set(ha[0], ha[1], ha[2]);
      joint.hingeAxisBLocal.set(ha[0], ha[1], ha[2]);
      joint.refAxisALocal.set(ra[0], ra[1], ra[2]);
      joint.refAxisBLocal.set(rb[0], rb[1], rb[2]);
      joint.aMin = lim?.aMin ?? 0;
      joint.aMax = lim?.aMax ?? Math.PI;
      joint.name = `${jd.b} hinge`;
      hinges.push(joint);
      solver.hinges.push(joint);
    }
  }

  // 配对过滤表
  solver.collisionPairs = buildCollisionPairs(def);

  return { solver, def, bodies: bodyMap, capsules, ballJoints, hinges, limits };
}

// ---------------------------------------------------------------------------
// 四元数旋转向量（v' = q·v·conj(q)，Rodrigues 等价实现）
// ---------------------------------------------------------------------------

function applyQuat(q: Readonly<MRQuat>, v: MRVec3): void {
  const { x, y, z, w } = q;
  const tx = 2 * (y * v.z - z * v.y);
  const ty = 2 * (z * v.x - x * v.z);
  const tz = 2 * (x * v.y - y * v.x);
  v.x += w * tx + (y * tz - z * ty);
  v.y += w * ty + (z * tx - x * tz);
  v.z += w * tz + (x * ty - y * tx);
}

function invApplyQuat(q: Readonly<MRQuat>, v: MRVec3): void {
  invApplyTmp.set(-q.x, -q.y, -q.z, q.w);
  applyQuat(invApplyTmp, v);
}
const invApplyTmp = new MRQuat();
const anchorTmp = new MRVec3();

/** 与单位轴 axis 正交的单位向量（局部系，确定性选择） */
function orthoAxisOfLocal(axis: Readonly<MRVec3>, out: MRVec3): void {
  const ax = Math.abs(axis.x), ay = Math.abs(axis.y), az = Math.abs(axis.z);
  if (ax <= ay && ax <= az) out.set(0, -axis.z, axis.y);
  else if (ay <= az) out.set(-axis.z, 0, axis.x);
  else out.set(-axis.y, axis.x, 0);
  out.normalize();
}
