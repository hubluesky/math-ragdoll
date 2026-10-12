/**
 * MRRagdollBuilder —— 从骨骼节点表生成 SkeletonDef（runtime 层，依赖 'cc'）
 * 见设计文档 §6.1。
 *
 * 用法：在模型的 bind 姿态（T-pose/待机）下，把 13 根骨骼节点按固定顺序
 * 传入（见 BONE_ORDER），构建时读取节点的当前世界位姿生成 capsule、关节、
 * 限位与过滤表，产出纯数据 SkeletonDef，由 core/instantiateSkeleton 实例化。
 *
 * 模型尺度：所有参数按"1 单位 = 1 米"标定（§9.11）；体型不是 1.75m 时用
 * scaleAdjust 整体缩放 capsule 半径（不要逐项调参补偿）。
 */
import { Node, Vec3, Quat } from 'cc';
import { MRVec3 } from '../core/MRMath';
import { BodyDef, JointDef, SkeletonDef } from '../core/MRSkeletonDef';

/** 13 体固定顺序（与 MRRagdoll 组件的骨骼节点数组一一对应） */
export const BONE_ORDER = [
  'pelvis', 'chest', 'head',
  'upperArmL', 'forearmL', 'handL', 'thighL', 'shinL',
  'upperArmR', 'forearmR', 'handR', 'thighR', 'shinR',
] as const;

/** 质量比例（§7.1；构建时归一化） */
const MASS_RATIO: Record<string, number> = {
  pelvis: 15, chest: 30, head: 8,
  upperArmL: 3.0, forearmL: 2.0, handL: 0.8,
  thighL: 10.0, shinL: 5.7,
  upperArmR: 3.0, forearmR: 2.0, handR: 0.8,
  thighR: 10.0, shinR: 5.7,
};

/** 末端体（无子骨骼）的 capsule 延长长度（米）：head/hand/shin 等末端段 */
const END_EXTEND_LEN = 0.22;

/** 关节限位默认表（§7.2，弧度） */
const DEG = Math.PI / 180;
const LIMIT_TABLE: Record<string, JointDef['limit']> = {
  chest: { coneMax: 30 * DEG, twistMin: -30 * DEG, twistMax: 30 * DEG },
  head: { coneMax: 40 * DEG, twistMin: -60 * DEG, twistMax: 60 * DEG },
  upperArmL: { coneMax: 90 * DEG, twistMin: -45 * DEG, twistMax: 45 * DEG },
  upperArmR: { coneMax: 90 * DEG, twistMin: -45 * DEG, twistMax: 45 * DEG },
  forearmL: { aMin: 0, aMax: 150 * DEG }, // 肘：铰链，弯曲朝前
  forearmR: { aMin: 0, aMax: 150 * DEG },
  handL: { coneMax: 30 * DEG, twistMin: -15 * DEG, twistMax: 15 * DEG },
  handR: { coneMax: 30 * DEG, twistMin: -15 * DEG, twistMax: 15 * DEG },
  thighL: { coneMax: 60 * DEG, twistMin: -30 * DEG, twistMax: 30 * DEG },
  thighR: { coneMax: 60 * DEG, twistMin: -30 * DEG, twistMax: 30 * DEG },
  shinL: { aMin: 0, aMax: 150 * DEG }, // 膝：铰链，弯曲朝后
  shinR: { aMin: 0, aMax: 150 * DEG },
};

/** 父体映射（与 core 默认人形一致） */
const PARENT_MAP: Record<string, string | null> = {
  pelvis: null, chest: 'pelvis', head: 'chest',
  upperArmL: 'chest', forearmL: 'upperArmL', handL: 'forearmL',
  thighL: 'pelvis', shinL: 'thighL',
  upperArmR: 'chest', forearmR: 'upperArmR', handR: 'forearmR',
  thighR: 'pelvis', shinR: 'thighR',
};

/** 铰链关节（子体名 → 弯曲方向，世界系；模型面朝 +z 时的默认） */
const HINGE_BEND_DIR: Record<string, [number, number, number]> = {
  forearmL: [0, 0, 1], forearmR: [0, 0, 1], // 肘弯朝前
  shinL: [0, 0, -1], shinR: [0, 0, -1], // 膝弯朝后
};

const tmpV1 = new Vec3();
const tmpV3 = new Vec3();
const tmpQ1 = new Quat();
const tmpV2 = new Vec3();
const mrTmp1 = new MRVec3();
const mrTmp2 = new MRVec3();

export interface BuildOptions {
  /** capsule 半径缩放（1.75m 体型基准 = 1） */
  scaleAdjust?: number;
  totalMass?: number;
}

/**
 * 由骨骼节点表构建 SkeletonDef。
 * @param boneNodes 13 个骨骼节点（顺序见 BONE_ORDER），必须在 bind 姿态下调用
 */
export function buildSkeletonDef(boneNodes: Node[], opts: BuildOptions = {}): SkeletonDef {
  if (boneNodes.length !== BONE_ORDER.length) {
    throw new Error(`boneNodes 数量应为 ${BONE_ORDER.length}，实际 ${boneNodes.length}`);
  }
  const scaleAdjust = opts.scaleAdjust ?? 1;
  const totalMass = opts.totalMass ?? 70;

  const nameToNode = new Map<string, Node>();
  BONE_ORDER.forEach((name, i) => nameToNode.set(name, boneNodes[i]));

  const nameToJointPos = new Map<string, Vec3>(); // 体"近端关节"世界位置 = 骨骼节点位置
  const nameToEndPos = new Map<string, Vec3>(); // 体"远端"世界位置
  for (const name of BONE_ORDER) {
    const node = nameToNode.get(name)!;
    const near = node.worldPosition.clone();
    // 远端：第一个子骨骼（关节图中的子体）的节点位置；末端体沿"父→本骨骼"方向延长
    let end: Vec3 | null = null;
    for (const childName in PARENT_MAP) {
      if (PARENT_MAP[childName] === name) {
        end = nameToNode.get(childName)!.worldPosition.clone();
        break;
      }
    }
    if (end === null) {
      // 末端体（无子骨骼）：沿骨骼自然延伸方向（父→本骨骼）延长固定长度。
      // 勘误：固定 -Y 会让头/手（骨骼方向非 -Y）的物理 capsule 与视觉错位，
      // 落地时无碰撞支撑引发持续抖动（编辑器联调实测）。
      const dir = tmpV1.set(0, -1, 0);
      const parentName = PARENT_MAP[name];
      if (parentName) {
        const pp = nameToNode.get(parentName)!.worldPosition;
        dir.set(near.x - pp.x, near.y - pp.y, near.z - pp.z);
        const dirLen = dir.length();
        if (dirLen > 1e-6) dir.multiplyScalar(1 / dirLen);
        else dir.set(0, -1, 0);
      }
      end = near.clone();
      Vec3.scaleAndAdd(end, near, dir, END_EXTEND_LEN);
    }
    nameToJointPos.set(name, near);
    nameToEndPos.set(name, end);
  }

  // ---- BodyDef ----
  const bodies: BodyDef[] = [];
  for (const name of BONE_ORDER) {
    const node = nameToNode.get(name)!;
    const near = nameToJointPos.get(name)!;
    const end = nameToEndPos.get(name)!;
    const halfLen = Vec3.distance(near, end) / 2;

    // COM = 线段中点；体旋转 = 骨骼节点世界旋转
    const com = near.clone().add(end).multiplyScalar(0.5);
    const qNode = node.worldRotation.clone();

    const radius = Math.min(0.12, Math.max(0.03, 0.45 * halfLen)) * scaleAdjust;

    // capsule 局部端点：R⁻¹·(joint − COM)
    const p1Local = toMrLocal(com, qNode, near, mrTmp1);
    const p2Local = toMrLocal(com, qNode, end, mrTmp2);

    bodies.push({
      name,
      parent: PARENT_MAP[name],
      massRatio: MASS_RATIO[name],
      capsule: { p1Local: [p1Local.x, p1Local.y, p1Local.z], p2Local: [p2Local.x, p2Local.y, p2Local.z], radius },
      bindPos: [com.x, com.y, com.z],
    });
  }

  // ---- JointDef ----
  const joints: JointDef[] = [];
  for (const name of BONE_ORDER) {
    const parentName = PARENT_MAP[name];
    if (parentName === null) continue;
    const limit = LIMIT_TABLE[name];
    if (!limit) continue;
    if (limit.aMin !== undefined) {
      // 铰链：轴 = 远端方向 × 弯曲方向（世界系），refAxis = 远端方向
      const childNode = nameToNode.get(name)!;
      const dWorld = Vec3.subtract(new Vec3(), nameToJointPos.get(name)!, nameToJointPos.get(parentName)!).normalize();
      const bend = HINGE_BEND_DIR[name] ?? [0, 0, 1];
      const axisWorld = Vec3.cross(new Vec3(), dWorld, new Vec3(bend[0], bend[1], bend[2])).normalize();
      // 轴/refAxis 转到 A（父体）局部系
      const qA = nameToNode.get(parentName)!.worldRotation;
      const axisLocal = worldToLocalDir(axisWorld, qA);
      const refLocal = worldToLocalDir(dWorld, qA);
      // B 侧零位参考 = 同一世界方向转到 B 局部系
      const qB = childNode.worldRotation;
      const refBLocal = worldToLocalDir(dWorld, qB);
      joints.push({
        type: 'hinge', a: parentName, b: name,
        limit: {
          aMin: limit.aMin, aMax: limit.aMax,
          hingeAxis: [axisLocal.x, axisLocal.y, axisLocal.z],
          refAxis: [refLocal.x, refLocal.y, refLocal.z],
          refAxisB: [refBLocal.x, refBLocal.y, refBLocal.z],
        },
      });
    } else {
      // 锥扭：主轴 = 远端方向（A/B 各自局部系）
      const dWorld = Vec3.subtract(new Vec3(), nameToJointPos.get(name)!, nameToJointPos.get(parentName)!).normalize();
      const qA = nameToNode.get(parentName)!.worldRotation;
      const qB = nameToNode.get(name)!.worldRotation;
      const axisALocal = worldToLocalDir(dWorld, qA);
      const axisBLocal = worldToLocalDir(dWorld, qB);
      joints.push({
        type: 'ball', a: parentName, b: name,
        limit: {
          coneMax: limit.coneMax, twistMin: limit.twistMin, twistMax: limit.twistMax,
          axisA: [axisALocal.x, axisALocal.y, axisALocal.z],
          axisB: [axisBLocal.x, axisBLocal.y, axisBLocal.z],
        },
      });
    }
  }

  // SkeletonDef 的 bindPos 即 COM bind 世界位置（core instantiateSkeleton 直接使用）
  const def: SkeletonDef = {
    bodies,
    joints,
    ignorePairs: [['handL', 'handR']],
    forcePairs: [],
    totalMass,
  };
  return def;
}

/** 世界点 → body 局部系（COM 原点、节点旋转） */
function toMrLocal(com: Vec3, q: Quat, world: Vec3, outMr: MRVec3): MRVec3 {
  tmpV2.set(world.x - com.x, world.y - com.y, world.z - com.z);
  const inv = Quat.invert(tmpQ1, q);
  Vec3.transformQuat(tmpV2, tmpV2, inv);
  outMr.set(tmpV2.x, tmpV2.y, tmpV2.z);
  return outMr;
}

/** 世界方向 → body 局部方向 */
function worldToLocalDir(d: Vec3, q: Quat): MRVec3 {
  const inv = Quat.invert(tmpQ1, q);
  Vec3.transformQuat(tmpV3, d, inv);
  return new MRVec3(tmpV3.x, tmpV3.y, tmpV3.z).normalize();
}

// rebuild trigger 1791775857
