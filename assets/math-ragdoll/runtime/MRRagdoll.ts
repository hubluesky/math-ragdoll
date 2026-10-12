/**
 * MRRagdoll —— 布娃娃主组件（runtime 层，依赖 'cc'）
 * 见设计文档 §2.1 / §6：构建、固定步长累加器、步进、骨骼回写。
 *
 * 使用：
 * 1. 挂在模型根节点上，把 13 根骨骼节点按 BONE_ORDER 拖入 boneNodes。
 * 2. 模型处于 bind 姿态（T-pose/待机）时点菜单或运行时 onLoad 自动 build()。
 * 3. 被动模式必须停掉模型上的 AnimationController（§6.3，组件已自动处理）。
 *
 * 时间步：Cocos 无 FixedUpdate，用累加器自实现固定步长（§6.2）；
 * 骨骼回写在 lateUpdate（渲染前最后执行，覆盖动画写入）。
 */
import { _decorator, Component, Node, CCInteger, CCFloat } from 'cc';
import { instantiateSkeleton, MRRagdollInstance } from '../core/MRDefaultSkeleton';
import { MRBody } from '../core/MRBody';
import { MRVec3, MRQuat } from '../core/MRMath';
import { MRGroundPlane } from '../core/MRGround';
import { MRGround } from './MRGround';
import { BONE_ORDER, buildSkeletonDef } from './MRRagdollBuilder';

const { ccclass, property, executionOrder } = _decorator;

/** 骨骼回写绑定：bone 在 body 局部系的 bind 偏移 */
interface BoneBinding {
  body: MRBody;
  bone: Node;
  localPos: MRVec3;
  localRot: MRQuat;
}

@ccclass('MRRagdoll')
@executionOrder(100) // 晚于动画/控制组件；回写在 lateUpdate，天然覆盖动画写入（§6.3）
export class MRRagdoll extends Component {
  @property({ type: [Node], tooltip: '13 根骨骼节点，顺序：pelvis,chest,head,upperArmL,forearmL,handL,thighL,shinL,upperArmR,forearmR,handR,thighR,shinR' })
  boneNodes: Node[] = [];

  @property({ type: MRGround, tooltip: '地面（可选，缺省为 y=0 无限平面）' })
  ground: MRGround | null = null;

  @property({ type: CCFloat, tooltip: '总质量 kg' })
  totalMass = 70;

  @property({ type: CCFloat, tooltip: 'capsule 半径缩放（1.75m 体型 = 1）' })
  scaleAdjust = 1;

  @property({ type: CCFloat, tooltip: '逻辑步长（秒）' })
  fixedDt = 1 / 60;

  @property({ type: CCInteger, tooltip: '每逻辑步的子步数（抖动则加大，勿放大 h）' })
  substeps = 8;

  @property
  /** 是否在 onLoad 自动构建（需骨骼处于 bind 姿态） */
  autoBuild = true;

  @property({ tooltip: '运行时自检日志：每 0.5s 输出体高/速度/NaN/锚点误差（联调验证用）' })
  debugLog = false;

  /** core 实例（构建后有效） */
  instance: MRRagdollInstance | null = null;

  /** 骨骼回写绑定 */
  private bindings: BoneBinding[] = [];
  /** 固定步长累加器 */
  private acc = 0;
  /** 是否已把骨骼 bind 相对位姿存好 */
  private built = false;
  /** 运行时间（自检日志计时用） */
  private elapsed = 0;
  /** 自检日志间隔计时 */
  private debugTimer = 0;

  onLoad(): void {
    if (this.autoBuild && !this.built) this.build();
  }

  onEnable(): void {
    // 被动模式：停掉动画，避免每帧覆盖物理回写（§6.3 / §9.6）
    // 兼容 Animation（关键帧动画）与 AnimationController（动画图）
    const anim = this.getComponent('Animation') || this.getComponent('AnimationController');
    (anim as { stop?: () => void } | null)?.stop?.();
  }

  /** 构建（必须在 bind 姿态下调用） */
  build(): void {
    if (this.boneNodes.length !== BONE_ORDER.length) {
      console.error(`[MRRagdoll] boneNodes 需要 ${BONE_ORDER.length} 个节点，实际 ${this.boneNodes.length}`);
      return;
    }
    // Cocos 场景里的世界矩阵确保最新
    for (const n of this.boneNodes) n.updateWorldTransform();

    const def = buildSkeletonDef(this.boneNodes, { scaleAdjust: this.scaleAdjust, totalMass: this.totalMass });
    this.instance = instantiateSkeleton(def);
    const solver = this.instance.solver;
    solver.substeps = this.substeps;
    // 未挂地面组件时缺省 y=0 无限平面
    solver.ground = this.ground !== null ? this.ground : new MRGroundPlane(0);

    // 骨骼回写绑定：bone 相对 body 的局部偏移（bind 时刻）
    this.bindings = [];
    BONE_ORDER.forEach((name, i) => {
      const body = this.instance!.bodies.get(name);
      if (!body) return;
      const bone = this.boneNodes[i];
      // boneLocal = R_body⁻¹ · (boneWorld − bodyCOM)
      const bw = bone.worldPosition;
      const rel = mrTmpV.set(bw.x - body.x.x, bw.y - body.x.y, bw.z - body.x.z);
      invQuatApply(body.q, rel);
      // localRot = conj(q_body) ⊗ q_bone
      const bq = bone.worldRotation;
      const q = body.q;
      const localRot = new MRQuat(
        q.w * bq.x - q.x * bq.w - q.y * bq.z + q.z * bq.y,
        q.w * bq.y + q.y * bq.w + q.z * bq.x - q.x * bq.z,
        q.w * bq.z + q.z * bq.w + q.x * bq.y - q.y * bq.x,
        q.w * bq.w - q.x * bq.x - q.y * bq.y - q.z * bq.z,
      );
      this.bindings.push({ body, bone, localPos: rel.clone(), localRot });
    });

    this.built = true;
    this.acc = 0;
    this.elapsed = 0;
    this.debugTimer = 0;
    if (this.debugLog) {
      const s = this.instance.solver;
      console.log(`[MRRagdoll] built: ${s.bodies.length} bodies, ${s.ballJoints.length + s.hinges.length} joints, ${s.limits.length} limits, ${s.collisionPairs.length} collision pairs, substeps=${s.substeps}`);
    }
  }

  /** 受击变软（委托 MRActiveController；未挂控制器时直接置空电机强度） */
  applyHit(strengthDrop: number, recoverSeconds: number): void {
    const ctrl = this.getComponent('MRActiveController') as unknown as { applyHit: (a: number, b: number) => void } | null;
    if (ctrl) ctrl.applyHit(strengthDrop, recoverSeconds);
  }

  update(dt: number): void {
    if (!this.instance) return;
    // 钳制切后台/卡顿的 dt 尖峰，防螺旋死亡（§6.2 / §9.12）
    this.acc += Math.min(dt, 0.1);
    let steps = 0;
    while (this.acc >= this.fixedDt && steps < 8) {
      this.instance.solver.step(this.fixedDt);
      this.acc -= this.fixedDt;
      steps++;
      this.elapsed += this.fixedDt;
      this.debugTimer += this.fixedDt;
    }
    if (this.acc >= this.fixedDt) this.acc = 0; // 丢弃积压（极端卡顿）
    if (this.debugLog && this.debugTimer >= 0.5) {
      this.logDiagnostics();
      this.debugTimer = 0;
    }
  }

  /** 自检诊断：min 体高 / max 速度 / NaN / 关节锚点误差（验收指标的运行时等价） */
  private logDiagnostics(): void {
    const s = this.instance!.solver;
    let minY = Number.POSITIVE_INFINITY;
    let maxSpeed = 0;
    let hasNaN = false;
    for (const b of s.bodies) {
      if (b.x.y < minY) minY = b.x.y;
      const sp = b.v.length();
      if (sp > maxSpeed) maxSpeed = sp;
      if (!b.x.isFinite() || !b.q.isFinite() || !b.v.isFinite() || !b.w.isFinite()) hasNaN = true;
    }
    let maxAnchorErr = 0;
    for (const j of s.ballJoints) {
      const pa = j.A.rot.applyToVec3(j.rALocal, new MRVec3()).add(j.A.x);
      const pb = j.B.rot.applyToVec3(j.rBLocal, new MRVec3()).add(j.B.x);
      const err = pa.distanceTo(pb);
      if (err > maxAnchorErr) maxAnchorErr = err;
    }
    console.log(
      `[MRRagdoll t=${this.elapsed.toFixed(1)}s] minY=${minY.toFixed(3)} maxSpeed=${maxSpeed.toFixed(3)} ` +
      `anchorErr=${(maxAnchorErr * 1000).toFixed(2)}mm contacts=${s.lastContactCount} NaN=${hasNaN}`,
    );
  }

  lateUpdate(): void {
    if (!this.instance) return;
    // 骨骼回写：按 def.bodies 顺序（父先子后）写世界位姿（§6.3）。
    // 顺序关键：worldPosition/Rotation 的 setter 依赖当前父节点变换，必须父→子。
    const order = this.instance.def.bodies;
    const byName = new Map(this.bindings.map((b) => [b.body.name, b]));
    for (const bd of order) {
      const binding = byName.get(bd.name);
      if (!binding) continue;
      const body = binding.body;
      // world = body.x ⊕ R·localPos；worldRot = q ⊗ localRot
      const wp = body.rot.applyToVec3(binding.localPos, mrTmpV).add(body.x);
      const wq = mrTmpQ.multiply(body.q, binding.localRot);
      binding.bone.worldRotation = { x: wq.x, y: wq.y, z: wq.z, w: wq.w } as never;
      binding.bone.worldPosition = { x: wp.x, y: wp.y, z: wp.z } as never;
    }
  }
}

// runtime 内部临时量
const mrTmpV = new MRVec3();
const mrTmpQ = new MRQuat();

/** v ← conj(q)·v·q（用 Rodrigues 等价实现，q 需为单位四元数） */
function invQuatApply(q: MRQuat, v: MRVec3): void {
  const x = -q.x, y = -q.y, z = -q.z, w = q.w;
  const tx = 2 * (y * v.z - z * v.y);
  const ty = 2 * (z * v.x - x * v.z);
  const tz = 2 * (x * v.y - y * v.x);
  const nx = v.x + w * tx + (y * tz - z * ty);
  const ny = v.y + w * ty + (z * tx - x * tz);
  const nz = v.z + w * tz + (x * ty - y * tx);
  v.set(nx, ny, nz);
}
