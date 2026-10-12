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
import { _decorator, Component, Node, CCInteger, CCFloat, input, Input, EventTouch, EventMouse, EventKeyboard, KeyCode, Camera, director, geometry } from 'cc';
import { instantiateSkeleton, MRRagdollInstance } from '../core/MRDefaultSkeleton';
import { MRBody } from '../core/MRBody';
import { MRVec3, MRQuat } from '../core/MRMath';
import { closestPtSegmentSegment } from '../core/MRCollision';
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

  @property({ type: CCFloat, tooltip: '点击命中时施加的冲量（N·s，M4 验收基准 200）；0 = 禁用点击击飞' })
  clickImpulse = 200;

  @property({ type: CCFloat, tooltip: '命中体速度上限（m/s）：冲量按命中体质量限幅，点手臂=甩动带动身体，点躯干=整体击飞' })
  maxHitDv = 8;

  @property({ type: CCFloat, tooltip: '击飞上抬分量：冲量方向 = 视线方向 + up×该系数（归一化）' })
  hitUpBias = 0.35;

  /** core 实例（构建后有效） */
  instance: MRRagdollInstance | null = null;

  /**
   * 骨骼回写绑定（build 后只读）。
   * localRot = conj(q_body_bind) ⊗ q_bone_bind（bone 世界旋转 = q_body ⊗ localRot），
   * 电机目标换算到求解器参考系需要它（MRActiveController.captureTargets）。
   */
  bindings: BoneBinding[] = [];
  /** 固定步长累加器 */
  private acc = 0;
  /** 是否已把骨骼 bind 相对位姿存好 */
  private built = false;
  /** build 时刻各骨骼节点的世界位姿快照（reset 恢复用） */
  private bindPoses: { pos: import('cc').Vec3; rot: import('cc').Quat }[] = [];
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
    // 演示交互：点击击飞 / R 复位（M4 验收场景；纯被动布娃娃，无电机参与）。
    // 坐标换算交给引擎：getLocation() 已含画布偏移/缩放/左下原点翻转，
    // 与 screenPointToRay 同一约定 —— 不要手动从 canvas rect 推算（DPR 假设脆弱）。
    input.on(Input.EventType.MOUSE_DOWN, this.onPointerDown, this);
    input.on(Input.EventType.TOUCH_START, this.onPointerDown, this);
    input.on(Input.EventType.KEY_DOWN, this.onKeyDown, this);
  }

  onDisable(): void {
    input.off(Input.EventType.MOUSE_DOWN, this.onPointerDown, this);
    input.off(Input.EventType.TOUCH_START, this.onPointerDown, this);
    input.off(Input.EventType.KEY_DOWN, this.onKeyDown, this);
  }

  /** 构建（必须在 bind 姿态下调用） */
  build(): void {
    if (this.boneNodes.length !== BONE_ORDER.length) {
      console.error(`[MRRagdoll] boneNodes 需要 ${BONE_ORDER.length} 个节点，实际 ${this.boneNodes.length}`);
      return;
    }
    // Cocos 场景里的世界矩阵确保最新
    for (const n of this.boneNodes) n.updateWorldTransform();
    // 快照当前世界位姿（reset 回到此姿态重新构建）
    this.bindPoses = this.boneNodes.map((n) => ({ pos: n.worldPosition.clone(), rot: n.worldRotation.clone() }));

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

  /**
   * 复位：把骨骼恢复到 build 时刻的 bind 世界位姿并重建实例（新 solver、零速度）。
   * 击倒后重新演示用。MRActiveController 检测到实例更换后自动重建电机并复位强度。
   */
  reset(): void {
    if (this.bindPoses.length !== this.boneNodes.length) return;
    for (let i = 0; i < this.boneNodes.length; i++) {
      const n = this.boneNodes[i];
      n.worldRotation = this.bindPoses[i].rot;
      n.worldPosition = this.bindPoses[i].pos;
    }
    this.instance = null;
    this.bindings = [];
    this.built = false;
    this.acc = 0;
    this.build();
  }

  // ---- 演示交互：点击击飞 / R 复位（M4 验收场景，纯被动无电机）----

  /** 点击（鼠标/触摸）：沿摄像机射线解析拾取 capsule，命中刚体在命中点施加冲量（按质量限幅） */
  private onPointerDown(e: EventTouch | EventMouse): void {
    const inst = this.instance;
    if (!inst || this.clickImpulse <= 0) return;
    const scene = director.getScene();
    const cam = scene ? scene.getComponentInChildren(Camera) : null;
    if (!cam) return;
    const loc = e.getLocation();
    cam.screenPointToRay(loc.x, loc.y, pointerRay);

    const ox = pointerRay.o.x, oy = pointerRay.o.y, oz = pointerRay.o.z;
    const dx = pointerRay.d.x, dy = pointerRay.d.y, dz = pointerRay.d.z;
    rayP1.set(ox, oy, oz);
    rayP2.set(ox + dx * PICK_DIST, oy + dy * PICK_DIST, oz + dz * PICK_DIST);
    rayDir.set(dx, dy, dz);

    // 逐 capsule 求射线-线段最近距离，取沿射线最近（最先命中）者
    let bestT = Number.POSITIVE_INFINITY;
    let bestBody: MRBody | null = null;
    for (const cap of inst.solver.capsules) {
      cap.worldP1(capP1);
      cap.worldP2(capP2);
      const distSqr = closestPtSegmentSegment(rayP1, rayP2, capP1, capP2, pickC1, pickC2);
      const rSum = cap.radius + PICK_PAD;
      if (distSqr >= rSum * rSum) continue;
      const t = pickC1.sub(rayP1).dot(rayDir);
      if (t < bestT) {
        bestT = t;
        bestBody = cap.body;
        bestHit.set(rayP1.x + rayDir.x * t, rayP1.y + rayDir.y * t, rayP1.z + rayDir.z * t);
      }
    }
    if (!bestBody) return;

    // 冲量：视线方向 + 上抬分量（击飞），作用于命中点（含翻转角冲量）。
    // 按命中体质量限幅：同一拍打力打在轻肢体上 Δv 会到几十 m/s（实测手臂 91），
    // 观感假；限幅后点手臂=甩动带动身体，点躯干=整体击飞。
    jDir.copy(rayDir).addScaled(UP_VEC, this.hitUpBias).normalize();
    const jMag = Math.min(this.clickImpulse, this.maxHitDv / bestBody.invMass);
    jImp.copy(jDir).scale(jMag);
    bestBody.v.addScaled(jImp, bestBody.invMass);
    rHit.set(bestHit.x - bestBody.x.x, bestHit.y - bestBody.x.y, bestHit.z - bestBody.x.z);
    rxJ.cross(rHit, jImp);
    const I = bestBody.invIworld.m;
    bestBody.w.x += I[0] * rxJ.x + I[1] * rxJ.y + I[2] * rxJ.z;
    bestBody.w.y += I[3] * rxJ.x + I[4] * rxJ.y + I[5] * rxJ.z;
    bestBody.w.z += I[6] * rxJ.x + I[7] * rxJ.y + I[8] * rxJ.z;
    if (this.debugLog) console.log(`[MRRagdoll] hit ${bestBody.name}, J=${jMag.toFixed(1)} N·s`);
  }

  /** R 键：复位到 bind 位姿重新掉落 */
  private onKeyDown(e: EventKeyboard): void {
    if (e.keyCode === KeyCode.KEY_R) this.reset();
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

// 点击拾取/冲量模块级常量与临时量
const PICK_DIST = 50; // 射线拾取线段长度（米）
const PICK_PAD = 0.08; // 拾取放宽半径（米）：capsule 较细，纯表面拾取难点中
const UP_VEC = new MRVec3(0, 1, 0);
const pointerRay = new geometry.Ray();
const rayP1 = new MRVec3();
const rayP2 = new MRVec3();
const rayDir = new MRVec3();
const capP1 = new MRVec3();
const capP2 = new MRVec3();
const pickC1 = new MRVec3();
const pickC2 = new MRVec3();
const bestHit = new MRVec3();
const jDir = new MRVec3();
const jImp = new MRVec3();
const rHit = new MRVec3();
const rxJ = new MRVec3();

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
