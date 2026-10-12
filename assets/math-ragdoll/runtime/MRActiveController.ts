/**
 * MRActiveController —— 主动布娃娃控制器（L3/L4，runtime 层，依赖 'cc'）
 * 见设计文档 §5：PD 电机驱动骨骼跟随动画目标，受击变软 / 起立恢复接口。
 *
 * 架构约束（§5.1）：
 * - 求解器不知道"动画"存在。本层每帧从骨骼世界旋转采集每关节目标相对旋转
 *   qTarget_rel，转成 MRMotor 挂到 solver.motors；strength 全局标量控制软硬。
 * - 受击变软 = strength 短时间衰减到低值再按曲线恢复，不改求解器一行代码。
 *
 * 注意（§5.2）：电机的调参占主动布娃娃的大半时间，kp/kd/tauMax 需在编辑器
 * 场景实测整定；此处默认值为文档给出的整定起点。
 */
import { _decorator, Component, Node, AnimationClip } from 'cc';
import { MRMotor } from '../core/MRMotor';
import { MRQuat, MRVec3, clamp } from '../core/MRMath';
import { MRRagdoll } from './MRRagdoll';
import { BONE_ORDER } from './MRRagdollBuilder';

const { ccclass, property, requireComponent } = _decorator;

/** 每关节电机的可调参数（编辑器里逐关节覆盖默认值） */
@ccclass('MRActiveController')
@requireComponent(MRRagdoll)
export class MRActiveController extends Component {
  @property({ tooltip: '电机强度 0=纯布娃娃 1=全力跟随；受击时按曲线衰减' })
  strength = 1;

  @property({ tooltip: '电机默认 kp（躯干 200~400 / 四肢 80~200）' })
  kp = 150;

  @property({ tooltip: '电机默认 kd（临界阻尼起点 kd = 2√kp）' })
  kd = 25;

  @property({ tooltip: '默认最大力矩 N·m' })
  tauMax = 60;

  @property({ type: [Node], tooltip: '动画驱动下的骨骼节点表（与 MRRagdoll.boneNodes 同序）；缺省复用 MRRagdoll 的表' })
  animBones: Node[] = [];

  @property
  /** 是否启用电机（被动布娃娃时关闭） */
  enabledMotors = true;

  private ragdoll: MRRagdoll | null = null;
  private motors = new Map<string, MRMotor>();
  /** 原始动画下各骨骼的 bind 局部旋转（采集 qTarget_rel 的参考系） */
  private motorStrengthScale = 1;

  onLoad(): void {
    this.ragdoll = this.getComponent(MRRagdoll);
  }

  /**
   * 构建/重建电机（在 MRRagdoll.build 之后调用；通常由 MRRagdoll 构建完成后
   * 或 first update 自动触发）。
   */
  ensureMotors(): void {
    const inst = this.ragdoll?.instance;
    if (!inst || this.motors.size > 0) return;

    const parentOf = new Map<string, string | null>();
    for (const bd of inst.def.bodies) parentOf.set(bd.name, bd.parent);

    for (const bd of inst.def.bodies) {
      const parentName = bd.parent;
      if (parentName === null) continue; // 根体无关节
      const A = inst.bodies.get(parentName);
      const B = inst.bodies.get(bd.name);
      if (!A || !B) continue;
      const motor = new MRMotor(A, B);
      motor.name = `${bd.name} motor`;
      // §5.2 整定起点：踝 60~120、四肢 80~200、躯干 200~400
      if (bd.name === 'chest' || bd.name === 'head') {
        motor.kp = Math.max(this.kp, 250);
      } else if (bd.name.startsWith('shin') || bd.name.startsWith('thigh')) {
        motor.kp = this.kp;
      } else {
        motor.kp = this.kp * 0.8;
      }
      motor.kd = 2 * Math.sqrt(motor.kp);
      motor.tauMax = this.tauMax;
      motor.strength = this.strength * this.motorStrengthScale;
      this.motors.set(bd.name, motor);
      inst.solver.motors.push(motor);
    }
  }

  /**
   * 每帧物理步进前采集目标姿态：读取各骨骼节点世界旋转（动画驱动），
   * 换算成求解器关节参考系下的目标相对旋转 qTarget_rel。
   * 允许滞后一帧（§5.3，对观感无影响）。
   */
  private captureTargets(): void {
    const inst = this.ragdoll?.instance;
    if (!inst) return;
    const bones = this.animBones.length === BONE_ORDER.length ? this.animBones : this.ragdoll?.boneNodes ?? [];
    if (bones.length !== BONE_ORDER.length) return;

    const boneByName = new Map<string, Node>();
    BONE_ORDER.forEach((name, i) => boneByName.set(name, bones[i]));

    // 骨骼世界旋转 → 相对父骨骼 → 映射到求解器关节参考系
    // qTarget_rel(q0 系) = conj(qA_bind)·qA_anim⁻¹·qB_anim（B 相对 A 的动画相对旋转，
    // 再经 bind 映射换算到求解器 q0Bind 参考系：qTarget = conj(q0)·q_rel_anim_map …）
    // 简化实现（bind 参考系 = 构建时骨骼 bind 旋转）：直接以动画相对旋转为目标，
    // 电机把刚体相对旋转拉向它 —— 与"骨骼回写以刚体为准"的松耦合跟随一致（§5.3）。
    for (const [childName, motor] of this.motors) {
      const boneB = boneByName.get(childName);
      const parentName = motor.A.name;
      const boneA = boneByName.get(parentName);
      if (!boneB || !boneA) continue;
      // 相对旋转：q_rel = conj(qA) ⊗ qB（A 局部系）
      const qA = boneA.worldRotation;
      const qB = boneB.worldRotation;
      const target = motor.targetRelLocal;
      target.set(
        qA.w * qB.x - qA.x * qB.w - qA.y * qB.z + qA.z * qB.y,
        qA.w * qB.y + qA.y * qB.w + qA.z * qB.x - qA.x * qB.z,
        qA.w * qB.z + qA.z * qB.w + qA.x * qB.y - qA.y * qB.x,
        qA.w * qB.w - qA.x * qB.x - qA.y * qB.y - qA.z * qB.z,
      );
      target.normalize();
    }
  }

  update(dt: number): void {
    this.ensureMotors();
    this.captureTargets();
    this.tickStrength(dt);
    const s = this.enabledMotors ? this.motorStrengthScale : 0;
    for (const motor of this.motors.values()) {
      motor.strength = s;
    }
  }

  // ---- 受击变软 / 起立恢复（§5.4）----

  private recoverTimer = -1; // <0 表示不在恢复曲线中
  private recoverFrom = 1;
  private recoverTo = 0.2;
  private recoverDuration = 1;

  /**
   * 受击变软：strength 按时间曲线 drop → recover（§5.4）。
   * @param strengthDrop 目标强度（0.1~0.3 常用）
   * @param recoverSeconds 恢复时长
   */
  applyHit(strengthDrop: number, recoverSeconds: number): void {
    this.recoverFrom = this.motorStrengthScale;
    this.recoverTo = clamp(strengthDrop, 0, 1);
    this.recoverDuration = Math.max(0.01, recoverSeconds);
    this.recoverTimer = 0;
  }

  private tickStrength(dt: number): void {
    if (this.recoverTimer < 0) return;
    this.recoverTimer += dt;
    const t = Math.min(1, this.recoverTimer / this.recoverDuration);
    // 快速衰减 + 缓恢复：前 20% 时间降到目标，之后 smoothstep 恢复到 strength
    if (t < 0.2) {
      this.motorStrengthScale = this.recoverFrom + (this.recoverTo - this.recoverFrom) * (t / 0.2);
    } else {
      const s = (t - 0.2) / 0.8;
      const smooth = s * s * (3 - 2 * s);
      this.motorStrengthScale = this.recoverTo + (this.strength - this.recoverTo) * smooth;
    }
    if (t >= 1) {
      this.motorStrengthScale = this.strength;
      this.recoverTimer = -1;
    }
  }

  /**
   * 起立恢复（阶段 5，工程路线）：
   * 1. 检测静止（骨盆高度/朝向 + 速度）。
   * 2. 电机把关节以受限速度匀速驱动到起身动画首帧姿态（2~3s，strength 低）。
   * 3. 播放起身动画同时 strength 渐升；完成后交还动画控制。
   * getUpClip 的播放交给调用方/动画状态机；本接口只负责姿态搬运与强度调度。
   */
  requestGetUp(getUpClip: AnimationClip | null, firstFramePoseBones?: Node[]): void {
    void getUpClip;
    void firstFramePoseBones;
    // 阶段 5 实现（骨架）：检测静止 → 目标姿态设为起身动画首帧 → applyHit 反向曲线。
    // 需要动画首帧姿态采样支持，在编辑器场景中实测后补全（M5 为可选里程碑）。
  }

  /** 调试：倒地判定（骨盆高度 + 朝向） */
  isDowned(pelvisUpThreshold = 0.4): boolean {
    const inst = this.ragdoll?.instance;
    if (!inst) return false;
    const pelvis = inst.bodies.get('pelvis');
    if (!pelvis) return false;
    const up = pelvis.rot.applyToVec3(new MRVec3(0, 1, 0), new MRVec3());
    return up.y < pelvisUpThreshold;
  }
}
