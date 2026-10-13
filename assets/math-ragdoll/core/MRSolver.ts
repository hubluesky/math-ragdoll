/**
 * MRSolver —— XPBD 子步主循环（L2 求解层，core 层，禁止 import 'cc'）
 * 见设计文档 §3.2（Small Steps：多子步 × 每子步 1 次迭代）。
 *
 * 子步内固定顺序（确定性要求）：
 *   电机（速度冲量，积分前）→ 重力/阻尼/clamp/积分 → 球关节/铰链位置
 *   → 铰链角对齐 → 锥扭限位 → 铰链行程限位 → 接触（法向+摩擦）→ 速度更新
 *   → 限位速度松弛（单边约束速度投影）
 *
 * 查漏补缺勘误（相对设计文档）：
 * - 电机施加时机统一为"子步积分前"（§3.2 伪代码原放在求解阶段，与 §5.2 矛盾）。
 * - MRSolver 增加 capsules / collisionPairs 字段（原 §4.2 签名缺少碰撞检测所需数据）。
 * - clampVel 增加位置/旋转 NaN 回滚防线（非仅速度钳制）。
 */
import { MRVec3, MRQuat, quatDeltaToAngularVelocity } from './MRMath';
import { MRBody } from './MRBody';
import { MRCapsule } from './MRShape';
import { capsuleCapsule, detectCapsuleGroundPoint } from './MRCollision';
import { MRContact } from './MRContact';
import { MRBallJoint, MRHingeJoint } from './MRJoint';
import { MRConeTwistLimit, solveHingeRange, solveHingeRangeVelocity } from './MRLimit';
import { MRMotor } from './MRMotor';
import { IGround } from './MRGround';

/** 速度钳制（§7.3，NaN 防线之一） */
export const MAX_LINEAR_VELOCITY = 30; // m/s
export const MAX_ANGULAR_VELOCITY = 60; // rad/s

export class MRSolver {
  bodies: MRBody[] = [];
  ballJoints: MRBallJoint[] = [];
  hinges: MRHingeJoint[] = [];
  limits: MRConeTwistLimit[] = [];
  motors: MRMotor[] = [];
  /** 碰撞形状（勘误补充：原设计 MRSolver 签名缺少） */
  capsules: MRCapsule[] = [];
  /** capsule 索引对（由 buildCollisionPairs 生成后填入） */
  collisionPairs: [number, number][] = [];
  ground: IGround | null = null;

  substeps = 8;
  gravity = new MRVec3(0, -9.81, 0);
  linDamp = 0.05;
  /**
   * 角阻尼：衰减时间常数 1/angDamp。0.05（文档默认起点）意味着任何残余角速度
   * 要 ~20s 才衰减到不可见 —— 落地后"骨头原地抽搐很长时间"的直接原因之一
   * （位置法约束每子步注入的微扰没有足够耗散通道）。0.25 → ~4s 静止，
   * 对落地翻滚观感影响可忽略（布娃娃本应快速瘫软）。
   */
  angDamp = 0.25;
  /**
   * 接触求解迭代轮数（每轮重新检测 + 求解）。
   * 位置法接触是隐式弹簧：稳态穿透 d ≈ N/(m/h²)。同刚体双端点接触的旋转耦合
   * 会让单轮求解的稳态穿透膨胀数倍，静摩擦区间（μ·d）随之失真；
   * 迭代收敛后 d 回到理论值，μ 与 tanθ 的经典静摩擦条件成立。
   */
  contactIterations = 3;

  // 接触池（每子步复用，避免热路径分配）
  private contactPool: MRContact[] = [];
  private contactCount = 0;
  /** 地面接触复用对象（序贯 Gauss-Seidel：逐端点检测→求解） */
  private groundContactA: MRContact | null = null;
  private groundContactB: MRContact | null = null;

  /** 诊断：最近一个子步的接触数 */
  get lastContactCount(): number {
    return this.contactCount;
  }

  step(fixedDt: number): void {
    const N = this.substeps;
    const h = fixedDt / N;

    for (let s = 0; s < N; s++) {
      // ---- 电机（子步积分前，速度冲量）----
      for (let i = 0; i < this.motors.length; i++) {
        this.motors[i].apply(h);
      }

      // ---- 1. 预测 ----
      for (let i = 0; i < this.bodies.length; i++) {
        const b = this.bodies[i];
        if (b.isStatic) continue;
        b.prevX.copy(b.x);
        b.prevQ.copy(b.q);
        b.prevR.copy(b.rot);

        b.v.addScaled(this.gravity, h);
        const linDampScale = 1 / (1 + h * this.linDamp);
        const angDampScale = 1 / (1 + h * this.angDamp);
        b.v.scale(linDampScale);
        b.w.scale(angDampScale);
        clampVel(b);

        b.x.addScaled(b.v, h);
        integrateQuat(b.q, b.w, h);
        b.refreshDerived();
      }

      // ---- 2. 求解（每子步 1 次迭代，固定顺序）----
      for (let i = 0; i < this.ballJoints.length; i++) this.ballJoints[i].solve(h);
      for (let i = 0; i < this.hinges.length; i++) this.hinges[i].solve(h);
      for (let i = 0; i < this.hinges.length; i++) this.hinges[i].solveAngleAlignment();
      for (let i = 0; i < this.limits.length; i++) this.limits[i].solve(h);
      for (let i = 0; i < this.hinges.length; i++) solveHingeRange(this.hinges[i]);
      this.detectAndSolveContacts(h);

      // ---- 3. 速度更新 ----
      for (let i = 0; i < this.bodies.length; i++) {
        const b = this.bodies[i];
        if (b.isStatic) continue;
        b.v.set(b.x.x - b.prevX.x, b.x.y - b.prevX.y, b.x.z - b.prevX.z).scale(1 / h);
        // dq = q_new ⊗ conj(q_prev) → ω
        dqTmp.copy(b.q).multiply(dqTmp, tConj.copy(b.prevQ).conjugateInPlace());
        quatDeltaToAngularVelocity(dqTmp, h, b.w);
        clampVel(b);
      }

      // ---- 4. 速度松弛（XPBD 速度阶段：单边/等式约束的速度投影）----
      // 位置法约束的修正经速度回读变成速度，与持续压入源（重力/电机/
      // 约束耦合）形成固定步级方波 —— 贴边关节原地高频抽搐的根源。
      // 顺序与位置阶段一致（确定性）：关节 → 角对齐 → 限位 → 接触。
      for (let i = 0; i < this.ballJoints.length; i++) this.ballJoints[i].solveVelocity();
      for (let i = 0; i < this.hinges.length; i++) this.hinges[i].solveVelocity();
      for (let i = 0; i < this.limits.length; i++) this.limits[i].solveVelocity(h);
      for (let i = 0; i < this.hinges.length; i++) solveHingeRangeVelocity(this.hinges[i]);
      this.detectAndSolveContactVelocities();
    }
  }

  /**
   * 接触求解：每子步 contactIterations 轮。
   * 地面接触的双端点先各自预检测：双端同时接触时按并联分摊——各修 depth/2。
   * 否则每个端点都按"刚体整体穿透"修全额，法向冲量过冲约 2 倍，摩擦锥随之膨胀，
   * μ<tanθ 的斜面也会被钉死（实测）。分摊后修正量合计 = 实际穿透。
   */
  private detectAndSolveContacts(h: number): void {
    this.contactCount = 0;
    // 地面接触的两个复用对象（solve 开头即测量摩擦位移，可顺序复用）
    const gcA = this.groundContactA ??= new MRContact(this.bodies[0], null);
    const gcB = this.groundContactB ??= new MRContact(this.bodies[0], null);
    for (let iter = 0; iter < this.contactIterations; iter++) {
      // 摩擦只在第一轮施加（见 MRContact.solve 注释）；后续轮仅法向收敛
      const applyFriction = iter === 0;
      // 自碰撞：capsule × capsule（过滤表内，序贯检测→求解）
      for (let p = 0; p < this.collisionPairs.length; p++) {
        const [ia, ib] = this.collisionPairs[p];
        const contact = this.acquireContact();
        if (capsuleCapsule(this.capsules[ia], this.capsules[ib], contact)) {
          contact.solve(h, applyFriction);
          this.contactCount++;
        }
      }

      // 地面：capsule × ground（双端并联分摊）
      const ground = this.ground;
      if (ground !== null) {
        for (let i = 0; i < this.capsules.length; i++) {
          const cap = this.capsules[i];
          cap.worldP1(tGroundP1);
          cap.worldP2(tGroundP2);
          const has1 = detectCapsuleGroundPoint(cap, tGroundP1, ground, gcA);
          const has2 = detectCapsuleGroundPoint(cap, tGroundP2, ground, gcB);
          if (has1 && has2) {
            gcA.depth *= 0.5;
            gcB.depth *= 0.5;
          }
          if (has1) {
            gcA.solve(h, applyFriction);
            this.contactCount++;
          }
          if (has2) {
            gcB.solve(h, applyFriction);
            this.contactCount++;
          }
        }
      }
    }
  }

  /**
   * 接触速度松弛：重新检测（位置阶段后位姿未再变，结果一致）后逐接触
   * 消除朝穿透方向的相对法向速度。不做接触池记录，检测开销微秒级。
   */
  private detectAndSolveContactVelocities(): void {
    for (let p = 0; p < this.collisionPairs.length; p++) {
      const [ia, ib] = this.collisionPairs[p];
      const contact = this.acquireContact();
      if (capsuleCapsule(this.capsules[ia], this.capsules[ib], contact)) {
        contact.solveVelocity();
      }
    }
    const ground = this.ground;
    if (ground !== null) {
      const gcA = this.groundContactA ??= new MRContact(this.bodies[0], null);
      const gcB = this.groundContactB ??= new MRContact(this.bodies[0], null);
      for (let i = 0; i < this.capsules.length; i++) {
        const cap = this.capsules[i];
        cap.worldP1(tGroundP1);
        cap.worldP2(tGroundP2);
        if (detectCapsuleGroundPoint(cap, tGroundP1, ground, gcA)) gcA.solveVelocity();
        if (detectCapsuleGroundPoint(cap, tGroundP2, ground, gcB)) gcB.solveVelocity();
      }
    }
  }

  /** 从池中取下一个接触对象（不足时扩容；未命中检测的槽位由计数器覆盖） */
  private acquireContact(): MRContact {
    if (this.contactCount >= this.contactPool.length) {
      const placeholder = this.bodies[0];
      const c = new MRContact(placeholder, null);
      this.contactPool.push(c);
      return c;
    }
    return this.contactPool[this.contactCount];
  }
}

// ---------------------------------------------------------------------------
// 辅助
// ---------------------------------------------------------------------------

const tConj = new MRQuat();
const dqTmp = new MRQuat();
const omegaQuat = new MRQuat();
const tGroundP1 = new MRVec3();
const tGroundP2 = new MRVec3();

/** 预测阶段速度钳制 + NaN 防线（清零/回滚） */
export function clampVel(b: MRBody): void {
  // NaN / Inf 防护：速度清零、位姿回滚到子步快照
  if (!b.v.isFinite() || !b.w.isFinite()) {
    b.v.set(0, 0, 0);
    b.w.set(0, 0, 0);
  }
  if (!b.x.isFinite() || !b.q.isFinite()) {
    b.x.copy(b.prevX);
    b.q.copy(b.prevQ).normalize();
    b.v.set(0, 0, 0);
    b.w.set(0, 0, 0);
    return;
  }
  const vLen = b.v.length();
  if (vLen > MAX_LINEAR_VELOCITY) b.v.scale(MAX_LINEAR_VELOCITY / vLen);
  const wLen = b.w.length();
  if (wLen > MAX_ANGULAR_VELOCITY) b.w.scale(MAX_ANGULAR_VELOCITY / wLen);
}

/** q̇ = ½ ω_quat ⊗ q（ω 世界系，左乘）；q += q̇·h 后归一化 */
export function integrateQuat(q: MRQuat, w: Readonly<MRVec3>, h: number): void {
  omegaQuat.set(w.x, w.y, w.z, 0);
  const dq = dqInt.multiply(omegaQuat, q); // ω ⊗ q
  const halfH = 0.5 * h;
  q.set(q.x + dq.x * halfH, q.y + dq.y * halfH, q.z + dq.z * halfH, q.w + dq.w * halfH);
  q.normalize();
}
const dqInt = new MRQuat();
