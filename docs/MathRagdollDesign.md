# 纯数学布娃娃系统设计文档（Cocos Creator 3.8.7 / TypeScript 版）

> 本文档是完整的实现规格书，面向实现者（人或 LLM）。目标是在 **Cocos Creator 3.8.7** 中**不使用任何物理系统**（builtin / cannon / bullet / PhysX 模块以及一切 `cc` 物理相关 API 一律禁用），用纯数学（XPBD 位置法动力学）实现角色布娃娃，并预留主动布娃娃扩展。
>
> 数学路线谱系：Verlet 积分 → Position Based Dynamics (PBD) → XPBD → XPBD 刚体+关节（本文）。与 Obi 插件模拟绳子同源，扩展到刚体即布娃娃。
>
> **v1.2（2026-10-10，实现期勘误）**：M1~M3 已实现并通过全部 vitest 验收（20/20）。实现期发现并修正了 v1.1 的若干公式/规格问题，全部以 **Müller 2020 论文与 Ten Minute Physics 第 25 课参考实现（25-joints.html，MIT）为准**，并用测试实证。修订点已在正文对应小节就地标注 `[v1.2]`，主要包括：限位修正改为**纯角修正**（§3.5，v1.1 的"绕锚点旋转"实测注入能量导致全身爆炸）、接触法线约定统一（§3.6.1/§3.6.4 原文自相矛盾）、capsule-ground 双端并联分摊（§3.6.3）、摩擦测量时机与"仅首轮施加"（§3.6.4）、铰链行程限位测量对象（§3.5.2）、IGround/实例化构建器移入 core（§2.1）、MRSolver/MRBody 签名补缺（§4.2）、骨骼回写父→子顺序（§6.3）、§9 新增坑 13~19。

---

## 0. 给实现者的工作须知（必读）

1. **语言与环境**：TypeScript，Cocos Creator 3.8.7。`import { _decorator, Component, Node, SkinnedMeshRenderer, Vec3, Quat } from 'cc'` 仅允许出现在 `runtime/` 层。
2. **禁止使用的 API**：一切物理模块内容——`cc` 的 physics 体系（RigidBody、Collider、raycast、物理材质等）。碰撞检测和响应全部自己写解析数学。（可选：项目设置里直接停用物理模块，避免误用。）
3. **核心参考资料**（实现前先读，公式有出入以参考为准）：
   - Müller et al. 2020, *Detailed Rigid Body Simulation with Extended Position Based Dynamics*：https://matthias-research.github.io/pages/publications/PBDBodies.pdf —— 本文的主要依据，关节、限位、接触公式都在里面。
   - Ten Minute Physics 第 22 课（刚体）、第 25 课（关节）：https://github.com/matthias-research/pages —— **JS/Web demo 参考实现，和本项目同为 TS/JS 生态，可直接对照**。
   - Macklin et al. 2019, *Small Steps in Physics Simulation*（子步长策略，见 matthias-research 出版页）。
   - Habrador/Ten-Minute-Physics-Unity：https://github.com/Habrador/Ten-Minute-Physics-Unity （C# 移植，结构组织可参考，MIT）。
4. **验证方式（替代 Unity Test Runner）**：`core/` 层是**纯 TypeScript、禁止 import 'cc'**，因此可以在 Node 里直接跑单元测试。项目根目录配置 **vitest**（`npm i -D vitest`，`npx vitest run`）。**每个阶段（§8）的验收测试全部通过才能进入下一阶段**；向用户报告完成时附测试运行结果。引擎表现类验收项在编辑器测试场景人工/半自动检查。
5. **实现纪律**：不允许留 TODO/占位代码；不确定公式方向（正负号、旋转方向）时写测试实证，不许靠猜。
6. 数值精度：TS number 即 float64，**求解器内部全程双精度**（比 Unity float32 更利于稳定与确定性），仅在回写骨骼时转成引擎类型。确定性要求：求解器内禁止 `Math.random`、禁止 `Date.now` 影响模拟、数组遍历顺序固定 → 同输入同输出。

---

## 1. 目标与范围

### 1.1 需求清单（用户原始需求 → 本设计落点）

| # | 需求 | 落点 |
|---|------|------|
| 1 | 布娃娃自碰撞 | 碰撞层：胶囊体×胶囊体解析检测 + 手工配对过滤表（§3.6、§4.3） |
| 2 | 不与场景物体碰撞，只做自碰撞 | 碰撞层只实现 capsule-capsule 与 capsule-ground，无通用碰撞系统 |
| 3 | 地面碰撞 | 碰撞层：`IGround` 接口（平面实现 + 高度场接口预留）（§3.6.3） |
| 4 | 关节角度限制 | 求解层：球关节 + 锥面/扭转限位 + 铰链关节（§3.4、§3.5） |
| 5 | 未来扩展：受击变软、起立恢复 | 主动控制层（PD 电机）+ 动作目标层，参数化挂载点（§5） |
| 6 | 纯数学，不用物理系统 | 全文前提；确定性、可测 |

### 1.2 非目标（明确不做）

- 与场景中任意网格/道具的碰撞（将来若需要，扩展新的解析图元对即可，架构不变）。
- 与其他角色的布娃娃互碰（将来扩展：把对方骨骼的 capsule 加入配对表即可，接触约束是通用的，现在不做）。
- 网格级精确碰撞、连续碰撞检测（CCD）——用小步长（子步）抑制穿透，足够。
- 强化学习控制（起立恢复走"预设起身动画+姿态混合"的工程路线，见 §5.4）。
- 多角色优化（Worker/wasm 化、LOD）——留作后续，不在本设计内。

---

## 2. 总体架构

四层结构，每层只依赖下一层。用户需求 5 的扩展只动第 3、4 层。

```
┌─────────────────────────────────────────────────────┐
│ L4 动作目标层   目标姿态从哪来：动画剪辑 / 程序化 /    │  ← 需求5扩展点（起身动画、受击表现）
│    (TargetPose) 外部输入。只输出"每关节目标相对旋转"。│
├─────────────────────────────────────────────────────┤
│ L3 主动控制层   PD 电机：把骨骼拉向目标姿态，输出      │  ← 需求5"受击变软"= 调低本层参数
│    (Active)     力矩。刚度/最大力矩可逐关节配置。      │
├─────────────────────────────────────────────────────┤
│ L2 求解层       XPBD 刚体求解器：重力积分、球关节、    │  ← 需求4
│    (Solver)     锥面/扭转限位、铰链关节、接触约束。    │
├─────────────────────────────────────────────────────┤
│ L1 碰撞层       capsule×capsule、capsule×ground 的    │  ← 需求1、2、3
│    (Collision)  解析检测 + 配对过滤表 + 摩擦。        │
├─────────────────────────────────────────────────────┤
│ L0 数学库       四元数、3x3 矩阵、惯性张量、轴角、     │
│    (Math)       swing-twist 分解、数值防护。          │
└─────────────────────────────────────────────────────┘
```

### 2.1 文件结构

```
<项目根>/
  tests/                        vitest 单元测试（Node 环境，不依赖引擎）
    M1_body.test.ts  M2_joint.test.ts  M3_ragdoll.test.ts ...
  assets/
    math-ragdoll/
      core/                     纯 TS，禁止 import 'cc'（可被 tests 直接加载）
        MRBody.ts               刚体状态与积分数据（含 rotateBodyAboutPoint 工具）
        MRShape.ts              Capsule 定义（局部线段端点+半径）+ 盒惯性公式
        MRCollision.ts          capsule-capsule / capsule-ground 检测（纯函数）
        MRSolver.ts             XPBD 子步主循环
        MRJoint.ts              球关节 / 铰链关节 + 纯角修正原语 applyAngularPair
        MRLimit.ts              锥面扭转限位 / 铰链行程限位
        MRContact.ts            接触约束 + 摩擦
        MRMotor.ts              PD 电机（L3）
        MRSkeletonDef.ts        骨骼拓扑、限位参数、配对过滤表定义（纯数据接口）
        MRGround.ts             [v1.2] IGround 接口 + 平面纯实现 + 高度场基类（vitest 需要）
        MRDefaultSkeleton.ts    [v1.2] 13 体合成人形 def + SkeletonDef→实例纯 TS 构建器
        MRMath.ts               MRVec3/MRQuat/MRMat3、轴角、swing-twist、3x3 求解、防护
      runtime/                  依赖 'cc'，薄集成层
        MRRagdoll.ts            Component 入口：构建、固定步长累加器、步进、回写骨骼
        MRRagdollBuilder.ts     从骨骼节点表生成 SkeletonDef（§6.1）
        MRGround.ts             MRGroundPlane Component（包装 core 的 IGround）
        MRActiveController.ts   阶段4：目标姿态采集 + 电机驱动 + 受击变软 API
```

命名约定：前缀 `MR`（MathRagdoll）。`core/` 内的向量/四元数类型是**本包自定义的 `MRVec3/MRQuat/MRMat3`（float64）**，不是 cc 类型；`runtime/` 负责与 `cc.Vec3/cc.Quat` 互转。这样 core 可在 Node 中确定性测试。

[v1.2] 职责划分调整：`SkeletonDef → MRBody 图`的实例化构建是纯 TS 逻辑且 vitest（M3）必需，故放入 core（`MRDefaultSkeleton.instantiateSkeleton`），runtime 的 `MRRagdollBuilder` 只负责"骨骼节点表 → SkeletonDef"，两者在 runtime 汇合复用。

---

## 3. 核心数学（L0 + L1 + L2）

> 记号：刚体 i 的状态为质心位置 `x`（世界系）、旋转 `q`（世界系，body→world）、线速度 `v`、角速度 `ω`（世界系）、质量 `m`、逆质量 `invM = 1/m`、**局部系**逆惯性张量 `invIlocal`（3x3 对角）。
> 世界系逆惯性：`invI = R · invIlocal · Rᵀ`，其中 `R` 为 `q` 对应的旋转矩阵，每个子步更新。
> 点 `r_local`（体局部系）的世界位置：`p = x + R · r_local`；记 `r = R · r_local`（COM→该点，世界系）。

### 3.1 刚体（MRBody）

```ts
export class MRBody {
  x = new MRVec3();  q = new MRQuat();    // 世界系位姿
  v = new MRVec3();  w = new MRVec3();    // 线速度 / 角速度 ω（世界系）
  invMass = 0;
  invIlocal = new MRMat3();               // 局部逆惯性（对角）
  prevX = new MRVec3();  prevQ = new MRQuat();  // 子步开始时快照（速度更新与摩擦用）
  isStatic = false;                       // 地面等静态体 invMass=0, invI=0
  // capsule 外观与碰撞形状见 MRShape，不挂在 body 上，由 SkeletonDef 关联
}
```

惯性张量：**默认用 capsule 包围盒的实心盒公式**（对布娃娃的观感完全够用，游戏 ragdoll 常规做法）：

```
盒半尺寸 (hx, hy, hz)，质量 m：
invIlocal = diag( 3/(m(hy²+hz²)),  3/(m(hx²+hz²)),  3/(m(hx²+hy²)) )
```

可选精确化（不做默认）：capsule 分解为圆柱+两半球，各部分绕 COM 用平行轴定理求和。**不要**用球形近似躯干——躯干转动惯量失真会让翻滚观感变差。

质量与 capsule 尺寸：由 `MRRagdollBuilder` 按骨骼节点位置自动生成（§6.1），质量按 §7.1 人体比例表归一化。

### 3.2 XPBD 子步主循环（MRSolver）

采用 **Small Steps** 策略：多子步 × 每子步 1 次迭代，代替少子步 × 多迭代（稳定性关键来源）。

```
每个固定步 fixedDt（见 §6.2 累加器）：
  substeps N = 8，h = fixedDt / N            # h ≈ 2~4ms
  for s in 0..N-1:
    # ---- 0. 电机（阶段4）[v1.2：统一为"子步积分前"施加速度冲量，消除 §5.2 与本节的矛盾] ----
    for motor in motors:            applyMotor(motor, h)
    # ---- 1. 预测 ----
    for body b (非静态):
      b.prevX = b.x;  b.prevQ = b.q;  b.prevR = R(b.q)    # 快照（速度更新与摩擦用）
      b.v += gravity * h
      b.v *= 1/(1 + h*linDamp);  b.w *= 1/(1 + h*angDamp)     # 可选微阻尼，默认 0.05
      clampVel(b)                                  # |v|≤30, |w|≤60，NaN 时清零并回滚位姿
      b.x += b.v * h
      b.q = normalize(b.q + 0.5*h*quat(0, b.w)*b.q)   # q̇ = ½ ω_quat ⊗ q（ω 世界系，左乘）
      b.refreshDerived()                           # rot/invIworld = R·invIlocal·Rᵀ
    # ---- 2. 求解（每子步 1 次迭代，固定顺序）----
    foreach joint in joints:        solveJoint(joint)      # 球关节/铰链位置
    foreach hinge in hinges:        solveAngleAlignment()  # 铰链角对齐
    foreach limit in limits:        solveLimit(limit)      # 锥面扭转（纯角修正）
    foreach hinge in hinges:        solveHingeRange()      # 铰链行程
    for iter in 0..contactIterations-1:                    # [v1.2：接触多轮，每轮重新检测]
      foreach contact in detectContacts(): solveContact(contact, h)
    # ---- 3. 速度更新 ----
    for body b (非静态):
      b.v = (b.x − b.prevX) / h
      b.w = quatDeltaToAngularVelocity(b.q * conj(b.prevQ), h)
```

[v1.2] **rot 缓存刷新**：`refreshDerived` 只在预测阶段调用（invIworld 按论文做法整子步缓存）；
但任何**位置修正改写了 q 之后必须同步刷新 `body.rot`**（球关节/限位的角修正、接触角修正内部处理）。
否则同一子步内后续约束用过期旋转矩阵换算锚点/接触点，角修正的整体贡献被延迟一个子步，
约束求解失去一阶精确性（实测单接触只消除 41% 的穿透）。

[v1.2] **接触多轮迭代**：同刚体双端点接触（如平躺 capsule 两端同时触地）的旋转耦合会让
单轮求解的法向冲量过冲约 2 倍；多轮"重新检测+求解"让穿透收敛到理论值 g·cosθ·h²/2，
静摩擦条件回到经典的 tanθ ≤ μ（M1 摩擦验收依赖此性质）。默认 `contactIterations = 3`。

`quatDeltaToAngularVelocity`：

```
dq = q_new * conj(q_prev)
if dq.w < 0: 取反所有分量                        # 走短弧
angle = 2 * acos(clamp(dq.w, -1, 1))
if sin(angle/2) < 1e-8:  w = 2 * dq.xyz / h     # 小角近似
else:                    w = (dq.xyz / sin(angle/2)) * (angle / h)
```

注意：
- **跳过陀螺项**（ω×Iω），游戏 ragdoll 标准做法，Müller 论文同款，利于稳定。
- `q + 0.5*h*ω_quat⊗q` 后必须 normalize。
- 静态体（地面）invMass=0，所有分配公式中自动不移动。

### 3.3 约束求解通用公式

位置法统一模式：约束误差 `C`，广义逆质量 `w`，修正乘子 `λ = −C / w`（XPBD 带 compliance 时 `λ = −C / (w + α̃)`，`α̃ = compliance / h²`），然后按 `λ·invM`（线）和 `λ·invI`（角）分配。

**点脉冲修正**（对刚体 i 在锚点 r 处沿方向 n 施加标量脉冲 λ）：

```
Δx_i = (λ / m_i) · n
Δφ_i = λ · invI_i · (r × n)          # 绕 COM 的角修正向量（轴角），世界系
q_i ← axisAngle(Δφ_i) ⊗ q_i          # 世界系修正左乘
```

**单点标量约束的广义逆质量**（方向 n、COM→点向量 r）：

```
w_i = 1/m_i + (r × n)ᵀ · invI_i · (r × n)
```

**角修正防护**：单次角修正 `|Δφ|` 钳制到 0.2 rad；单次线修正 `|Δx|` 钳制到 0.05 m。防止求解爆炸。

### 3.4 球关节（MRBallJoint）

三自由度位置约束：父子两体在锚点重合。锚点在两体局部系固定：`rALocal`、`rBLocal`（构建时捕获）。

```
pA = x_A + R_A·rA_local,  pB = x_B + R_B·rB_local,  rA = R_A·rA_local,  rB = R_B·rB_local
C = pA − pB                                    # 3 维向量约束
K = (invM_A + invM_B)·I₃
    − [rA]× · invI_A · [rA]×
    − [rB]× · invI_B · [rB]×                   # 3x3 对称半正定；[r]× 为 r 的反对称叉乘矩阵
解 (K + α̃·I₃)·λ = −C                           # 3x3 线性方程，compliance 默认 0 → 直接解 K·λ=−C
施加：x_A += λ·invM_A;  Δφ_A = invI_A·(rA × λ);  q_A ← axisAngle(Δφ_A)⊗q_A
      x_B −= λ·invM_B;  Δφ_B = −invI_B·(rB × λ); q_B ← axisAngle(Δφ_B)⊗q_B
```

实现细节：3x3 求解用带部分主元的高斯消元（`MRMath.solve3x3`）；`invI` 世界系每子步算一次缓存到 body 上。

### 3.5 关节限位（MRLimit）

**需要两种限位**，覆盖全身关节（默认参数见 §7.2）：

> **[v1.2] 修正方式的重大勘误：限位用"纯角修正"，禁止"绕锚点旋转"。**
>
> v1.1 的"绕锚点 a 旋转"方案实测会**注入能量**：绕锚点旋转平移 COM（Δx ≈ φ·|COM−a|），
> 速度回读 v = Δx/h 把长杠杆限位修正（锚点距 COM 可达 0.4m）变成每子步数 m/s 的速度
> 脉冲；约束间（关节↔限位）拉锯下布娃娃永不静止，且子步越多爆炸越快（h 越小 Δx/h 越大）。
>
> 正确做法（Müller 2020 / Ten Minute Physics 第 25 课同款）：限位修正**只转姿态、COM 不动**
> ——构造旋转差向量 `corr`（方向 = 修正旋转轴，大小 = sin(角差)），两体各施加
> `0.5·invI·corr` 的角修正（applyAngularPair，含 0.5 收敛系数）。锚点位姿由球关节
> 位置约束在后续求解中维护，该拉锯不产生 COM 动量，天然稳定。此方案的修正量为
> sin(角差)（线性化欠修正），靠多子步收敛（Small Steps 哲学）；M3 验收（超限启动
> 1 个固定步内回到限内 0.5°）实测通过。

#### 3.5.1 锥面+扭转限位（MRConeTwistLimit）—— 肩、髋、颈、腰、踝、腕

构建时捕获：主轴 `axisALocal/axisBLocal`（对应参考实现 x 轴，bind 时世界对齐，如指向
远端骨骼方向）、扭转测量垂直参考轴 `twistRefALocal/twistRefBLocal`（对应参考实现 y 轴，
bind 时与主轴世界对齐的正交轴）、bind 相对旋转 `q0 = conj(q_A_bind) · q_B_bind`。

```
# ---- 锥面（swing）---- [v1.2：参考实现语义]
a0 = R_A · axisA_local,  a1 = R_B · axisB_local
n = normalize(cross(a0, a1))            # 两轴叉积（a0∥a1 退化时跳过本子步）
phi = getAngle(n, a0, a1)               # 有符号角，恒 ≥ 0（见下方 getAngle）
if phi > coneMax:
    limitAngle(n, a0, a1, 0, coneMax)   # 内部：ra = a0 绕 n 转 clamp(phi)；corr = cross(ra, a1)
                                        # applyAngularPair(A, B, corr) 纯角修正

# ---- 扭转（twist）----
n2 = normalize(a0 + a1)                 # 主轴角平分线（测量轴）
vA = R_A · twistRefA_local,  vB = R_B · twistRefB_local
vA、vB 投影到 ⊥n2 平面并归一化
limitAngle(n2, vA, vB, twistMin, twistMax)

# ---- 测角原语 getAngle(n, a, b)（参考实现）----
c = cross(a, b);  phi = asin(clamp(c·n, -1, 1))
if a·b < 0: phi = π − phi               # 象限修正
wrap 到 (−π, π]
```

[v1.2] 符号约定（测试实证）：扭转测量角以 "vA → vB 绕 n2" 为正；绕主轴正向旋转 B 在
该定义下为负角。超限判定按两侧界（twistMin/twistMax），不做单侧假设。

#### 3.5.2 铰链关节限位（MRHingeJoint + 行程限位）—— 膝、肘

铰链 = 球关节（位置）+ 角对齐（消除两自由度）+ 1 个行程限位。

构建时捕获：锚点、父体铰链轴 `hingeAxisALocal`（如膝盖屈伸轴）、子体铰链轴 `hingeAxisBLocal`、父体零位参考轴 `refAxisALocal`（⊥ 铰链轴）、**子体零位参考轴 `refAxisBLocal`**（[v1.2] 新增：bind 时与 refAxisALocal 世界对齐，行程测量用）、行程范围 `[aMin, aMax]`。

```
# ---- 角对齐 [v1.2：参考实现方式，单一向量约束同时消除两自由度] ----
uA = R_A·hingeAxisA_local,  uB = R_B·hingeAxisB_local
corr = cross(uA, uB)                    # 旋转差向量：方向 = 修正轴，大小 = sin(夹角)
applyAngularPair(A, B, corr)            # 纯角修正（COM 不动）
# （v1.1 的"C1 = dot(uB, vA) / C2 = dot(uB, cross(uA,vA))"两个标量约束与之等价，
#   但参考实现的单一叉积向量约束更简洁且实证收敛，采用之。）

# ---- 行程限位 ----
# [v1.2] 测量对象勘误：v1.1 写"测 uB 相对 vA 的角"自相矛盾——uB（铰链轴）已被
# 角对齐钉死在 uA 上，其垂直平面投影恒为零向量。实际测量的是 B 体的零位参考方向：
n  = R_A·hingeAxisA_local               # 世界铰链轴
vA = R_A·refAxisA_local,  vB = R_B·refAxisB_local   # 各自投影到 ⊥n 平面并归一化
limitAngle(n, vA, vB, aMin, aMax)       # 同 §3.5.1 的 limitAngle（纯角修正）
```

实现时先在 M2 用单摆验证铰链自由度正确（只绕铰链轴转），再加行程限位。

### 3.6 碰撞检测（L1，MRCollision）

全部解析闭式，无加速结构需求（13 体 ≈ 78 对，每子步全算只需微秒级）。

#### 3.6.1 capsule × capsule

Capsule 定义：体局部系线段端点 `p1Local, p2Local` + 半径 `r`（构建时按骨骼生成，§6.1）。检测时变换到世界系。

线段-线段最近距离（标准算法，Ericson《Real-Time Collision Detection》§5.1.9，含平行退化分支）得最近点 `cA、cB`：

```
d = |cB − cA|
if d < rA + rB:  生成接触
    n = d > 1e-9 ? (cB−cA)/d : 任一正交轴          # 法线从 A 指向 B（约定！）
    depth = rA + rB − d
    接触点：各体上的局部偏移 = worldToLocal(cA / cB)
```

[v1.2] 法线约定全文统一：**n 从 A 指向 B，法向修正 A 沿 −n 退、B 沿 +n 退**。
v1.1 的 §3.6.4 公式（x_A += λn / x_B −= λn）与此约定相反，按本约定修正（见 §3.6.4）。

#### 3.6.2 配对过滤表（自碰撞核心）

- 默认规则：**骨骼关节图的相邻体（graph distance ≤ 1）互斥**，其余两两检测。这样大腿-大腿、手-躯干、前臂-对侧大腿等自然生效。
- `MRSkeletonDef` 提供 `ignorePairs`（额外排除）与 `forcePairs`（强制包含）覆盖列表，可逐角色调整。
- 典型 13 体角色：78 对过滤后剩 ~50 对，单子步 ~50 次线段距离计算，成本可忽略。

#### 3.6.3 capsule × 地面（IGround）

```ts
export interface IGround {
  /** 返回点 pos 处地面高度；normal 输出地面法线（世界系，朝上） */
  sampleHeight(pos: Readonly<MRVec3>, normal: MRVec3): number;
}
```

- **MRGroundPlane**（默认实现）：`y = height` 平面，法线 `(0,1,0)`；[v1.2] 支持**任意平面**
  （点 + 法线，M1 的 30° 斜面摩擦测试需要），纯实现放 core（`core/MRGround.ts`）供 vitest
  直接使用，runtime 的 `MRGround` Component 包装它。
- **高度场接口**（预留）：采样 + 有限差分求法线，本阶段给平面等价的简单实现即可。
- 检测 [v1.2 勘误]：对线段**两端点各自独立检测**（有符号距离沿线段是线性函数，端点检测
  对无限平面是精确的），depth > 0 的端点各生成一个接触。
  **v1.1 的"取最低者单点检测"会导致平躺 capsule 绕单接触点跷跷板抖动**，无法满足 M1
  的"静止无抖动"验收。
- **双端并联分摊**（实现关键）：同一体两端同时接触时，各端点按 `depth/2` 修正——否则每个
  端点都按"刚体整体穿透"修全额，法向冲量过冲约 2 倍，摩擦锥随之膨胀，μ < tanθ 的斜面
  也会被钉死（M1 摩擦验收实测依赖此分摊）。

#### 3.6.4 接触约束 + 摩擦（MRContact）

接触是非弹性（restitution=0，ragdoll 需要）。每个接触：`bodyA, bodyB（可 null=静态地面）、n（A→B）、depth、rALocal、rBLocal（接触点体局部偏移）、friction`。

```
# ---- 法向（[v1.2] n: A→B；A 沿 −n 退、B 沿 +n 退。v1.1 公式符号与此约定相反，已修正）----
rA = R_A·rA_local, rB = R_B·rB_local
w = wA + wB，其中 wi = invM_i + (r_i × n)ᵀ invI_i (r_i × n)   # 静态体 w=0
λ = depth / w                                   # 恒为正
x_A −= (λ·invM_A)·n;   Δφ_A = −λ·invI_A·(rA × n)
x_B += (λ·invM_B)·n;   Δφ_B = +λ·invI_B·(rB × n)

# ---- 摩擦（Macklin 位置法摩擦）[v1.2：两个关键实现约束，缺一则 μ 完全失效]----
# (1) 切向相对位移 dispT 必须在法向求解【之前】测量（材料点自子步开始的位移）；
# (2) 每个子步只在第一轮接触迭代施摩擦，后续轮次仅解法向。
dA = (x_A + R_A·rA_local) − (prevX_A + prevR_A·rA_local)   # R 用子步开始缓存 rot
dB = 同理
disp = dA − dB
dispT = disp − n·dot(disp, n)                   # 切向分量
maxT = μ · depth                                # μ：地面 0.7，自碰撞 0.3（§7.3）
if |dispT| > maxT: dispT *= maxT/|dispT|
沿 −dispT 方向对 A、+dispT 对 B 施加同法向机制的修正（标量 = |dispT| / w_t，n 换成 dispT 方向）
```

[v1.2] 实测记录（为什么是这两条约束）：
- 若 dispT 在法向求解后测量，法向角修正对接触点的位移会混入 dispT 被摩擦"消除"，
  形成系统性推动（μ 越大推得越快的正反馈，单 capsule 斜面恒速爬行）。
- 若多轮迭代每轮都施摩擦，累计消除量远超 μ·depth 预算（每轮按"新测 dispT"消除，
  双端点互相喂养），实测 μ=0.4 与 μ=0.7 的斜面行为无差别（都被钉死）。

求解顺序：先法向后摩擦；接触解在关节解**之后**（顺序固定即可，子步多时顺序不敏感）。

---

## 4. 数据结构与关键接口

### 4.1 骨骼体表（13 体标准人形）

```
pelvis(root) ← 无父关节          head       ← chest
chest       ← pelvis             L/R upperArm ← chest
L/R thigh   ← pelvis             L/R forearm  ← upperArm
L/R shin    ← thigh              L/R hand     ← forearm
```

[v1.2] 体数口径澄清：§3.6 的"13 体 ≈ 78 对"按 **手独立成体、脚并入小腿** 计
（C(13,2) = 78）；本表即 13 体口径。脚并入小腿质量（§7.1 的 foot"（含）"并入 shin）。
`core/MRDefaultSkeleton.createDefaultHumanoidDef()` 为对应合成实现（M3 验收用）。

### 4.2 核心类签名（实现者照此写，可增不可减核心语义）

```ts
// core/MRJoint.ts
export class MRBallJoint {
  A: MRBody; B: MRBody;
  rALocal: MRVec3; rBLocal: MRVec3;   // 锚点在两体局部系
  compliance = 0;
}
export class MRHingeJoint {
  A: MRBody; B: MRBody;
  rALocal: MRVec3; rBLocal: MRVec3;
  hingeAxisALocal: MRVec3; hingeAxisBLocal: MRVec3; refAxisALocal: MRVec3;
  refAxisBLocal: MRVec3;              // [v1.2] 子体零位参考方向（行程限位测量用，§3.5.2）
  aMin = 0; aMax = Math.PI;
}

// core/MRLimit.ts
export class MRConeTwistLimit {
  A: MRBody; B: MRBody;
  rALocal: MRVec3; rBLocal: MRVec3;   // 锚点（构建/诊断用）
  axisALocal: MRVec3; axisBLocal: MRVec3;             // 主轴（参考实现 x 轴）
  twistRefALocal: MRVec3; twistRefBLocal: MRVec3;     // [v1.2] 扭转测量垂直轴（参考实现 y 轴）
  q0Bind: MRQuat;                     // conj(qA_bind)·qB_bind（诊断用）
  coneMax = 0; twistMin = 0; twistMax = 0;   // 弧度
}

// core/MRContact.ts
export class MRContact {
  A: MRBody; B: MRBody | null;        // B=null 表示静态地面
  n: MRVec3; depth: number;
  rALocal: MRVec3; rBLocal: MRVec3;
  friction: number;
}

// core/MRSkeletonDef.ts —— 纯数据，可由 JSON 构建
export interface BodyDef { name: string; parent: string | null;
  massRatio: number; capsule: { p1Local: [number,number,number]; p2Local: [number,number,number]; radius: number };
  bindPos: [number,number,number]; }  // [v1.2] COM 的 bind 世界位置（实例化必需）
export interface JointDef { type: 'ball' | 'hinge'; a: string; b: string;
  limit?: { coneMax?: number; twistMin?: number; twistMax?: number;
            aMin?: number; aMax?: number; hingeAxis?: [number,number,number];
            refAxis?: [number,number,number]; refAxisB?: [number,number,number];
            axisA?: [number,number,number]; axisB?: [number,number,number] }; }
// [v1.2] 锥扭限位增加 axisA/axisB（关节主轴）；铰链增加 refAxisB（子体零位参考）。
// v1.1 的 JointDef 无法从纯拓扑数据构建限位与位姿。
export interface SkeletonDef {
  bodies: BodyDef[]; joints: JointDef[];
  ignorePairs: [string, string][];    // 额外排除（默认已排除 graph distance ≤1）
  forcePairs: [string, string][];     // 强制包含
  totalMass: number;
}

// core/MRSolver.ts
export class MRSolver {
  bodies: MRBody[]; ballJoints: MRBallJoint[]; hinges: MRHingeJoint[];
  limits: MRConeTwistLimit[]; motors: MRMotor[];
  capsules: MRCapsule[];              // [v1.2] 碰撞形状（v1.1 签名缺少，检测必需）
  collisionPairs: [number, number][]; // [v1.2] 过滤表索引对（buildCollisionPairs 生成）
  contactIterations = 3;              // [v1.2] 接触求解轮数（§3.2）
  ground: IGround | null;
  substeps = 8; gravity = new MRVec3(0, -9.81, 0);
  linDamp = 0.05; angDamp = 0.05;
  step(fixedDt: number): void;        // §3.2 主循环
}

// [v1.2] MRBody 的每子步派生缓存（v1.1 未列出）：invIworld（整子步缓存）、
// rot（R(q)，位置修正后必须同步刷新）、prevR（子步开始旋转快照，摩擦用）。
```

---

## 5. 主动控制层（L3/L4，阶段4实现；阶段1-3 只留接口）

### 5.1 架构约束（为需求5预留的正确做法）

- 求解器**不知道**"动画"存在。L4 只产出每关节的**目标相对旋转** `qTarget_rel`；L3 把它转成力矩。
- 主动/被动切换 = 电机的 `strength`（0~1 全局标量）与逐关节 `kp/kd/τmax` 调节。**受击变软 = strength 短时间衰减到 0.1~0.3 再按曲线恢复**，不改求解器一行代码。

### 5.2 PD 电机（MRMotor，速度冲量式）

```
每子步开始（积分前）对每个有关节的骨骼对 (A=父, B=子)：
    q_rel  = conj(q_A) · q_B
    q_err  = qTarget_rel · conj(q_rel)
    if q_err.w < 0: 取反（短弧）
    (axis_local, θ) = toAxisAngle(q_err)        # A 局部系
    axis_world = R_A · axis_local
    τ = kp·θ·axis_world − kd·(w_B − w_A)        # 世界系力矩
    τ = clampMagnitude(τ, τmax * strength)
    w_B += invI_B·τ·h；w_A −= invI_A·τ·h        # 牛三，作用反作用
    单子步 |Δω| ≤ 20 rad/s（防过冲钳制）
```

参数整定起点：`kd = 2·√kp`（临界阻尼）；`kp` 躯干 200~400、四肢 80~200、踝 60~120；`τmax` 按 §7.2 备注与实测。**电机的调参占阶段4 大半时间，属预期**。

### 5.3 目标姿态采集（MRActiveController）

- 正常运动时 `AnimationController` 照常播放（物理层之外）；每帧物理步进前，读取各关节骨骼节点世界旋转，换算成 `qTarget_rel`（相对父骨骼，并映射到求解器的关节参考系——构建时存 bind 映射）。允许落后一帧（目标滞后一帧对观感无影响）。
- 骨骼回写始终以刚体为准（§6.3），动画只是"目标"，所以动作是松耦合跟随之感（TABS/Human Fall Flat 风格）。

### 5.4 受击变软 / 起立恢复（接口预留，阶段5）

```ts
export class MRActiveController {
  strength = 1;                                   // 0=纯布娃娃, 1=全力跟随
  applyHit(strengthDrop: number, recoverSeconds: number): void;   // 受击变软
  requestGetUp(getUpClip: AnimationClip): void;   // 起立（阶段5）
}
```

- **受击变软**（阶段4验收项）：`strength` 按时间曲线 `0→drop→recover`；倒地判定（骨盆高度/朝向）决定是否进入完全被动直到静止。
- **起立恢复**（阶段5，工程路线，不做 RL）：检测静止 → 电机把关节以受限速度匀速驱动到起身动画首帧姿态（2~3s 过渡，期间 strength 低）→ 播放起身动画同时 strength 渐升 → 完成后交还动画控制。起身动画本身用普通动画做，电机只负责"姿态搬运"。

---

## 6. Cocos Creator 集成（runtime 层）

### 6.1 构建流程（MRRagdollBuilder）

Cocos 没有类似 Unity Mecanim 的人形 avatar 系统，因此用**显式骨骼映射表**：

1. `MRRagdoll` 组件上配置骨骼名映射（13 体各自对应的骨骼节点路径/名称，或 `@property([Node])` 直接引用）；也内置一个 Mixamo/标准人形命名约定的默认映射。构建时用节点当前世界位姿（**必须是 bind 姿态**，建议在 T-pose/待机 bind 下点"生成"）。
2. 每体 capsule：线段 = 本体关节位置 → 子关节位置（末端体取短段），世界系转体局部系；半径 = `clamp(0.45 × 线段长, 0.03, 0.12) × scaleAdjust`（1.75m 体型基准，见 §7.1；**先确认模型 1 单位 = 1 米**，不是就整体乘缩放系数）。
3. 关节锚点 = 两骨骼连接处（子骨骼的关节位置）；`rALocal/rBLocal` 由世界点转两体局部系。
4. 限位参数查 §7.2 默认表（`SkeletonDef` 内可覆盖）。
5. 捕获 bind 数据：`q0Bind`、关节参考轴、骨骼相对刚体的局部偏移（回写用）。
6. 过滤表按 §3.6.2 默认规则生成。
7. 产出 `SkeletonDef`（纯数据），运行时由它实例化 `MRBody` 图并绑定 `SkinnedMeshRenderer.bones`。

### 6.2 时间步与确定性（关键）

Cocos **没有 FixedUpdate**，用累加器在组件里自实现固定步长：

```ts
const FIXED_DT = 1 / 60;           // 逻辑步长
update(dt: number) {
  this.acc += Math.min(dt, 0.1);   // 钳制切后台/卡顿的 dt 尖峰，防螺旋死亡
  while (this.acc >= FIXED_DT) { this.stepOnce(FIXED_DT); this.acc -= FIXED_DT; }
}
```

- 逻辑步长 1/60，内部 8 子步 → h≈2ms；若抖动明显，substeps 升到 12，**不要放大 h**。
- 求解器（core 层）内**禁止** `Math.random`、`Date.now`、`Map` 无序遍历影响模拟。数组顺序固定 → 同输入同输出（M3 有确定性测试）。
- 慢动作/倍速：调整每帧执行多少个逻辑步，不改 `h`。

### 6.3 骨骼回写（物理 → 渲染 的桥）

构建时存每根骨骼相对其刚体的局部偏移 `boneLocalPos、boneLocalRot`（bind 时刻，bone 在 body 局部系）。每帧物理后（`lateUpdate`）：

```ts
for (const b of this.bindings) {   // b: { body, bone: Node, localPos: MRVec3, localRot: MRQuat }
  const wp = bodyPoint(b.body, b.localPos);       // body.x + q·localPos（转 cc.Vec3）
  const wq = bodyQuat(b.body, b.localRot);        // q ⊗ localRot（转 cc.Quat）
  b.bone.worldRotation = wq;
  b.bone.worldPosition = wp;
}
```

- Cocos 的 `SkinnedMeshRenderer` 按 `bones` 数组节点的**世界变换**计算蒙皮矩阵，直接写 `worldRotation/worldPosition` 即可，层级结构无所谓。
- **[v1.2] 回写顺序必须父先子后（骨骼树拓扑序）**：`worldPosition/worldRotation` 的 setter
  依赖当前父节点变换反解 local 值；先写子后写父会让子的世界位姿被随后写入的父变换带动漂移。
  `def.bodies` 本身即拓扑序，按数组顺序写即可。
- 被动模式：**必须停掉该模型上的 `AnimationController/Animation`**（否则动画每帧覆盖回写）。
- 主动模式：动画照常播放（§5.3 读目标），回写放 `lateUpdate`（渲染前最后执行，天然覆盖动画写入）。若仍出现动画覆盖物理的一帧抖动，用 `@executionOrder` 调整组件先后——这是需要实测确认的点。
- 回写后无需手动刷新蒙皮：渲染阶段自动取骨骼世界矩阵。

### 6.4 性能预算

13 体 / ~50 接触对 / 8 子步 ≈ 每逻辑步 < 0.3ms（普通 TS）。10 角色以内无需 Worker/wasm。热循环里注意基本规矩：复用 `MRVec3` 临时对象、不在子步内 new 对象，但**禁止提前做对象池/SoA 内存布局这类优化**（反过度设计），实测 profile 不够再说。

---

## 7. 参数默认值

### 7.1 质量比例（占总量 %，Winter 人体测量学近似；构建时按骨架缩放归一化）

| 体 | % | 体 | % |
|---|---|---|---|
| pelvis | 15 | head | 8 |
| chest | 30 | upperArm ×2 | 3.0 |
| forearm ×2（含手） | 2.5 | thigh ×2 | 10.0 |
| shin ×2 | 4.7 | foot ×2（含） | 1.0 |

capsule 半径基准（1.75m 体型）：chest 0.12、pelvis 0.11、head 0.11、thigh 0.075、shin 0.06、upperArm 0.05、forearm 0.045。

### 7.2 关节限位默认表（度）

| 关节 | 类型 | coneMax | twist [min,max] | 说明 |
|---|---|---|---|---|
| chest←pelvis（腰） | 锥扭 | 30 | [-30, 30] | |
| head←chest（颈） | 锥扭 | 40 | [-60, 60] | |
| shoulder（肩）×2 | 锥扭 | 90 | [-45, 45] | |
| elbow（肘）×2 | 铰链 | — | [0, 150] | 弯曲方向朝前 |
| wrist（腕）×2 | 锥扭 | 30 | [-15, 15] | [v1.2] 补充默认值（13 体口径含手独立） |
| hip（髋）×2 | 锥扭 | 60 | [-30, 30] | |
| knee（膝）×2 | 铰链 | — | [0, 150] | 弯曲方向朝后，**别反** |
| ankle（踝）×2 | 锥扭 | 30 | [-15, 15] | |

铰链行程参数 `aMin/aMax` 填入 `MRHingeJoint`；参考轴方向（肘朝前/膝朝后）在 `SkeletonDef` 里配。文档角度单位是度，**实现里一律存弧度**。

### 7.3 求解与接触

| 参数 | 默认 | 说明 |
|---|---|---|
| substeps | 8 | h≤5ms；抖动则加 |
| iterations/substep | 1 | Small Steps 策略 |
| gravity | (0, −9.81, 0) | |
| linDamp / angDamp | 0.05 / 0.05 | 微阻尼 |
| μ 地面 | 0.7 | |
| μ 自碰撞 | 0.3 | 太大会肢体粘连 |
| restitution | 0 | 位置法接触天然非弹性 |
| 角修正钳制 | 0.2 rad/次 | |
| 线修正钳制 | 0.05 m/次 | |
| \|v\| / \|ω\| 钳制 | 30 m/s / 60 rad/s | NaN 防线之一 |

---

## 8. 实施阶段与验收标准（Definition of Done）

> M1~M3 的验收测试跑在 **vitest / Node**（core 层引擎无关，全部可自动化）；场景观感项在编辑器测试场景检查。每阶段完成后**必须先跑通全部验收测试**再进入下一阶段，报告时附 `npx vitest run` 结果。禁止跳阶段。

### M1 单刚体落地（L0+L1 地面部分+L2 骨架）
- 单 capsule 从 5m 自由落体到平面。
- 测试：3s 后 |v| < 0.01 且穿透 < 1e-3；静止 10s 无抖动（位移方差 < 1e-6）；全程无 NaN。
- 摩擦测试：μ=0.7 时放在 30° 斜面（tan30°≈0.577 < 0.7）不滑；μ=0.4 时滑。→ **验证摩擦公式方向**。
- M1b：盒惯性两体（竖杆倒下）翻滚自然。

### M2 关节链（L2 关节）
- 2 体球关节单摆：初始水平释放，摆动且能量单调衰减（有阻尼）；锚点距离误差 < 1mm。
- 5 体链 + 铰链关节：只绕铰链轴弯，其他自由度锁死（最大偏移 < 1°）。
- N=8 链甩动 10s 无爆炸、无 NaN。

### M3 全身被动布娃娃（L1 全部 + L2 全部）★ 核心里程碑
- 合成 13 体 SkeletonDef，T-pose 1m 落地：(a) 3s 内静止；(b) 静止时所有限位满足（超差 < 2°）；(c) 全程过滤表内 capsule 对穿透 < 1mm；(d) 60s 无 NaN。以上全部 vitest 自动断言。
- 限位符号实证测试：逐关节超限启动，一步内回到限内。
- 确定性测试：同输入跑两遍，位置序列 hash 相同。
- 性能：单角色 60s 模拟（60Hz×8 子步）挂钟时间 < 3s（粗略线，超了先 profile 再说）。
- 编辑器场景人工抽查：手臂折叠贴躯干不穿、落地姿态像人（接一个真模型看观感）。

### M4 主动布娃娃 + 受击变软（L3/L4）
- 挂走路/站立动画：直立跟随 > 30s 不倒（参数合理时），或至少实现可调稳定站立。
- 胸部施加 200 N·s 冲量：strength 衰减 → 踉跄/倒地 → 恢复期无 NaN、无限位爆炸。
- 调参报告：记录最终 kp/kd/τmax/strength 曲线。

### M5（可选）起立恢复
- 仰卧静止 → 3s 内电机搬运到起身动画首帧 → 播放起身动画 → 站立，全程限位满足。

---

## 9. 已知的坑（实现前再读一遍）

1. **限位修正的旋转方向符号**（§3.5.1 ⚠）：全项目最高频 bug 源。必须测试实证。
2. **四元数短弧**：算 dq、轴角分解时 `w<0` 取反，漏掉会偶发 ω 翻转抽搐。
3. **陀螺项不要加**（ω×Iω）：加了必炸。
4. **world invI 每子步更新**：忘了更新 = 关节行为随旋转错乱。
5. **穿透钳制**：大穿透一步解完会弹飞，靠 per-correction clamp + 子步小步长兜底。
6. **Cocos 动画与回写的顺序**：被动模式必须 stop `AnimationController`，否则动画每帧覆盖物理回写；主动模式回写放 `lateUpdate`，不对劲就用 `@executionOrder` 调（§6.3）。
7. **质量比**：chest:forearm ≈ 10:1 是正常的，别"平均分配"——躯干轻了翻滚会假。
8. **μ 自碰撞别照抄地面**：0.7 会让肢体互相"粘住"。
9. **COM 位置**：capsule 质心取线段中点即可（盒惯性对应），不要取骨骼原点。
10. **相邻体互斥不等于关掉自碰撞**：过滤表漏配对 = 手穿躯干；多配了 = 角色僵硬。M3 的抽查测试就是查这个。
11. **模型尺度**：所有参数按"1 单位 = 1 米"标定，接入模型前先量一下骨骼身高，不一致就整体缩放，别逐项调参补偿。
12. **切后台 dt 尖峰**：Cocos 页面失焦再回来 `update(dt)` 可能给个大 dt，累加器必须 clamp（§6.2），否则一帧内疯狂补步。
13. **[v1.2] 限位禁止绕锚点旋转**：绕锚点修正平移 COM，速度回读 v = Δx/h 把长杠杆修正变成速度脉冲，约束拉锯下布娃娃永不静止且子步越多爆炸越快。限位一律纯角修正（§3.5），锚点交给球关节维护。
14. **[v1.2] 位置修正改 q 后同步刷新 rot 缓存**：否则同子步内后续约束用过期旋转矩阵，角修正贡献延迟一个子步，约束失去一阶精确性（实测单接触只消除 41% 穿透）。invIworld 例外——整子步缓存（论文做法）。
15. **[v1.2] 双端接触必须分摊**：平躺 capsule 两端同时触地，各修 depth/2；否则法向冲量过冲约 2 倍，摩擦锥被膨胀，μ<tanθ 的斜面也静止（§3.6.3）。
16. **[v1.2] 摩擦 dispT 在法向求解前测量 + 仅首轮接触迭代施加**：违反任一条都会让 μ 失效（§3.6.4）。
17. **[v1.2] 接触多轮迭代让穿透收敛**：稳态穿透 = g·cosθ·h²/2（每端点），μ·depth 的静摩擦条件 tanθ ≤ μ 才与经典结果一致（§3.2）。
18. **[v1.2] 限位修正是 sin(角差) 的线性化**：单次只消除一部分，靠多子步收敛——这是参考实现的刻意选择（稳定优先），不要"修成正"（大角过冲更危险）。M3 的"1 个固定步内回到限内"验收即按此标定。
19. **[v1.2] 电机施加时机**：子步积分前（§3.2 步骤 0），效果经位置/旋转积分体现、由速度更新回读——与 §5.2 一致（v1.1 两处描述矛盾）。

---

## 10. 参考资料汇总

| 资料 | 链接 |
|---|---|
| Müller et al. 2020《Detailed Rigid Body Simulation with Extended PBD》（**主依据**） | https://matthias-research.github.io/pages/publications/PBDBodies.pdf |
| Macklin et al. 2016《XPBD》 | https://matthias-research.github.io/pages/publications/XPBD.pdf |
| Macklin et al. 2019《Small Steps in Physics Simulation》 | matthias-research.github.io/pages/publications/ 出版页 |
| Müller et al. 2006《Position Based Dynamics》 | https://matthias-research.github.io/pages/publications/posBasedDyn.pdf |
| Jakobsen 2001《Advanced Character Physics》（Verlet 布娃娃鼻祖，背景阅读） | https://github.com/krisives/advanced-character-physics |
| Ten Minute Physics 频道（22 刚体 / 25 关节课） | https://www.youtube.com/channel/UCTG_vrRdKYfrpqCv_WV4eyA |
| Ten Minute Physics 全部 demo（**JS 参考实现，与 TS 同生态**） | https://github.com/matthias-research/pages |
| Ten-Minute-Physics-Unity（C# 移植，结构参考） | https://github.com/Habrador/Ten-Minute-Physics-Unity |
| PositionBasedDynamics C++ 库（学术参考） | https://github.com/InteractiveComputerGraphics/PositionBasedDynamics |
| ashleve/ActiveRagdoll（PD 主动布娃娃参考） | https://github.com/ashleve/ActiveRagdoll |

---

*文档版本：v1.2（2026-10-10）。v1.0 为 Unity/C# 版；v1.1 目标平台改为 Cocos Creator 3.8.7 / TypeScript：core 层改为引擎无关纯 TS（float64）+ vitest 验证，集成层改为累加器固定步长 + lateUpdate 骨骼回写 + 显式骨骼映射构建，数学与验收标准不变。**v1.2 为实现期勘误**：M1~M3 全部实现并通过 vitest 验收（20/20），依据 Müller 2020 论文与 Ten Minute Physics 第 25 课参考实现（25-joints.html，MIT）修正了限位修正方式（纯角修正）、接触法线约定、capsule-ground 双端分摊、摩擦测量时机、铰链行程测量对象等规格问题（正文以 [v1.2] 标注，§9 增补坑 13~19），并补齐 IGround/实例化构建器入 core、MRSolver/BodyDef 等签名缺口。实现过程中发现本文公式与参考论文冲突时，以论文为准并在代码注释中标注出处；发现本文缺规格的细节，按"Müller 2020 论文 + M1~M5 测试能过"的原则自行补齐，并在交付说明中列出所有自行决定项。*
