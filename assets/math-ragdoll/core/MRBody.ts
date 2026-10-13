/**
 * MRBody —— XPBD 刚体状态（纯数据 + 少量派生缓存，禁止 import 'cc'）
 * 见设计文档 §3.1。COM 即 body 局部系原点。
 */
import { MRVec3, MRQuat, MRMat3 } from './MRMath';

export class MRBody {
  /** 世界系质心位置 */
  x = new MRVec3();
  /** 世界系旋转（body→world） */
  q = new MRQuat();
  /** 线速度（世界系） */
  v = new MRVec3();
  /** 角速度 ω（世界系） */
  w = new MRVec3();
  /** 逆质量；静态体为 0 */
  invMass = 0;
  /** 局部系逆惯性张量（对角） */
  invIlocal = new MRMat3();
  /** 子步开始时质心快照（速度更新与摩擦用） */
  prevX = new MRVec3();
  /** 子步开始时旋转快照 */
  prevQ = new MRQuat();
  /** 静态体（地面等）：invMass = 0 且不被积分 */
  isStatic = false;
  /** 诊断名（配对过滤表 / 回写映射用） */
  name = '';

  // ---- 每子步派生缓存（由 solver 刷新，不参与持久状态）----
  /** 世界系逆惯性张量 invI = R · invIlocal · Rᵀ */
  invIworld = new MRMat3();
  /** 子步开始时旋转矩阵快照 R(prevQ)，摩擦的 material point 起点用 */
  prevR = new MRMat3();
  /** 当前旋转矩阵 R(q) */
  rot = new MRMat3();

  /** 刷新世界系派生量（每个子步预测阶段调用一次） */
  refreshDerived(): void {
    this.rot.fromQuat(this.q);
    // invI = R · invIlocal · Rᵀ
    this.invIworld.mulMat(this.rot, this.invIlocal);
    rotTransposeTmp.copy(this.rot).transposeInPlace();
    this.invIworld.mulMat(this.invIworld, rotTransposeTmp);
  }

  /** 静态体初始化辅助：invI 清零并刷新派生缓存（rot/invIworld——静态体不参与
   *  求解器的预测阶段，若不在此刷新，rot 将保持零矩阵，任何读取 A.rot 的
   *  约束/阻尼测量都会拿到退化向量） */
  makeStatic(): this {
    this.isStatic = true;
    this.invMass = 0;
    this.invIlocal.setDiagonal(0, 0, 0);
    this.refreshDerived();
    return this;
  }
}

/**
 * 绕世界点 anchor 施加旋转 (axis 单位向量, angle 弧度)：
 * x' = anchor + Rot·(x − anchor)；q' = axisAngle(axis, angle) ⊗ q（世界系左乘）。
 * 锚点保持不动 —— 关节限位的角修正用，避免破坏已解的球关节位置约束。
 */
export function rotateBodyAboutPoint(
  body: MRBody, anchor: Readonly<MRVec3>, axis: Readonly<MRVec3>, angle: number,
): void {
  if (angle === 0 || !Number.isFinite(angle)) return;
  // Rodrigues：v' = v·cosθ + (k×v)·sinθ + k(k·v)(1−cosθ)
  const rx = body.x.x - anchor.x;
  const ry = body.x.y - anchor.y;
  const rz = body.x.z - anchor.z;
  const cos = Math.cos(angle), sin = Math.sin(angle), one = 1 - cos;
  const kdotv = axis.x * rx + axis.y * ry + axis.z * rz;
  // k × v
  const cx = axis.y * rz - axis.z * ry;
  const cy = axis.z * rx - axis.x * rz;
  const cz = axis.x * ry - axis.y * rx;
  body.x.set(
    anchor.x + rx * cos + cx * sin + axis.x * kdotv * one,
    anchor.y + ry * cos + cy * sin + axis.y * kdotv * one,
    anchor.z + rz * cos + cz * sin + axis.z * kdotv * one,
  );
  rotAboutTmp.fromAxisAngle(axis, angle);
  body.q.multiply(rotAboutTmp, body.q).normalize();
  // 同步刷新旋转矩阵缓存：同一子步内后续约束/接触求解必须看到本次角修正
  // （invIworld 仍为子步开始缓存，与 Müller 2020 一致；rot 用于位置换算需即时）
  body.rot.fromQuat(body.q);
}

const rotAboutTmp = new MRQuat();

// refreshDerived 内部复用的临时矩阵（模块级，避免子步内分配）
const rotTransposeTmp = new MRMat3();
