/**
 * MRMath —— L0 数学库（纯 TypeScript，禁止 import 'cc'）
 *
 * 自定义 float64 向量 / 四元数 / 3x3 矩阵。与 Cocos 的 cc.Vec3/cc.Quat 互转在 runtime 层完成。
 * 求解器内部全程双精度；禁止 Math.random / Date.now 影响模拟（确定性要求）。
 *
 * 约定：
 * - 四元数 (x, y, z, w)，表示 body→world 的旋转（世界系左乘修正）。
 * - 矩阵行主序（row-major），m[row * 3 + col]。
 * - 轴角修正向量 Δφ：方向为旋转轴，模长为旋转角（弧度），右手系。
 */

export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;

export function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

// ---------------------------------------------------------------------------
// MRVec3
// ---------------------------------------------------------------------------

export class MRVec3 {
  x = 0;
  y = 0;
  z = 0;

  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  set(x: number, y: number, z: number): this {
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }

  copy(v: Readonly<MRVec3>): this {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
    return this;
  }

  clone(): MRVec3 {
    return new MRVec3(this.x, this.y, this.z);
  }

  add(v: Readonly<MRVec3>): this {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
    return this;
  }

  addScaled(v: Readonly<MRVec3>, s: number): this {
    this.x += v.x * s;
    this.y += v.y * s;
    this.z += v.z * s;
    return this;
  }

  sub(v: Readonly<MRVec3>): this {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
    return this;
  }

  scale(s: number): this {
    this.x *= s;
    this.y *= s;
    this.z *= s;
    return this;
  }

  negate(): this {
    this.x = -this.x;
    this.y = -this.y;
    this.z = -this.z;
    return this;
  }

  dot(v: Readonly<MRVec3>): number {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }

  /** this = a × b */
  cross(a: Readonly<MRVec3>, b: Readonly<MRVec3>): this {
    const ax = a.x, ay = a.y, az = a.z;
    const bx = b.x, by = b.y, bz = b.z;
    this.x = ay * bz - az * by;
    this.y = az * bx - ax * bz;
    this.z = ax * by - ay * bx;
    return this;
  }

  length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }

  lengthSqr(): number {
    return this.x * this.x + this.y * this.y + this.z * this.z;
  }

  /** 到点 a 的距离平方 */
  distanceSqrTo(a: Readonly<MRVec3>): number {
    const dx = this.x - a.x, dy = this.y - a.y, dz = this.z - a.z;
    return dx * dx + dy * dy + dz * dz;
  }

  /** 到点 a 的距离 */
  distanceTo(a: Readonly<MRVec3>): number {
    return Math.sqrt(this.distanceSqrTo(a));
  }

  /** 就地归一化；零向量时保持不变（调用方需自行处理退化分支） */
  normalize(): this {
    const len = this.length();
    if (len > 1e-12) {
      const inv = 1 / len;
      this.x *= inv;
      this.y *= inv;
      this.z *= inv;
    }
    return this;
  }

  isFinite(): boolean {
    return Number.isFinite(this.x) && Number.isFinite(this.y) && Number.isFinite(this.z);
  }

  static dot(a: Readonly<MRVec3>, b: Readonly<MRVec3>): number {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  static distance(a: Readonly<MRVec3>, b: Readonly<MRVec3>): number {
    const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
}

// ---------------------------------------------------------------------------
// MRQuat
// ---------------------------------------------------------------------------

export class MRQuat {
  x = 0;
  y = 0;
  z = 0;
  w = 1;

  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }

  set(x: number, y: number, z: number, w: number): this {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
    return this;
  }

  copy(q: Readonly<MRQuat>): this {
    this.x = q.x;
    this.y = q.y;
    this.z = q.z;
    this.w = q.w;
    return this;
  }

  clone(): MRQuat {
    return new MRQuat(this.x, this.y, this.z, this.w);
  }

  identity(): this {
    this.x = 0;
    this.y = 0;
    this.z = 0;
    this.w = 1;
    return this;
  }

  /** this = a ⊗ b（先施加 b 再施加 a，与 Cocos/Müller 记号一致） */
  multiply(a: Readonly<MRQuat>, b: Readonly<MRQuat>): this {
    const ax = a.x, ay = a.y, az = a.z, aw = a.w;
    const bx = b.x, by = b.y, bz = b.z, bw = b.w;
    this.x = aw * bx + ax * bw + ay * bz - az * by;
    this.y = aw * by + ay * bw + az * bx - ax * bz;
    this.z = aw * bz + az * bw + ax * by - ay * bx;
    this.w = aw * bw - ax * bx - ay * by - az * bz;
    return this;
  }

  conjugate(): this {
    this.x = -this.x;
    this.y = -this.y;
    this.z = -this.z;
    return this;
  }

  /** conj(q) 就地：this 变为自身共轭 */
  conjugateInPlace(): this {
    return this.conjugate();
  }

  dot(q: Readonly<MRQuat>): number {
    return this.x * q.x + this.y * q.y + this.z * q.z + this.w * q.w;
  }

  length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
  }

  /** 就地归一化；长度接近 0 时回退为单位四元数（数值防护） */
  normalize(): this {
    const len = this.length();
    if (len > 1e-12) {
      const inv = 1 / len;
      this.x *= inv;
      this.y *= inv;
      this.z *= inv;
      this.w *= inv;
    } else {
      this.identity();
    }
    return this;
  }

  /** 由单位轴 axis 与角 angle（弧度）构造 */
  fromAxisAngle(axis: Readonly<MRVec3>, angle: number): this {
    const half = angle * 0.5;
    const s = Math.sin(half);
    this.x = axis.x * s;
    this.y = axis.y * s;
    this.z = axis.z * s;
    this.w = Math.cos(half);
    return this;
  }

  /** 由单位四元数分解出轴角（角度 ∈ [0, 2π)，按 w ≥ 0 给出短弧） */
  toAxisAngle(outAxis: MRVec3): number {
    // 保证短弧：w < 0 时取反全部分量
    let x = this.x, y = this.y, z = this.z, w = this.w;
    if (w < 0) {
      x = -x; y = -y; z = -z; w = -w;
    }
    const len = Math.sqrt(x * x + y * y + z * z);
    if (len < 1e-12) {
      outAxis.set(1, 0, 0);
      return 0;
    }
    const inv = 1 / len;
    outAxis.set(x * inv, y * inv, z * inv);
    return 2 * Math.atan2(len, w);
  }

  isFinite(): boolean {
    return Number.isFinite(this.x) && Number.isFinite(this.y)
      && Number.isFinite(this.z) && Number.isFinite(this.w);
  }
}

/** dq = q_new · conj(q_prev) → 世界系角速度 ω（见设计文档 §3.2） */
export function quatDeltaToAngularVelocity(dq: Readonly<MRQuat>, h: number, out: MRVec3): MRVec3 {
  // 短弧：dq.w < 0 时取反所有分量
  let x = dq.x, y = dq.y, z = dq.z, w = dq.w;
  if (w < 0) {
    x = -x; y = -y; z = -z; w = -w;
  }
  const sinHalf = Math.sqrt(x * x + y * y + z * z);
  if (sinHalf < 1e-8) {
    // 小角近似：ω ≈ 2 * dq.xyz / h
    return out.set(2 * x / h, 2 * y / h, 2 * z / h);
  }
  const angle = 2 * Math.atan2(sinHalf, w); // = 2*acos(w) 的数值更稳形式
  const k = (angle / h) / sinHalf;
  return out.set(x * k, y * k, z * k);
}

// ---------------------------------------------------------------------------
// MRMat3 —— 行主序 3x3
// ---------------------------------------------------------------------------

export class MRMat3 {
  /** m[row * 3 + col] */
  readonly m = new Float64Array(9);

  identity(): this {
    const m = this.m;
    m[0] = 1; m[1] = 0; m[2] = 0;
    m[3] = 0; m[4] = 1; m[5] = 0;
    m[6] = 0; m[7] = 0; m[8] = 1;
    return this;
  }

  /** 对角阵 */
  setDiagonal(dxx: number, dyy: number, dzz: number): this {
    const m = this.m;
    m[0] = dxx; m[1] = 0; m[2] = 0;
    m[3] = 0; m[4] = dyy; m[5] = 0;
    m[6] = 0; m[7] = 0; m[8] = dzz;
    return this;
  }

  copy(o: Readonly<MRMat3>): this {
    this.m.set(o.m);
    return this;
  }

  /** 由单位四元数 q（body→world）构造旋转矩阵 R */
  fromQuat(q: Readonly<MRQuat>): this {
    const { x, y, z, w } = q;
    const x2 = x + x, y2 = y + y, z2 = z + z;
    const xx = x * x2, xy = x * y2, xz = x * z2;
    const yy = y * y2, yz = y * z2, zz = z * z2;
    const wx = w * x2, wy = w * y2, wz = w * z2;
    const m = this.m;
    m[0] = 1 - (yy + zz); m[1] = xy - wz; m[2] = xz + wy;
    m[3] = xy + wz; m[4] = 1 - (xx + zz); m[5] = yz - wx;
    m[6] = xz - wy; m[7] = yz + wx; m[8] = 1 - (xx + yy);
    return this;
  }

  /** this = A · B */
  mulMat(a: Readonly<MRMat3>, b: Readonly<MRMat3>): this {
    const am = a.m, bm = b.m, m = this.m;
    for (let row = 0; row < 3; row++) {
      const r0 = row * 3;
      const a0 = am[r0], a1 = am[r0 + 1], a2 = am[r0 + 2];
      m[r0] = a0 * bm[0] + a1 * bm[3] + a2 * bm[6];
      m[r0 + 1] = a0 * bm[1] + a1 * bm[4] + a2 * bm[7];
      m[r0 + 2] = a0 * bm[2] + a1 * bm[5] + a2 * bm[8];
    }
    return this;
  }

  /** 转置（旋转矩阵的转置 = 逆） */
  transposeInPlace(): this {
    const m = this.m;
    const t = m[1]; m[1] = m[3]; m[3] = t;
    const t2 = m[2]; m[2] = m[6]; m[6] = t2;
    const t3 = m[5]; m[5] = m[7]; m[7] = t3;
    return this;
  }

  /** out = Mᵀ（不改变自身） */
  transposeTo(out: MRMat3): MRMat3 {
    const m = this.m, o = out.m;
    o[0] = m[0]; o[1] = m[3]; o[2] = m[6];
    o[3] = m[1]; o[4] = m[4]; o[5] = m[7];
    o[6] = m[2]; o[7] = m[5]; o[8] = m[8];
    return out;
  }

  /** out = M · v */
  applyToVec3(v: Readonly<MRVec3>, out: MRVec3): MRVec3 {
    const m = this.m;
    const x = v.x, y = v.y, z = v.z;
    return out.set(
      m[0] * x + m[1] * y + m[2] * z,
      m[3] * x + m[4] * y + m[5] * z,
      m[6] * x + m[7] * y + m[8] * z,
    );
  }
}

/** 反对称叉乘矩阵 [r]×：[r]× · v = r × v */
export function skewSymmetric(r: Readonly<MRVec3>, out: MRMat3): MRMat3 {
  const m = out.m;
  m[0] = 0; m[1] = -r.z; m[2] = r.y;
  m[3] = r.z; m[4] = 0; m[5] = -r.x;
  m[6] = -r.y; m[7] = r.x; m[8] = 0;
  return out;
}

// ---------------------------------------------------------------------------
// 3x3 线性方程求解（带部分主元的高斯消元）
// ---------------------------------------------------------------------------

const solveTmp = new Float64Array(9);

/**
 * 解 M · x = b（M 为 3x3 对称半正定或一般矩阵，部分主元高斯消元）。
 * 奇异 / 近奇异时按最小对角兜底返回截断解（调用方通常已保证 M 正定）。
 */
export function solve3x3(M: Readonly<MRMat3>, b: Readonly<MRVec3>, out: MRVec3): MRVec3 {
  const a = solveTmp;
  a.set(M.m);
  const rhs = [b.x, b.y, b.z];
  const x = out;

  for (let col = 0; col < 3; col++) {
    // 选部分主元
    let pivotRow = col;
    let pivotAbs = Math.abs(a[col * 3 + col]);
    for (let row = col + 1; row < 3; row++) {
      const v = Math.abs(a[row * 3 + col]);
      if (v > pivotAbs) {
        pivotAbs = v;
        pivotRow = row;
      }
    }
    if (pivotRow !== col) {
      for (let k = 0; k < 3; k++) {
        const t = a[col * 3 + k];
        a[col * 3 + k] = a[pivotRow * 3 + k];
        a[pivotRow * 3 + k] = t;
      }
      const tb = rhs[col];
      rhs[col] = rhs[pivotRow];
      rhs[pivotRow] = tb;
    }
    if (Math.abs(a[col * 3 + col]) < 1e-14) {
      // 奇异兜底：该分量按微小主元处理（仅出现在完全自由的维度）
      a[col * 3 + col] = a[col * 3 + col] < 0 ? -1e-14 : 1e-14;
    }
    const invPivot = 1 / a[col * 3 + col];
    for (let row = col + 1; row < 3; row++) {
      const factor = a[row * 3 + col] * invPivot;
      if (factor === 0) continue;
      for (let k = col; k < 3; k++) {
        a[row * 3 + k] -= factor * a[col * 3 + k];
      }
      rhs[row] -= factor * rhs[col];
    }
  }

  // 回代
  const c2 = rhs[2] / a[8];
  const c1 = (rhs[1] - a[5] * c2) / a[4];
  const c0 = (rhs[0] - a[1] * c1 - a[2] * c2) / a[0];
  return x.set(c0, c1, c2);
}
