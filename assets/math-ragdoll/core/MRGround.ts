/**
 * MRGround —— 地面接口与纯数学实现（core 层，禁止 import 'cc'）
 *
 * IGround 放在 core 层（设计文档 §3.6.3 的查漏补缺调整）：vitest 需要在 Node 中
 * 直接使用平面实现做 M1 落地/摩擦验收，runtime 的 Component 版地面包装它。
 */
import { MRVec3 } from './MRMath';

export interface IGround {
  /**
   * 返回点 pos 相对地面的有符号高度（沿地面法线方向的距离，位于地面上方为正）；
   * normal 输出地面法线（世界系，朝上，单位向量）。
   */
  sampleHeight(pos: Readonly<MRVec3>, normal: MRVec3): number;
}

/**
 * 一般平面实现：过点 point、单位法线 normal（朝上）的半无限平面。
 * 默认构造 y = 0 水平面；30° 斜面等测试场景直接给 point/normal。
 */
export class MRGroundPlane implements IGround {
  point = new MRVec3(0, 0, 0);
  normal = new MRVec3(0, 1, 0);

  constructor(pointY = 0) {
    this.point.set(0, pointY, 0);
    this.normal.set(0, 1, 0);
  }

  /** 由平面内任意一点与法线（会自动归一化）构造 */
  static fromPointNormal(px: number, py: number, pz: number, nx: number, ny: number, nz: number): MRGroundPlane {
    const g = new MRGroundPlane(0);
    g.point.set(px, py, pz);
    g.normal.set(nx, ny, nz).normalize();
    return g;
  }

  sampleHeight(pos: Readonly<MRVec3>, normal: MRVec3): number {
    normal.copy(this.normal);
    const dx = pos.x - this.point.x;
    const dy = pos.y - this.point.y;
    const dz = pos.z - this.point.z;
    return dx * this.normal.x + dy * this.normal.y + dz * this.normal.z;
  }
}

/** 高度场接口（预留，§3.6.3）：由采样高度 + 中心差分求法线的简易实现基类 */
export abstract class MRHeightFieldBase implements IGround {
  abstract sampleHeightOnly(x: number, z: number): number;

  sampleHeight(pos: Readonly<MRVec3>, normal: MRVec3): number {
    const e = 0.05;
    const hL = this.sampleHeightOnly(pos.x - e, pos.z);
    const hR = this.sampleHeightOnly(pos.x + e, pos.z);
    const hD = this.sampleHeightOnly(pos.x, pos.z - e);
    const hU = this.sampleHeightOnly(pos.x, pos.z + e);
    // 法线 = normalize((-dh/dx, 1, -dh/dz))
    normal.set(-(hR - hL) / (2 * e), 1, -(hU - hD) / (2 * e)).normalize();
    return this.sampleHeightOnly(pos.x, pos.z) - pos.y;
  }
}
