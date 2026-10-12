/**
 * MRShape —— 碰撞形状（capsule），禁止 import 'cc'
 * 见设计文档 §3.6.1：体局部系线段端点 p1Local/p2Local + 半径 r。
 * 线段中点即 COM（局部系原点）。
 */
import { MRVec3, MRMat3 } from './MRMath';
import { MRBody } from './MRBody';

export class MRCapsule {
  body: MRBody;
  /** 局部系线段端点（相对 COM） */
  p1Local = new MRVec3();
  p2Local = new MRVec3();
  radius = 0.05;
  /** 地面接触摩擦系数 */
  groundFriction = 0.7;
  /** 自碰撞接触摩擦系数 */
  selfFriction = 0.3;
  /** 诊断名（与 body.name 一致） */
  name = '';

  constructor(body: MRBody) {
    this.body = body;
  }

  /** 世界系线段端点 p1：x + R·p1Local */
  worldP1(out: MRVec3): MRVec3 {
    return this.body.rot.applyToVec3(this.p1Local, out).add(this.body.x);
  }

  /** 世界系线段端点 p2 */
  worldP2(out: MRVec3): MRVec3 {
    return this.body.rot.applyToVec3(this.p2Local, out).add(this.body.x);
  }
}

/**
 * 实心盒惯性张量（局部系对角），半尺寸 (hx, hy, hz)、质量 m。
 * 默认用 capsule 包围盒的实心盒公式（设计文档 §3.1）。
 */
export function boxInvInertia(hx: number, hy: number, hz: number, mass: number, out: MRMat3): MRMat3 {
  return out.setDiagonal(
    3 / (mass * (hy * hy + hz * hz)),
    3 / (mass * (hx * hx + hz * hz)),
    3 / (mass * (hx * hx + hy * hy)),
  );
}
