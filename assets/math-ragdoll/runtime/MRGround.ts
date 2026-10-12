/**
 * MRGround —— 地面组件（runtime 层，依赖 'cc'）
 * 包装 core 的 IGround；默认 y=0 水平面，可配置为任意平面（M1 摩擦测试用斜面语义）。
 */
import { _decorator, Component } from 'cc';
import { IGround, MRGroundPlane } from '../core/MRGround';
import { MRVec3 } from '../core/MRMath';

const { ccclass, property } = _decorator;

@ccclass('MRGround')
export class MRGround extends Component implements IGround {
  @property
  /** 平面高度（水平面模式） */
  height = 0;

  @property({ visible() { return !this.useHorizontal; } })
  /** 自定义法线 x（倾斜平面模式） */
  normalX = 0;

  @property({ visible() { return !this.useHorizontal; } })
  normalY = 1;

  @property({ visible() { return !this.useHorizontal; } })
  normalZ = 0;

  @property
  /** 是否水平面（勾选后忽略法线设置，直接用 height） */
  useHorizontal = true;

  private _plane: MRGroundPlane = new MRGroundPlane(0);
  private _normalOut = new MRVec3();

  onLoad(): void {
    this.rebuild();
  }

  /** 参数变化后重建内部平面 */
  rebuild(): void {
    if (this.useHorizontal) {
      this._plane = MRGroundPlane.fromPointNormal(0, this.height, 0, 0, 1, 0);
    } else {
      this._plane = MRGroundPlane.fromPointNormal(0, this.height, 0, this.normalX, this.normalY, this.normalZ);
    }
  }

  /** 返回 pos 相对地面的有符号高度（normal 输出地面法线） */
  sampleHeight(pos: Readonly<MRVec3>, normal: MRVec3): number {
    return this._plane.sampleHeight(pos, normal);
  }

  /** 供无组件场景使用的纯接口 */
  get plane(): IGround {
    return this._plane;
  }
}
