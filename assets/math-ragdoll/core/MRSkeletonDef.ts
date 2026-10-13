/**
 * MRSkeletonDef —— 骨骼拓扑、限位参数、碰撞配对过滤表定义（纯数据，可由 JSON 构建）
 * 见设计文档 §3.6.2、§4.2。角度单位：弧度（文档表格里的度为编辑参考值）。
 */

export interface BodyDef {
  name: string;
  /** 父体名；根为 null（pelvis） */
  parent: string | null;
  /** 质量占比（构建时按总和归一化，见 §7.1） */
  massRatio: number;
  /** capsule：局部系线段端点（COM 为原点）+ 半径 */
  capsule: { p1Local: [number, number, number]; p2Local: [number, number, number]; radius: number };
  /** COM 的 bind 世界位置（实例化用；查漏补缺补充，原文档缺失） */
  bindPos: [number, number, number];
  /** 地面摩擦 / 自碰摩擦覆盖（缺省用 §7.3 默认） */
  groundFriction?: number;
  selfFriction?: number;
}

export interface JointDef {
  type: 'ball' | 'hinge';
  /** 父体名 */
  a: string;
  /** 子体名 */
  b: string;
  limit?: {
    /** 锥扭限位（ball）：关节参考轴（A/B 局部系，bind 时世界对齐；查漏补缺补充） */
    axisA?: [number, number, number];
    axisB?: [number, number, number];
    coneMax?: number;
    twistMin?: number;
    twistMax?: number;
    /** [v1.4] 球关节被动扭转阻尼时间常数（秒，关节囊黏性项，见 MRLimit.DEFAULT_TWIST_DAMP）；
     *  缺省用引擎默认，显式 0 = 关闭 */
    twistDamp?: number;
    /** [v1.5] 球关节被动扭转刚度回中时间常数（秒，关节囊弹性项，见 MRLimit.DEFAULT_TWIST_STIFFNESS）；
     *  缺省用引擎默认，显式 0 = 关闭 */
    twistStiffness?: number;
    /** [v1.8] 铰链弯曲被动阻尼时间常数（秒，关节被动矩黏性项，见 MRLimit.DEFAULT_FLEX_DAMP）；
     *  缺省用引擎默认，显式 0 = 关闭 */
    flexDamp?: number;
    /** 铰链行程（hinge） */
    aMin?: number;
    aMax?: number;
    /** 铰链轴 / 参考垂直轴（hinge，A 局部系；bind 时 B 侧同值） */
    hingeAxis?: [number, number, number];
    refAxis?: [number, number, number];
    /** 子体零位参考方向（B 局部系，bind 时与 refAxis 世界对齐；缺省同 refAxis） */
    refAxisB?: [number, number, number];
  };
}

export interface SkeletonDef {
  bodies: BodyDef[];
  joints: JointDef[];
  /** 额外排除的碰撞对（默认已排除 graph distance ≤ 1） */
  ignorePairs: [string, string][];
  /** 强制包含的碰撞对（覆盖任何排除规则） */
  forcePairs: [string, string][];
  totalMass: number;
}

/**
 * 生成自碰撞配对过滤表（§3.6.2）：
 * 默认规则 = 骨骼关节图相邻（graph distance ≤ 1）互斥，其余两两检测；
 * ignorePairs 额外排除，forcePairs 强制包含（最高优先级）。
 * 返回 capsule 索引对（按 def.bodies 顺序索引）。
 */
export function buildCollisionPairs(def: SkeletonDef): [number, number][] {
  const n = def.bodies.length;
  const index = new Map<string, number>();
  def.bodies.forEach((b, i) => index.set(b.name, i));

  // 关节图邻接
  const adjacent: boolean[][] = Array.from({ length: n }, () => new Array(n).fill(false));
  for (const j of def.joints) {
    const ia = index.get(j.a);
    const ib = index.get(j.b);
    if (ia === undefined || ib === undefined) continue;
    adjacent[ia][ib] = true;
    adjacent[ib][ia] = true;
  }

  const ignored = new Set<string>();
  for (const [a, b] of def.ignorePairs) ignored.add(pairKey(a, b));
  const forced = new Set<string>();
  for (const [a, b] of def.forcePairs) forced.add(pairKey(a, b));

  const pairs: [number, number][] = [];
  for (let i = 0; i < n; i++) {
    for (let k = i + 1; k < n; k++) {
      const nameI = def.bodies[i].name;
      const nameK = def.bodies[k].name;
      const key = pairKey(nameI, nameK);
      if (forced.has(key)) {
        pairs.push([i, k]);
        continue;
      }
      if (ignored.has(key)) continue;
      if (adjacent[i][k]) continue; // graph distance ≤ 1 互斥
      pairs.push([i, k]);
    }
  }
  return pairs;
}

function pairKey(a: string, b: string): string {
  return a < b ? `${a}|${b}` : `${b}|${a}`;
}
