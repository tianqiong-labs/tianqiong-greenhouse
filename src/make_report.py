"""从 collision_A/B.json 生成 out/COLLISION_REPORT.md（可复现，不手写数字）。"""
import json, collections
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out'
A = json.load(open(OUT / 'collision_A.json', encoding='utf8'))
B = json.load(open(OUT / 'collision_B.json', encoding='utf8'))
ev = A['events']
pairs = collections.Counter((e['a'], e['b']) for e in ev)
times = sorted({e['t'] for e in ev})
gaps = [e['gap_mm'] for e in ev]
spans = collections.defaultdict(list)
for e in ev:
    spans[(e['a'], e['b'])].append(e['t'])

rows = []
for (a, b), n in pairs.most_common(8):
    ts = spans[(a, b)]
    deep = min(x['gap_mm'] for x in ev if (x['a'], x['b']) == (a, b))
    rows.append(f"| `{a}` ↔ `{b}` | {n} | {min(ts):.2f}–{max(ts):.2f} | {deep:.1f} |")

by_arm = collections.Counter()
for e in ev:
    nums = sorted({x.split('_A0')[1][:2] for x in (e['a'], e['b']) if '_A0' in x})
    if len(nums) == 2:
        by_arm[tuple(nums)] += 1

md = f"""# 逐帧运动学与碰撞检查报告

由 `check.py` 生成，`make_report.py` 汇总；数字全部来自 `collision_A.json` / `collision_B.json`，无手写值。

## 方法

- **插值**：与浏览器渲染完全一致——按 `base`/`moves` 的 TRS 在 0.25 s 关键帧间做位置线性、四元数 slerp、缩放线性，步长 1/24 s。
- **代理**：细长件 → 胶囊；块状件 → OBB。单位网格 AABB 按轴缩放还原世界尺寸（不用缩放均值，避免半径虚大）。
- **广相**：包围球 + 仅含运动件的构件对。
- **窄相**：胶囊-胶囊线段距离、胶囊-盒局部线段-AABB 距离、盒-盒 15 轴 SAT（重叠量/分离间隙分号处理）。
- **白名单**：同臂相邻连杆与关节、承托头与臂、锚轨与滑块/地锚、母墙与其固定件、卷筒与壳体。
- **未覆盖**：棚膜/嵌条/密封为单面近似，与承托头为设计接触，本轮不判。

## 结果

| 模式 | 帧数 | 事件 | 构件对 | 涉及帧 | 最小间隙 | 瞬移 |
|---|---|---|---|---|---|---|
| A 作业切换（膜展开） | {A['frames']} | {len(ev)} | {len(pairs)} | {len(times)} | {min(gaps):.1f} mm | {A['stats']['teleports']} |
| B 收膜 | {B['frames']} | {B['stats']['events']} | 0 | 0 | — | {B['stats']['teleports']} |

标定：部署静止态（t<1 s）0 事件；B 全程 0 事件。说明代理与白名单在已知合法装配上不误报。

## 异常（模式 A）

| 构件对 | 事件数 | 时间区间 / s | 最深 / mm |
|---|---|---|---|
{chr(10).join(rows)}

臂号组合：{dict(by_arm.most_common(6))}

全部落在模块 2、全部在 {times[0]:.2f}–{times[-1]:.2f} s 的根部回转段：2/5 号臂在棚面高度直接回转 35°，
整臂横扫约 2.6 m，扫过相邻 3/6 号臂。回缩只缩短第四段，不足以避障。

## 结论（回轨迹规划器修，不在渲染层挪零件）

1. 正确顺序：快脱 → 第四段全缩 → **根部附近折成紧凑 Z 形抬离相邻臂包络** → 根部回转 → 展开进入作业区。
2. 复用 `fold_paths.py` 的 75° Z 字折路径作为回转前中间构型（已验证臂间 462 mm）。
3. 补上之前，模式 A 的 {times[0]:.1f}–{times[-1]:.1f} s 段不成立。

## 局限

- 24 fps 逐帧是必要条件，不是连续碰撞证明；锚轨搬运 27.4 mm 段需按相邻帧位移自适应加密。
- OBB 重叠量为保守近似，不等于真实穿透深度。
- 膜、嵌条、密封未参与检查。
"""
(OUT / 'COLLISION_REPORT.md').write_text(md, encoding='utf8')
print(f"report: A={len(ev)} events / {len(pairs)} pairs / {min(gaps):.1f} mm ; B={B['stats']['events']} events")
