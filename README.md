# 田穹 TIANQIONG · Mode A 数字样机（v2 候选）

模块化可变构暖棚的 Mode A（棚膜保持展开条件下的作业臂切换）运动学验证与调试数据。

## 目录
- `src/planner.py` 5 维 RRT-Connect + 自适应运动边检查
- `src/geom.py` 统一点到线段距离 + 解析单元测试 + 几何不变量
- `src/trajectory2.py` 分段余弦时间律（节点速度归零）
- `src/modeA_timeline.py` / `timeline_land.py` 六段轨迹 → 1425 帧全局 24 fps 序列 + 精确终点
- `src/scene_check.py` / `v2_scene.py` 六臂全场景检查（补位后 Y）
- `src/gap_v2.py` 膜面间隙优化实验（独立目录，不覆盖基线）
- `src/publish.py` 发布闸门（task_id / 时间戳 / 文件锁 / 沙箱隔离）
- `src/out/baseline/` v1 冻结基线；`src/out/experiments/gap_v2/` v2 候选与回归
- `studio/` three.js 渲染台与 `house_data_v2.js`

## 验证结果（scene_verify_v3.json，统一距离定义）

| 指标 | 24 fps 全段 | 240 fps 重采样 | 说明 |
|---|---|---|---|
| 膜面净空 | 27.64 mm | 27.64 mm | 杆件采样点→膜折线线段，扣 73 mm |
| 邻臂 | 368.6 mm | 368.6 mm | 五臂部署位，扣 2×49 mm |
| 卷筒壳体 | 43.39 mm | 43.39 mm | 点到盒 −73 mm |
| 锚轨 / 母墙 | 0.0 mm | 0.0 mm | 设计接触，裸距离 |
| 负间隙样本 | 0 | 0 | — |
| 不变量违反 | 0 | 0 | perp ≤ vert |

240 fps 为**真实重采样**（窗口恰覆盖全段 59.5 s），不是复用 24 fps 帧。
27.64 mm 是**单段候选值**，过 20 mm 门槛，**不等于整棚 30 mm 验收**。

## 当前状态
- v2 候选：膜面最小净空 **27.64 mm**（当前实现为**杆件离散采样点 → 膜折线线段**距离，含 73 mm 关节包络；另做过一次线段-线段交叉复核，结果相同），20 mm 严格边检查通过；未达 30 mm
- 时间轴：1425 帧，段间连续（最大相邻帧关节差 1.25°）；`assert_timeline.py` 强制断言 5 个接头 + 回归用例
- 附属件随动：42 个（承托头 24 / 闸门 12 / 磁件 6）按父刚体 `T_child(t)=T_parent(t)·T_parent(0)⁻¹·T_child(0)` 绑定，
  6 构型局部变换偏差 0.000000000 mm，导出值与独立重算 0.000000000 mm
- 浏览器一致性：10 个时刻 × 6 个动件，读节点原始 TRS 与 FK 比对——最大位置误差 **0.0245 mm**、最大旋转 **0.00105°**
- **未完成**：闸门开启运动、短片段与全长调试片

## 复现
```bash
python timeline_land.py     # 生成时间轴
python geom.py              # 距离函数单元测试 + 不变量
python verify_chain.py      # 严格边检查 + 回归用例
```
