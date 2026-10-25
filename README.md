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

## 当前状态
- v2 候选：膜面最小净空 **27.64 mm**（线段-线段，含 73 mm 关节包络），20 mm 严格边检查通过；未达 30 mm
- 时间轴：1425 帧，段间连续（最大相邻帧关节差 1.25°）
- **未完成**：承托头/闸门/磁件按父刚体绑定、浏览器 matrixWorld 校验、调试片

## 复现
```bash
python timeline_land.py     # 生成时间轴
python geom.py              # 距离函数单元测试 + 不变量
python verify_chain.py      # 严格边检查 + 回归用例
```
