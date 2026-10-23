# 田穹 TIANQIONG · v2 数字样机（装配 / 关节 / 校核）

这是「田穹」模块化可变构暖棚的第二代数字样机工作区：**几何由关节图驱动**，不是手工摆放的静态模型。第一代（单模块 F04 R02）保留在仓库根目录 `F04_R02/`。

## 目录

```
v2/
├─ cad/                  # 从 R02 继承的已审查建模源码（W01/M02/P03）
│  ├─ model.py           # R02 生成器：单模块 STEP/GLB/视图数据
│  ├─ parameters.json    # F04 参数
│  └─ recovered_sources/ # 恢复的 W01 / M02 / P03 原始建模脚本
├─ src/
│  ├─ greenhouse.py      # 整棚装配生成：模块阵列 + 小墙 + 端墙 + BR750 + 连续膜
│  ├─ assembly.py        # 装配图与关节定义（86 组件 / 419 关节）
│  ├─ kinematics.py      # 关节空间求解（转动 / 移动 / 固定）
│  ├─ poses.py           # 姿态 → 几何 → STEP（构件到组件的映射）
│  ├─ audit.py           # 装配审计（拓扑、限位、映射、STEP 一致性）
│  ├─ verify2.py         # 口径统计、逐构件比对、BR750、承托头账目
│  └─ rail_carry.py      # 锚轨搬运：IK + 碰撞扫描
├─ out/                  # 生成物（STEP 较大，见下）
└─ studio/               # 浏览器三维视图（three.js，离线可用）
```

## 怎么跑

```bash
python -m venv .venv && . .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install cadquery==2.8.0 numpy==2.5.4 scipy==1.18.1 trimesh==5.1.1
cd v2/src

python assembly.py                     # 生成 out/assembly.json（86 组件 / 419 关节）
python audit.py                        # 装配审计，输出 out/audit_report.{json,md}
python verify2.py                      # 补充审计（逐构件比对等）
python poses.py --check                # 部署态零位校验
python poses.py --export deployed transport   # 导出两套 STEP（约 40 MB）
python rail_carry.py --seat-z 2600 --limit 150  # 锚轨搬运 IK 与碰撞扫描
```

`greenhouse.py` 会优先从 `v2/cad/` 读取 R02 资产；若该目录缺失才回退到开发机上的绝对路径，因此**克隆后无需修改路径即可运行**。

## 当前状态

| 项 | 结果 |
|---|---|
| 装配图 | 86 功能组件 / 341 刚体节点 / 419 关节（转动 91、移动 267、固定 61） |
| 实体机构 vs 方案件 | 244 physical / 175 declared |
| 部署态零位 | 变换矩阵偏离单位阵 1.11×10⁻¹⁶ |
| 部署态 STEP 与 FK | 体积相对差 4.3×10⁻¹⁴ |
| 运输态 STEP 与 FK | 体积相对差 1.7×10⁻¹⁴，部署臂区残留实体 0 |
| 逐构件比对 | 1180 个构件，1179 个与 STEP 实体质心误差 < 1 mm |
| 锚轨搬运 | 座高 2600 mm 时 0 碰撞，最小间隙 27.4 mm，末端误差 0.59 mm |

## 已知限制（不要当成已验证）

- 无碰撞连续扫掠（离散采样 40 mm），无结构强度 / 驱动选型 / 安全校核
- 关节解贴限位，姿态不唯一，需要加关节居中正则
- 棚膜卷收、嵌条层叠、材料大变形均未建模；运输态不含膜
- 继承缺陷：腕部代理球入土 73 mm、原 P03 腕部与支架代理重叠
- 小墙、端墙的拼接/拆卸动作未建模

## 协作约定

- 大文件：`out/*.step` 约 40 MB，已直接入库（未用 LFS）；重新生成后请只提交有意义的变更
- 生成物与源码要一起改：改几何参数请改 `cad/` 或 `assembly.py`，然后重跑 `assembly.py → audit.py → poses.py`
- 提交前请跑一次 `python audit.py`，确保 error / warn 为 0
- 分支：功能改动走 `feat/*`，几何基线改动请先提 issue 讨论
