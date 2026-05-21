# ROVandAUV

无人艇（USV）/无人机（UAV）对抗仿真：白方依据规则进行探测、分配与锁定，黑方按预设方案推进，仿真结束后产生日志与指标，并可通过回放工具进行可视化。

## 环境要求

- Python 3.9+（3.10 / 3.11 / 3.12 亦可）
- 回放可视化需要 `matplotlib`：

```bash
python3 -m pip install matplotlib
```

## 目录结构

```
.
├─ sim_main.py                 # 仿真入口（产生日志与指标）
├─ viz_log.py                  # 日志回放可视化
├─ uav_rules_sim.py            # 规则相关脚本
├─ uavsim/                     # 仿真引擎
│   ├─ entities.py             # 实体定义（USV / UAV）
│   ├─ geometry.py             # 几何计算
│   ├─ grid.py                 # 网格划分
│   ├─ sensing.py              # 感知模型
│   ├─ control.py              # 控制逻辑
│   ├─ energy.py               # 能耗模型
│   ├─ locks.py                # 锁定逻辑
│   ├─ matching.py             # 目标分配
│   ├─ game.py                 # 博弈/对抗逻辑
│   └─ sim.py                  # 主仿真器 Simulator
├─ tests/                      # 单元测试
├─ 任务区域.json                # 任务区域配置
├─ 方案1-黑方艇数量_3.json ... 方案15-黑方艇数量_15.json   # 黑方方案
├─ logDemo_out.json            # 示例日志
└─ metrics.json                # 示例指标
```

> 注意：所有命令均从项目根目录（与 `sim_main.py` 同级）运行。

## 快速开始

### 1. 运行仿真（生成日志与指标）

最低只需提供任务区域与黑方方案，其余参数均使用默认值：

```bash
python3 sim_main.py \
  --area ./任务区域.json \
  --black-plan ./方案1-黑方艇数量_3.json \
  --stream
```

也可不带任何参数（自动使用默认文件 `任务区域.json` 与 `方案1-黑方艇数量_3.json`）：

```bash
python3 sim_main.py
```

输出文件：

- 日志：`./logDemo_out.json`
- 指标：`./metrics.json`

### 2. 回放可视化

```bash
python3 viz_log.py \
  --log ./logDemo_out.json \
  --area ./任务区域.json \
  --metrics ./metrics.json \
  --fps 200
```

- 右上 HUD：白被锁定 / 黑被锁定（当前累计 / 总计）、首次突防时间、碰撞（当前累计 / 总计）
- 左上 HUD：在场数量（白 USV / 黑 USV / UAV）

## 常用参数

`sim_main.py` 支持的部分关键参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--area` | `任务区域.json` | 任务区域配置文件 |
| `--black-plan` | `方案1-黑方艇数量_3.json` | 黑方方案 JSON |
| `--dt` | `10.0` | 仿真步长（秒） |
| `--duration` | `None` | 总仿真时长（秒）；省略则直到在场黑方为 0 自动结束 |
| `--outfile` | `./logDemo_out.json` | 日志输出路径 |
| `--metrics` | `./metrics.json` | 指标输出路径 |
| `--seed` | `42` | 随机种子 |
| `--lock-success-prob` | `0.8` | 锁定成功概率 |
| `--uav-land-soc` | `0.2` | UAV 返航电量阈值 |
| `--grid-cell-km` | `5.0` | 网格格长（km） |
| `--assign-period` | `60.0` | 分配周期（秒） |
| `--uav-cruise-speed` | `120.0` | UAV 巡航速度（m/s） |
| `--stream / --no-stream` | 开启 | 流式写出日志 |

完整参数列表可通过 `python3 sim_main.py --help` 查看。

## 测试

```bash
python3 -m pytest tests/
```

## 常见问题

- **`ImportError` 或找不到 `Simulator`**：请从项目根目录运行，并确认 `uavsim/` 下存在 `__init__.py`。
- **中文显示为方块**：`viz_log.py` 会自动选取常见中文字体；若仍乱码，请在本机安装中文字体。
- **未显示任何轨迹**：请确认所用 `logDemo_out.json` 与 `metrics.json` 由本次仿真生成。
