# 科目三规划程序改动说明

本文档总结本轮针对比赛方动态障碍物新参数所做的代码修改、配置变化和验证结果。

## 1. 修改背景

比赛方将 10、11、12 号障碍物设置为高速周期运动障碍物。高速测试参数如下：

| 编号 | 运动方向角度 | 振幅（m） | 周期（s） | 朝向角度 | 峰值平移速度（m/s） |
|---|---:|---:|---:|---:|---:|
| 10 | 0° | 300 | 6 | 30° | 314.16 |
| 11 | 30° | 200 | 16 | 30° | 78.54 |
| 12 | 90° | 500 | 10 | 30° | 314.16 |

峰值速度按下式计算：

```text
v_peak = 2π × 振幅 / 周期
```

原程序使用 3 秒滚动预测。12 号障碍物会在约 33 秒时快速横穿机群航路，导致某架无人机的全部局部候选轨迹被淘汰，原始测试在第 165 周期报错：

```text
base single-UAV planning failed
```

## 2. 总体解决方案

修改后的规划流程分为三层：

1. 对速度极高的动态障碍物，在全局 A* 中使用完整周期扫掠包络。
2. 局部滚动规划仍使用障碍物的真实时变位置进行精确预测和安全检查。
3. 多机发生冲突时，使用局部联合 primitive 分配；终端阶段使用错峰进入和提前对准目标。

扫掠包络只影响全局引导路径，不会替代真实动态障碍物，也不会改变最终动态净空的计算口径。

## 3. 按速度自动启用扫掠包络

配置位置：

```text
code/run_example/mamp/configs/subject3_config.py
```

主要配置：

```python
ENABLE_FAST_DYNAMIC_SWEEP_GUIDES = True
FAST_DYNAMIC_SWEEP_SPEED_THRESHOLD = 2.0 * V_MAX
```

当前无人机最大速度 `V_MAX=50 m/s`，因此判定阈值为 `100 m/s`：

- 峰值速度小于 100 m/s：保持原来的动态预测方式；
- 峰值速度大于等于 100 m/s：全局 A* 将完整扫掠区域视为静态禁飞区。

对于当前高速参数：

- 10 号：314.16 m/s，启用扫掠包络；
- 11 号：78.54 m/s，不启用扫掠包络；
- 12 号：314.16 m/s，启用扫掠包络。

扫掠包络根据以下信息自动生成：

- 障碍物中心位置；
- 障碍物长度、宽度和高度；
- 障碍物自身朝向；
- 运动方向；
- 往复运动振幅。

同时，全局 A* 搜索边界会包含扫掠包络，避免障碍物运动范围超出原始搜索网格。

相关文件：

```text
code/run_example/mamp/envs/subject3_environment.py
code/run_example/run_phase6b_final_validation.py
```

## 4. 高速绕行 primitive 改进

高速扫掠包络会产生比原路径更大的绕行转角。原 primitive 从 50 m/s 最多只能降低到 40 m/s，部分拐角无法满足动力学约束。

高速模式新增减速候选：

```python
FAST_DYNAMIC_SWEEP_DELTA_SPEED_CANDIDATES = (
    -30.0, -20.0, -10.0, 0.0, 10.0, 20.0
)
```

这使无人机在必要时可以将终端候选速度降低到 20～30 m/s。普通低速配置仍使用原候选集合。

## 5. 多机 primitive 联合选择

原协调器采用按优先级顺序预约轨迹的方式。前面的 UAV 可能占用全部可行空间，导致后面的 UAV 即使自身存在大量有效 primitive，也无法通过多机安全检查。

本轮做了以下修改：

- 首先尝试每架 UAV 排名前 3 的候选；
- 第二级搜索可使用每架 UAV 最多 30 条候选；
- 联合选择优先扩大候选轨迹之间的最小距离；
- 支持最多 8 架邻近 UAV 的联合冲突簇；
- 使用冲突剪枝回溯搜索；
- 单次联合搜索最多检查约 25 万个节点，避免组合爆炸；
- 仅在普通顺序协调失败时启动联合搜索，正常周期不承担该开销。

相关文件：

```text
code/run_example/mamp/planners/swarm_coordinator.py
code/run_example/mamp/planners/local_joint_coordinator.py
code/run_example/mamp/planners/recursive_feasibility_coordinator.py
```

## 6. 终端进场改进

扫掠绕行后，无人机到达终点附近时可能仍保持较大的侧向速度。如果仍在距离目标约 60 米时才转向真实目标，会出现急转和急减速，最终造成所有终端 primitive 动力学不可行。

高速模式采用以下处理：

- 每组 8 架无人机按编号错峰进入终端阶段；
- 相邻 UAV 的放行时间间隔为 3 秒；
- 等待阶段参考速度限制为 30 m/s；
- 从距离目标 250 米处开始逐渐朝向真实目标，而不是最后 60 米才急转；
- 实际滚动执行周期仍为 0.2 秒，没有降低控制更新频率。

主要配置：

```python
FAST_SWEEP_TERMINAL_STAGGER_SECONDS = 3.0
FAST_SWEEP_TERMINAL_WAIT_REFERENCE_SPEED = 30.0
FAST_SWEEP_TERMINAL_GOAL_DIRECTION_DISTANCE = 250.0
```

相关文件：

```text
code/run_example/mamp/planners/terminal_policy.py
code/run_example/mamp/planners/receding_horizon_planner.py
code/run_example/run_phase6b_final_validation.py
```

## 7. 失败诊断增强

批量规划失败时现在会保存以下信息：

- 失败周期和任务时间；
- 失败 UAV 编号；
- 当前模式（`CRUISE` 或 `TERMINAL`）；
- 位置、速度、加速度；
- primitive 总数；
- 动力学可行、静态安全、动态安全候选数量；
- 最近静态和动态障碍物编号；
- 对应最小净空；
- K3、扩展联合搜索的冲突簇和失败原因。

相关文件：

```text
code/run_example/mamp/planners/swarm_base_batch_planner.py
code/run_example/mamp/planners/recursive_feasibility_coordinator.py
```

## 8. 目标点临时覆盖接口

`run_with_config.py` 新增了不修改原始 CSV 的目标点覆盖参数：

```text
--initial-goal UAV编号 X Y Z
--changed-goal UAV编号 X Y Z
```

参数可以重复使用，例如：

```bash
python code/run_example/run_with_config.py \
  --config-dir code/config \
  --mode fixed \
  --cycles 100 \
  --initial-goal 1 5000 50 -105 \
  --initial-goal 2 5000 50 -90
```

覆盖坐标会写入运行期临时配置，原始目标点 CSV 不会被修改。

## 9. 当前正式动态障碍物配置

文件：

```text
code/config/dynamic_motions.csv
```

当前内容为高速测试参数：

```csv
编号,运动方向角度,振幅,周期,朝向角度
10,0,300,6,30
11,30,200,16,30
12,90,500,10,30
```

## 10. 测试结果

### 10.1 高速动态障碍物样例

参数：`300/6、200/16、500/10`。

运行结果：

| 指标 | 结果 |
|---|---:|
| 成功 | True |
| 到达数量 | 24/24 |
| 完成周期 | 584 |
| 任务时间 | 116.8 s |
| 最小 UAV 间距 | 3.002 m |
| 最小静态障碍物净空 | 1.875 m |
| 最小动态障碍物净空 | 7.874 m |
| 总能耗 | 795.267 |
| 平均路径平滑性 | 0.805680 |

结果文件：

```text
code/run_example/output/trajectory_output_fast_swept_staggered.json
```

### 10.2 中低速动态障碍物样例

参数：`300/60、200/160、500/100`。

三者峰值速度均低于 100 m/s，因此没有启用静态扫掠包络。

| 指标 | 结果 |
|---|---:|
| 成功 | True |
| 到达数量 | 24/24 |
| 完成周期 | 583 |
| 任务时间 | 116.6 s |
| 最小 UAV 间距 | 3.008 m |
| 最小静态障碍物净空 | 2.355 m |
| 最小动态障碍物净空 | 103.019 m |
| 总能耗 | 617.185 |
| 平均路径平滑性 | 0.959723 |

结果文件：

```text
code/run_example/output/trajectory_output_sample_300_60_200_160_500_100.json
```

## 11. 运行方法

在项目根目录运行：

```bash
python code/run_example/run_with_config.py \
  --config-dir code/config \
  --mode competition \
  --cycles 2000 \
  --seed 0 \
  --output code/run_example/output/trajectory_output.json
```

在 Windows 下也可以进入 `code/run_example` 后运行：

```bat
python run_with_config.py --config-dir ..\config --mode competition --cycles 2000 --seed 0
```

程序在 24 架无人机全部到达后会提前结束，因此不一定执行满 2000 个周期。

## 12. 注意事项

- 坐标顺序为 `[X, Y高度, Z侧向]`。
- UAV 编号在 CSV 和命令行中从 1 开始，程序内部数组下标从 0 开始。
- 扫掠包络是安全优先的保守策略，会增加路径长度、规划时间和能量消耗。
- `FAST_DYNAMIC_SWEEP_SPEED_THRESHOLD` 可根据比赛要求调整；降低阈值会让更多动态障碍物进入全局静态包络。
- 当前正式配置是高速样例；中低速样例通过运行时覆盖完成测试，没有覆盖正式 CSV。
- 修改阈值、终端错峰时间或 primitive 候选后，应重新执行完整 `competition` 测试，而不能只做短周期冒烟测试。
