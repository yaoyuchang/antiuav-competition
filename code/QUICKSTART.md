# 科目3 无人机路径规划 - 快速开始

## ✅ 完成的接口化改造

现在代码支持**从CSV表格加载配置**，符合比赛接口要求！

## 📁 项目结构

```
code/
├── config/                          # 配置文件（输入接口）
│   ├── obstacles.csv               # 障碍物表格（题目表1）
│   ├── start_points.csv            # 起点表格
│   ├── goal_points_initial.csv     # 初始终点表格（题目表2）
│   └── goal_points_changed.csv     # 切换后终点表格（题目表3）
│
├── run_example/
│   ├── run_with_config.py          # 新：标准化运行入口（读取CSV）
│   ├── evaluate_metrics.py         # 新：独立评估脚本
│   ├── config_loader.py            # 新：配置加载模块
│   └── mamp/                       # 原有规划器核心
│
└── output/                          # 输出接口
    ├── trajectory_output.json      # 规划结果
    └── metrics.json                # 性能指标
```

## 🚀 使用方法

### 方式1：使用配置文件（推荐，符合比赛要求）

```bash
cd code/run_example

# 运行规划（从CSV读取配置）
python run_with_config.py \
  --config-dir ../config \
  --mode competition \
  --output output/trajectory_output.json

# 计算性能指标
python evaluate_metrics.py \
  --input output/trajectory_output.json \
  --output output/metrics.json
```

### 方式2：原有方式（保持兼容）

```bash
cd code/run_example
python animate_phase6b_final_validation.py --mode competition --cycles 2000
```

## 📊 配置文件说明

所有配置文件都在 `code/config/` 目录：

1. **obstacles.csv** - 障碍物配置（题目表1）
2. **start_points.csv** - 起点坐标
3. **goal_points_initial.csv** - 初始终点（题目表2）
4. **goal_points_changed.csv** - 切换后终点（题目表3）

评委可以替换这些CSV文件来测试不同场景！

## 🎯 核心改动

1. ✅ `subject3_environment.py` - 支持从CSV加载起点/终点
2. ✅ `run_phase6b_final_validation.py` - formal_case()支持CSV路径参数
3. ✅ `run_with_config.py` - 新的标准化入口
4. ✅ `evaluate_metrics.py` - 独立的指标计算脚本
5. ✅ 保持向后兼容 - 原有代码仍然可用

详细说明请查看 [接口使用说明.md](接口使用说明.md)
