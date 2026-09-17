# 科目3：24架无人机三维动态规划动画

## 环境

```bash
python3 -m pip install numpy matplotlib
```

## 运行

```bash
cd /home/mc/Counter_Drone/code/run_example
python3 -u animate_phase6b_final_validation.py
```

默认使用 `competition` 模式（3500 m 独立目标切换、正弦扰动和随机误差），最多运行 2000 个规划周期，并在24架无人机全部到达后停止。

常用参数：

```bash
python3 -u animate_phase6b_final_validation.py \
  --mode switch \
  --cycles 3000 \
  --speed 10 \
  --vertical-exaggeration 8
```

保存 GIF：

```bash
python3 -u animate_phase6b_final_validation.py \
  --cycles 3000 \
  --speed 10 \
  --output subject3_complete.gif \
  --no-show
```

查看全部参数：

```bash
python3 animate_phase6b_final_validation.py --help
```

一次规划并同时生成局部跟随和完整赛道动画：

```bash
python3 -u animate_phase6b_final_validation.py \
  --mode switch --cycles 3000 --speed 10 --no-show \
  --local-output subject3_local.gif \
  --overview-output subject3_overview.gif
```
