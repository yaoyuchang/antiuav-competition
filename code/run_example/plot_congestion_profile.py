"""绘制拥堵分布剖面图（分析报告图4）。

数据来源：对三层(30/60/90)与四层(15/40/65/90)两套分层方案各跑一次完整任务
（competition模式、种子0），统计24架机两两间距按x坐标分桶的近距离事件数。
原始统计脚本见 scratchpad/congestion_profile.py，其输出已固化在下方
MEASURED 常量中，使本脚本可独立运行、无需重跑约5分钟的仿真。

要用新数据重绘：跑一次 congestion_profile.py，把输出的分桶计数替换 MEASURED 即可。

用法：
    python plot_congestion_profile.py                      # 输出 docs/图4_拥堵分布.png
    python plot_congestion_profile.py --output other.png
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

# 分桶左边界（每桶200米）
BUCKETS = list(range(0, 5000, 200))

# 实测事件数：{方案: {阈值: [各分桶计数]}}
# 三层 = GUIDE_ALTITUDE_BANDS (30,60,90)；四层 = (15,40,65,90)
MEASURED = {
    '三层分层': {
        6.0: [567, 53, 152, 259, 308, 298, 240, 215, 200, 126, 138, 123,
              68, 40, 22, 22, 157, 128, 75, 100, 69, 41, 22, 10269, 10076],
        4.0: [370, 3, 51, 58, 80, 90, 74, 49, 54, 32, 37, 36,
              28, 11, 7, 1, 39, 22, 30, 53, 21, 4, 0, 22, 0],
    },
    '四层分层': {
        6.0: [650, 15, 122, 220, 125, 176, 234, 254, 241, 176, 139, 87,
              69, 60, 60, 63, 194, 119, 40, 24, 22, 9, 11, 10223, 10127],
        4.0: [428, 0, 39, 48, 42, 18, 54, 73, 54, 47, 5, 0,
              14, 20, 20, 20, 43, 58, 15, 24, 0, 0, 0, 17, 0],
    },
}

# 起飞区与终点区按设计就是密集编队（起点间距3.478m、终点间距5m），
# 计入会把中段真实拥堵压成一条平线，故剖面图只画中段。
MID_FLIGHT = (200, 4600)
SWITCH_ZONE = (3200, 3600)   # 实测拥堵反弹区，与切换点3500m重合

# 配色取自校验通过的分类色板前两槽（CVD ΔE 24.7，远高于8的门槛）
SERIES = {'三层分层': '#2a78d6', '四层分层': '#eb6834'}
SURFACE = '#fcfcfb'
INK_PRIMARY = '#0b0b0b'
INK_SECONDARY = '#52514e'
INK_MUTED = '#898781'
GRIDLINE = '#e1e0d9'
BASELINE = '#c3c2b7'


def _use_cjk_font():
    """挑一个系统里可用的中文字体，避免中文显示成方框。"""
    for name in ('WenQuanYi Zen Hei', 'Noto Sans CJK SC', 'SimHei',
                 'Microsoft YaHei', 'Unifont'):
        try:
            font_manager.findfont(name, fallback_to_default=False)
        except Exception:
            continue
        plt.rcParams['font.sans-serif'] = [name]
        plt.rcParams['axes.unicode_minus'] = False
        return name
    return None


def _mid_flight_slice():
    lo = BUCKETS.index(MID_FLIGHT[0])
    hi = BUCKETS.index(MID_FLIGHT[1])
    return lo, hi


def plot(output_path):
    _use_cjk_font()
    lo, hi = _mid_flight_slice()
    centers = [b + 100 for b in BUCKETS[lo:hi]]

    figure, axes = plt.subplots(2, 1, figsize=(10, 7.4), sharex=True,
                                facecolor=SURFACE)
    figure.subplots_adjust(hspace=0.2, top=0.87, bottom=0.145,
                           left=0.09, right=0.97)

    panels = [
        (axes[0], 6.0, '机间距 < 6 m'),
        (axes[1], 4.0, '机间距 < 4 m（接近 3 m 安全红线）'),
    ]

    for axis, threshold, panel_title in panels:
        axis.set_facecolor(SURFACE)
        # 切换区底纹：先画，让数据线压在上面
        axis.axvspan(SWITCH_ZONE[0], SWITCH_ZONE[1], color=GRIDLINE,
                     alpha=0.75, zorder=0, linewidth=0)
        axis.grid(axis='y', color=GRIDLINE, linewidth=0.8, zorder=1)
        axis.set_axisbelow(True)

        for label, color in SERIES.items():
            values = MEASURED[label][threshold][lo:hi]
            axis.plot(centers, values, color=color, linewidth=2.0,
                      marker='o', markersize=5, markerfacecolor=color,
                      markeredgecolor=SURFACE, markeredgewidth=1.2,
                      label=label, zorder=3, clip_on=False)

        axis.set_title(panel_title, color=INK_PRIMARY, fontsize=11,
                       loc='left', pad=8)
        axis.set_ylabel('事件数', color=INK_SECONDARY, fontsize=10)
        axis.set_ylim(bottom=0)
        axis.tick_params(colors=INK_MUTED, labelsize=9)
        for side in ('top', 'right'):
            axis.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            axis.spines[side].set_color(BASELINE)

    # 切换点标注只在上面板出现一次，避免重复噪声
    top = axes[0]
    top.axvline(3500, color=INK_MUTED, linewidth=1.2, linestyle='--', zorder=2)
    top.annotate('终点切换点 3500 m', xy=(3500, top.get_ylim()[1] * 0.92),
                 xytext=(3140, top.get_ylim()[1] * 0.92), ha='right',
                 color=INK_SECONDARY, fontsize=9.5,
                 arrowprops=dict(arrowstyle='->', color=INK_MUTED, linewidth=1))
    axes[1].axvline(3500, color=INK_MUTED, linewidth=1.2, linestyle='--',
                    zorder=2)

    axes[1].set_xlabel('沿航向距离 x (m)', color=INK_SECONDARY, fontsize=10)
    axes[1].set_xticks(list(range(400, 4601, 400)))

    # 图例放页脚，避免与标题、标注抢占面板顶部空间
    handles, labels = top.get_legend_handles_labels()
    figure.legend(handles, labels, loc='lower center', frameon=False,
                  fontsize=10, labelcolor=INK_SECONDARY, ncol=2,
                  bbox_to_anchor=(0.53, 0.012), columnspacing=2.4)

    figure.suptitle('近距离事件沿航向的分布：拥堵集中在终点切换点附近',
                    x=0.09, y=0.972, ha='left', color=INK_PRIMARY,
                    fontsize=13.5, fontweight='bold')
    figure.text(0.09, 0.928,
                '中段 x∈[200, 4600) m；起飞编队与终点停泊区按设计即密集，已排除',
                ha='left', color=INK_MUTED, fontsize=9.5)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=200, facecolor=SURFACE)
    print('已输出: {}'.format(output_path))
    return output_path


def main():
    parser = argparse.ArgumentParser(description='绘制拥堵分布剖面图')
    parser.add_argument('--output',
                        default=str(Path(__file__).resolve().parents[2] /
                                    'docs' / '图4_拥堵分布.png'))
    plot(parser.parse_args().output)


if __name__ == '__main__':
    main()
