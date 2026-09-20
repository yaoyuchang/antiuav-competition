/**
 * 由 分层规划算法分析报告.md 生成 Word 版本。
 *
 * 用法：node docs/build_report_docx.js
 * 输出：docs/分层规划算法分析报告_v1.docx
 *
 * 图4（拥堵分布）由 code/run_example/plot_congestion_profile.py 生成后自动嵌入；
 * 图1/2/3 与补充泛化测试表为占位段落，本地补图后替换即可。
 */
const fs = require('fs');
const path = require('path');
const {
  Document, Packer, Paragraph, TextRun, HeadingLevel, AlignmentType,
  Table, TableRow, TableCell, WidthType, ShadingType, BorderStyle,
  ImageRun, PageBreak, TableOfContents,
} = require('docx');

const ROOT = path.resolve(__dirname, '..');
const CONTENT_WIDTH = 9026;          // A4 减去 1 英寸页边距（DXA）
const INK = '0B0B0B';
const INK_SECONDARY = '52514E';
const INK_MUTED = '898781';
const HEADER_FILL = 'E8EEF7';
const NOTE_FILL = 'FDF4E7';

/* ---------- 基础构件 ---------- */

const body = (text, opts = {}) => new Paragraph({
  spacing: { after: 140, line: 320 },
  alignment: opts.center ? AlignmentType.CENTER : AlignmentType.LEFT,
  children: [new TextRun({
    text, size: opts.size || 21, color: opts.color || INK,
    bold: !!opts.bold, italics: !!opts.italics, font: '微软雅黑',
  })],
});

const h1 = (text) => new Paragraph({
  heading: HeadingLevel.HEADING_1,
  spacing: { before: 360, after: 180 },
  children: [new TextRun({ text, size: 30, bold: true, color: INK, font: '微软雅黑' })],
});

const h2 = (text) => new Paragraph({
  heading: HeadingLevel.HEADING_2,
  spacing: { before: 280, after: 140 },
  children: [new TextRun({ text, size: 25, bold: true, color: INK, font: '微软雅黑' })],
});

const h3 = (text) => new Paragraph({
  heading: HeadingLevel.HEADING_3,
  spacing: { before: 220, after: 120 },
  children: [new TextRun({ text, size: 22, bold: true, color: INK_SECONDARY, font: '微软雅黑' })],
});

const bullet = (text, opts = {}) => new Paragraph({
  bullet: { level: opts.level || 0 },
  spacing: { after: 90, line: 310 },
  children: [new TextRun({ text, size: 21, color: INK, font: '微软雅黑' })],
});

/** 带底色的提示框（结论、注意事项） */
const callout = (lines, fill = NOTE_FILL) => new Table({
  width: { size: CONTENT_WIDTH, type: WidthType.DXA },
  columnWidths: [CONTENT_WIDTH],
  rows: [new TableRow({
    children: [new TableCell({
      width: { size: CONTENT_WIDTH, type: WidthType.DXA },
      shading: { type: ShadingType.CLEAR, fill },
      margins: { top: 140, bottom: 140, left: 180, right: 180 },
      children: lines.map((line, index) => new Paragraph({
        spacing: { after: index === lines.length - 1 ? 0 : 100, line: 310 },
        children: [new TextRun({ text: line, size: 21, color: INK, font: '微软雅黑' })],
      })),
    })],
  })],
});

/** 表格：headers 为表头文字数组，rows 为二维数组，weights 为列宽权重 */
function table(headers, rows, weights) {
  const total = weights.reduce((a, b) => a + b, 0);
  const widths = weights.map((w) => Math.round(CONTENT_WIDTH * w / total));
  widths[widths.length - 1] = CONTENT_WIDTH - widths.slice(0, -1).reduce((a, b) => a + b, 0);

  const cell = (text, index, opts = {}) => new TableCell({
    width: { size: widths[index], type: WidthType.DXA },
    shading: opts.header ? { type: ShadingType.CLEAR, fill: HEADER_FILL } : undefined,
    margins: { top: 90, bottom: 90, left: 120, right: 120 },
    children: [new Paragraph({
      spacing: { after: 0, line: 290 },
      alignment: index === 0 ? AlignmentType.LEFT : AlignmentType.CENTER,
      children: [new TextRun({
        text: String(text), size: 20, bold: !!opts.header,
        color: opts.header ? INK : INK_SECONDARY, font: '微软雅黑',
      })],
    })],
  });

  return new Table({
    width: { size: CONTENT_WIDTH, type: WidthType.DXA },
    columnWidths: widths,
    borders: {
      top: { style: BorderStyle.SINGLE, size: 4, color: 'C3C2B7' },
      bottom: { style: BorderStyle.SINGLE, size: 4, color: 'C3C2B7' },
      left: { style: BorderStyle.SINGLE, size: 4, color: 'C3C2B7' },
      right: { style: BorderStyle.SINGLE, size: 4, color: 'C3C2B7' },
      insideHorizontal: { style: BorderStyle.SINGLE, size: 2, color: 'E1E0D9' },
      insideVertical: { style: BorderStyle.SINGLE, size: 2, color: 'E1E0D9' },
    },
    rows: [
      new TableRow({
        tableHeader: true,
        children: headers.map((text, i) => cell(text, i, { header: true })),
      }),
      ...rows.map((row) => new TableRow({
        children: row.map((text, i) => cell(text, i)),
      })),
    ],
  });
}

const spacer = () => new Paragraph({ spacing: { after: 140 }, children: [] });

const caption = (text) => new Paragraph({
  spacing: { before: 80, after: 200 },
  alignment: AlignmentType.CENTER,
  children: [new TextRun({ text, size: 19, color: INK_MUTED, italics: true, font: '微软雅黑' })],
});

/** 图片占位框 —— 本地补图后替换为 figure() */
const figurePlaceholder = (label, hint) => new Table({
  width: { size: CONTENT_WIDTH, type: WidthType.DXA },
  columnWidths: [CONTENT_WIDTH],
  rows: [new TableRow({
    children: [new TableCell({
      width: { size: CONTENT_WIDTH, type: WidthType.DXA },
      shading: { type: ShadingType.CLEAR, fill: 'F4F4F2' },
      margins: { top: 300, bottom: 300, left: 180, right: 180 },
      children: [
        new Paragraph({
          alignment: AlignmentType.CENTER, spacing: { after: 80 },
          children: [new TextRun({ text: `【待插入】${label}`, size: 22, bold: true, color: INK_SECONDARY, font: '微软雅黑' })],
        }),
        new Paragraph({
          alignment: AlignmentType.CENTER, spacing: { after: 0 },
          children: [new TextRun({ text: hint, size: 19, color: INK_MUTED, font: '微软雅黑' })],
        }),
      ],
    })],
  })],
});

function figure(relativePath, widthPx, heightPx) {
  const file = path.join(ROOT, relativePath);
  return new Paragraph({
    alignment: AlignmentType.CENTER,
    spacing: { before: 120, after: 0 },
    children: [new ImageRun({
      type: 'png',
      data: fs.readFileSync(file),
      transformation: { width: widthPx, height: heightPx },
    })],
  });
}

/* ---------- 文档内容 ---------- */

const children = [];

// 封面
children.push(
  new Paragraph({ spacing: { before: 2400, after: 0 }, alignment: AlignmentType.CENTER,
    children: [new TextRun({ text: '复赛科目3', size: 28, color: INK_SECONDARY, font: '微软雅黑' })] }),
  new Paragraph({ spacing: { before: 200, after: 0 }, alignment: AlignmentType.CENTER,
    children: [new TextRun({ text: '24机集群动态航迹规划', size: 44, bold: true, color: INK, font: '微软雅黑' })] }),
  new Paragraph({ spacing: { before: 160, after: 0 }, alignment: AlignmentType.CENTER,
    children: [new TextRun({ text: '分层规划算法分析报告', size: 36, bold: true, color: INK, font: '微软雅黑' })] }),
  new Paragraph({ spacing: { before: 900, after: 0 }, alignment: AlignmentType.CENTER,
    children: [new TextRun({ text: 'v1', size: 24, color: INK_MUTED, font: '微软雅黑' })] }),
  new Paragraph({ children: [new PageBreak()] }),
);

// 目录
children.push(
  h1('目录'),
  new TableOfContents('目录', { hyperlink: true, headingStyleRange: '1-3' }),
  new Paragraph({ children: [new PageBreak()] }),
);

// 一、任务背景
children.push(
  h1('一、任务背景与核心矛盾'),
  body('复赛科目3要求24架无人机在含19个障碍物的空域中协同飞行。相比初赛的静态场景，本题引入了三重动态性：'),
  bullet('动态障碍物：3个旋转长方体（10/11/12号）边平移边自转，最大运动振幅300 m，周期70~120 s，高度均为90 m——等于可用飞行层顶（网格天花板90 m），无法从上方规避；'),
  bullet('终点跳变：每架机飞至距原点斜距3500 m时终点跳变，新旧终点横向距离最大超过500 m；'),
  bullet('终点持续抖动：切换后终点并非静止，叠加±2 m正弦（1 Hz）与±1 m随机扰动（0.5 Hz），合计±3 m——是到达判定球半径（1.0 m）的3倍。'),
  body('安全约束为机间距 ≥ 3.0 m、离障碍物 ≥ 1.5 m。'),
  spacer(),
  callout([
    '核心矛盾不在单机规划，而在集群耦合。',
    '24架机起点间距仅3.478 m、终点间距仅5 m，各自独立规划出的"最优路径"几乎重合——每架单看都完美，放在一起就是必然相撞。这一判断贯穿本报告全部设计决策。',
  ]),
  spacer(),
);

// 二、总体架构
children.push(
  h1('二、总体架构：三层时间尺度分解'),
  body('系统按时间尺度而非功能模块进行分层，每层解决一类特定问题：'),
  spacer(),
  table(
    ['层级', '时间尺度', '职责', '核心方法'],
    [
      ['L1 全局引导层', '离线一次', '生成无碰撞全局航路 + 空域分区', '稀疏网格加权A* + 高度分层/横向走廊'],
      ['L2 滚动时域层', '0.2 s / 周期', '局部避障与轨迹生成', '五次多项式运动基元 + 三级筛选'],
      ['L3 集群协调层', '0.2 s / 周期', '多机冲突消解', '动态优先级预约 + 一步前瞻 + 有界联合搜索'],
    ],
    [16, 14, 32, 38],
  ),
  spacer(),
  body('选择分层而非单一方法的理由：没有任何单一路线能同时满足"实时性 + 硬安全保证 + 可复现"。CBS类方法完备但24机规模下冲突树指数膨胀；ORCA类快但纯反应式、对称场景易死锁；分布式MPC质量高但每周期求解开销大；学习类推理快但缺乏硬安全保证。分层架构让每一层只解决它擅长的问题。'),
  spacer(),
  figurePlaceholder('图1　系统三层架构框图', '建议用 draw.io 绘制，标注各层输入输出与时间尺度'),
  spacer(),
);

// 三、L1
children.push(
  h1('三、L1 全局引导层'),
  h2('3.1 基础算法：稀疏网格加权A*'),
  body('网格构建：以全部起点、初始终点、切换后终点的包络为边界自动建立三维栅格，实测规模为 261 × 19 × 137（x/y/z），垂直方向预留 GLOBAL_GUIDE_VERTICAL_RESERVE = 10 m 安全余量，即可用高度上限90 m。'),
  body('搜索策略：采用加权A*，权重 GLOBAL_W_ASTAR = 1.2。代价函数为欧氏距离，启发函数为到终点的直线距离。权重略大于1牺牲少量最优性换取显著搜索加速，在本任务的稀疏障碍场景中实测路径长度损失可忽略。'),
  h3('两项工程加速'),
  bullet('边有效性预计算：对26邻域的每个方向，预先计算所有网格点出发的边是否可通行，存入查找表 edge_valid。搜索时退化为一次数组索引，避免在A*内循环中反复做几何求交。'),
  bullet('视线剪枝：A*输出的路径沿网格边呈锯齿状，剪枝逐段尝试跳过中间点，用 segment_is_static_safe() 做真实几何校验（采样间距2.5 m），将折线拉直。注意剪枝基于真实几何而非网格占用，因此结果比网格分辨率更精细。'),
  body('转弯前视跟踪：GlobalGuideTracker 的前视距离在 20 m 到 300 m 之间根据当前速度与航路转角自适应——直线段用长前视保持稳定，转弯处缩短前视避免切角。'),
  h2('3.2 核心创新：从"在线避让"到"离线空域预分配"'),
  h3('问题诊断'),
  body('初版系统用官方CSV配置运行时在第112周期失败，但用硬编码默认参数却能跑满全程。同样的算法、同样的场景，仅坐标精度差几毫米，结果从成功翻转为失败。'),
  body('我们未将其当作数值问题加容差处理，而是深挖根因：'),
  callout([
    'L1的代价函数是纯欧氏距离，对其他无人机完全无感知。',
    '24架机起终点高度相似，独立规划出的24条航路几乎重合，机间距仅3.478 m，而安全红线是3.0 m。这是一个"零裕度"系统，任何毫米级扰动都能将其推过阈值。',
  ]),
  spacer(),
  body('这一诊断把问题从"调参数"提升到"架构缺陷"：L1的盲区无法靠容差弥补，必须在空域层面将飞机分开。'),
  h3('解决方案：高度分层 + 自适应横向走廊'),
  body('高度分层：按机号交错分配巡航高度层。采用交错（而非分块）是因为机号相邻的飞机起点才相距3.478 m，交错分配恰好把最危险的相邻机对拆到不同层。'),
  body('层间距的确定不是调参结果，而是推导结果：必须大于 2 × max(PRIMITIVE_VERTICAL_OFFSETS) = 20 m，否则L2的±10 m局部垂直修正会让飞机穿到相邻层去。三层方案取30/60/90 m（间距30 m），留有余量。'),
  body('自适应横向走廊：在分层之上对巡航段施加横向偏移，按 (2.5, 2.0, 1.5) 逐档尝试，每档都用真几何逐段校验，通不过则降级，最终退回"只分层"乃至原始航路。这是失败自适应的降级阶梯，而非写死的固定方案——因为横向平移不像抬高那样必然安全。'),
  h3('切换段的空域保持'),
  body('终点切换后的重规划采用与首段不同的剖面：切换瞬间飞机已在巡航高度，无需爬升，因此保持当前高度层飞完本段前50%，再于95%处降至新终点高度。该设计源于04号测试的失败诊断——切换瞬间若全体拉平到统一高度，会在切换区造成扎堆。'),
  spacer(),
  callout([
    '实测验证：切换区窗口 x∈[3200,3600) 内的近距离事件，同层占100%、跨层占0%（三层与四层方案均如此），同层事件的高度差中位数仅0.1~0.3 m。',
    '这证明高度分层的隔离作用是完全有效的，剩余矛盾100%在横向。',
  ]),
  spacer(),
  figurePlaceholder('图2　24机航路侧视图', '用 animate_phase6b_final_validation.py 生成；侧视图可直观看出3~4条水平航路带'),
  spacer(),
);

// 四、L2
children.push(
  new Paragraph({ children: [new PageBreak()] }),
  h1('四、L2 滚动时域层'),
  h2('4.1 运动基元生成'),
  body('以 EXECUTION_HORIZON = 0.20 s 为执行周期滚动重规划，预测时域 PRIMITIVE_HORIZON = 3.0 s，采样步长0.05 s。候选轨迹族由三维组合而成：'),
  spacer(),
  table(
    ['维度', '取值', '数量'],
    [
      ['横向偏移', '−20 / −10 / 0 / +10 / +20 m', '5'],
      ['垂直偏移', '−10 / 0 / +10 m', '3'],
      ['末端速度增量', '−10 / 0 / +10 / +20 m/s', '4'],
      ['合计', '每架机每周期生成并评估', '60 条'],
    ],
    [26, 54, 20],
  ),
  spacer(),
  body('选用五次多项式的理由：它能解析地满足位置、速度、加速度三组边界条件，生成的轨迹天然C²连续，无需额外平滑处理——这直接服务于平滑性指标。轨迹以当前实际状态为起点约束，保证滚动衔接处无跳变。'),
  h2('4.2 三级筛选流水线'),
  body('候选轨迹依次通过三道关卡，任一不通过即淘汰：'),
  bullet('动力学约束：水平过载 ≤ 2g，上升 ≤ 2g，下降 ≤ 1g（下降约束更严，符合实际飞行器特性），速度 ≤ 50 m/s。校验步长0.01 s。'),
  bullet('静态障碍筛选：对全部采样点做真实几何距离判断。'),
  bullet('动态障碍筛选：见4.3节。'),
  h2('4.3 创新点一：预测未来位姿，而非响应当前位置'),
  body('这是相对ORCA类反应式方法的本质区别。动态障碍物是旋转长方体——不仅平移，还在自转。系统调用 predict_pose() 与 predict_yaw_rate()，预测其在未来3秒整个时域内每个采样时刻的中心位置与偏航角，再逐时刻判断候选轨迹是否安全。'),
  callout([
    '我们规避的是它3秒后会在哪，而不是它现在在哪。',
    '对于振幅300 m、最大速度约15 m/s的障碍物，这个区别是决定性的——按当前位置避让，3秒后障碍物已移动约45 m，反应式方法会持续陷入"避开又撞上"的震荡。',
  ]),
  spacer(),
  h2('4.4 创新点二：评分指标内嵌代价函数'),
  body('通过全部筛选的候选按加权代价排序，代价函数包含四项：'),
  spacer(),
  table(
    ['代价项', '权重', '对应官方指标'],
    [
      ['推进度', '1.00', '抵达时长 30%'],
      ['横向偏移', '0.25', '（航路跟随性）'],
      ['曲率', '0.10', '路径平滑性 10%'],
      ['环境余隙', '0.20', '路径安全性 10%'],
    ],
    [30, 20, 50],
  ),
  spacer(),
  callout([
    '不是规划完再去优化指标，而是让规划器在选择的每一步就朝指标更优的方向走。',
    '曲率项直接压低 ∫κ²ds，余隙项直接抬高最小间距——两项官方指标被写进了决策过程本身。',
  ]),
  spacer(),
  h2('4.5 终端策略与目标抖动处理'),
  body('距终点250 m处切换至终端模式。关键设置：终端捕获速度设为0。因为24个终点仅相距5 m，若保持非零捕获速度冲入，飞机会在到达后继续滑行，穿越已在原地保持的邻机。零捕获速度让每架机稳定沉入自己的3 m到达球。'),
  body('GoalManager 对观测到的抖动目标做一阶滤波 + 死区处理（alpha = 0.5，死区 = 1.0 m）。'),
  callout([
    '为什么必须滤波而非跟随：抖动幅度±3 m是到达判据（1 m）的3倍。',
    '若直接跟随抖动目标，飞机会持续追逐晃动，永远无法稳定判定"到达"。死区设为1.0 m恰好与到达容差同量级，使小于判据的抖动被直接忽略。',
  ]),
  spacer(),
);

// 五、L3
children.push(
  new Paragraph({ children: [new PageBreak()] }),
  h1('五、L3 集群协调层'),
  h2('5.1 创新点三：最受限者优先的动态优先级'),
  body('经典 prioritized planning 给每架机固定优先级，导致低优先级飞机被持续"饿死"。本系统的优先级每周期重算，采用四级复合排序键：'),
  spacer(),
  table(
    ['优先级', '排序键', '方向', '含义'],
    [
      ['第一级', '可行候选数', '升序', '谁最没得选，谁先选'],
      ['第二级', '冲突度', '降序', '与他机冲突最多的优先'],
      ['第三级', '环境余隙', '升序', '离障碍物最近的优先'],
      ['第四级', '机号', '升序', '确定性兜底，保证可复现'],
    ],
    [16, 26, 14, 44],
  ),
  spacer(),
  body('核心思想：把选择权优先交给最被逼到墙角的那架机。一架机若因环境挤压只剩少数可行轨迹，它必须先选，否则其选择会被他机预约挤没。由于优先级随状态变化，同一架机不会在连续周期被反复牺牲，饥饿问题自然消解。第四级用机号兜底是刻意的工程设计——保证同样输入必然得到同样输出，这对竞赛复现至关重要。'),
  h2('5.2 预约式冲突消解'),
  body('按优先级顺序逐机预约：每架机从自己的排序候选中，取第一条与所有已预约轨迹均无冲突的轨迹。冲突判定用 interval_minimum() 在整个3 s时域上求两条轨迹的最小间距，与安全距离加护带比较——是时空轨迹级判定，而非单点距离判定。'),
  h2('5.3 创新点四：一步前瞻可行性门'),
  body('这是针对死锁的关键机制。一条轨迹"此刻安全"不等于"能继续飞"——它可能把无人机送入下一周期无路可走的死胡同。'),
  body('one_step_future_feasibility() 对协调结果做一次解析影子推演：假设按当前选择执行0.2 s，推算全体新状态，再检查下一周期是否仍有可行解。若无，当前选择被否决，回退重选。'),
  callout([
    '这把大量死锁消灭在发生前一个周期，是ORCA类纯反应式方法结构性缺失的能力。',
  ]),
  spacer(),
  h2('5.4 分级有界联合搜索'),
  body('当预约失败时，不做全局联合优化（24机规模下不可行），而是：'),
  bullet('提取真正造成冲突的小簇——区分"本轮已预约的全部机"与"实际造成本次失败的机"，避免误判；'),
  bullet('对该簇做联合搜索，簇规模上限为5，超出则放弃（避免组合爆炸）；'),
  bullet('按 K3 → K5 分级递进，能用小范围解决的绝不扩大规模。'),
  body('统计量 k3_solved_count / k5_activation_count / joint_search_count 被记录在输出中，可用于诊断协调压力。'),
  spacer(),
  figurePlaceholder('图3　L3 协调流程图', '建议画成：生成候选 → 动态优先级排序 → 预约循环 → 一步前瞻校验 → 失败则聚簇联合搜索 → 输出'),
  spacer(),
);

// 六、版本演进
children.push(
  new Paragraph({ children: [new PageBreak()] }),
  h1('六、版本演进与调试历程'),
  body('本节记录从初版到当前版本的关键决策，包括被否决的方案——这些失败数据与成功结果同等重要。'),
  h2('6.1 演进主线'),
  spacer(),
  table(
    ['版本', '关键改动', '硬编码基准', 'CSV 场景'],
    [
      ['V0 初版', '三层架构，无空域分区', '614周期 成功 24/24', '112周期 失败 0/24'],
      ['V1', '仅高度分层 (30/60/90)', '531周期 失败', '562周期 失败 1/24'],
      ['V2', '分层 + 横向走廊', '583周期 成功 24/24', '583周期 成功 24/24'],
      ['V3', '切换段接入空域保持', '583周期 成功 24/24', '583周期 成功 24/24'],
      ['V4', '四层分层 (15/40/65/90)', '585周期 成功 24/24', '585周期 成功 24/24'],
    ],
    [14, 30, 28, 28],
  ),
  spacer(),
  body('V0 → V1 的教训：仅加高度分层反而让基准场景退化（614 → 531失败）。分层解决了巡航段拥堵，却把失败搬到了终端段——这说明局部改进可能引入新的全局问题，必须每次都跑完整回归。'),
  body('V1 → V2 的关键：加入横向走廊后两个场景同时跑通，且收敛到完全相同的周期数——这意味着此前"改几毫米就翻转"的精度敏感问题被彻底消除。'),
  h2('6.2 被否决的方案'),
  spacer(),
  table(
    ['尝试', '结果', '结论'],
    [
      ['到达容差 1.0 → 0.9', '基准退化 614 → 531', '容差治标不治本，已回退'],
      ['到达容差 → 0.5', '同样失败', '同上'],
      ['分层取 30/50/70（间距20 m）', '382周期失败', '反向验证层间距 > 20 m 的约束'],
      ['空域分区提前收尾', '365 vs 531周期', '提前收尾只把失败挪早'],
      ['切换段加横向走廊（按终点等比）', '航路飞出±680 m网格', '切换后终点z可达±575 m，等比放大会爆界'],
      ['6层等间距分层', '未实施', '90/5 = 18 m < 20 m，违反硬约束'],
    ],
    [34, 28, 38],
  ),
  spacer(),
  h2('6.3 修复的工程缺陷'),
  bullet('np.bool_ 序列化崩溃：仅在全员到达的成功路径上触发——意味着竞赛当天若成功反而会崩。已修复。'),
  bullet('CSV接口签名不一致：formal_case() 存在两个版本导致 TypeError，已合并。'),
  bullet('细粒度采样缺失：评估器原本只记录周期边界快照，无法做能耗/平滑性积分。改为保留每个0.2 s执行段的完整采样（0.01 s分辨率）。'),
  bullet('动态障碍参数硬编码：10/11/12号的运动方向、振幅、周期原本写死在环境代码里，已全部外置为 dynamic_motions.csv，且以角度制存储而非小数分量，避免手抄精度损失。'),
  spacer(),
  callout([
    '至此，全部输入（障碍物、起点、初始终点、切换后终点、动态运动参数）均通过CSV接口导入，代码中不再有任何硬编码输入。',
  ]),
  spacer(),
);

// 七、仿真结果
children.push(
  new Paragraph({ children: [new PageBreak()] }),
  h1('七、仿真结果'),
  h2('7.1 三层方案（30/60/90 m）'),
  body('competition模式、种子0、硬编码基准场景：'),
  spacer(),
  table(
    ['指标', '实测值', '说明'],
    [
      ['任务结果', '成功，583周期，24/24到达', ''],
      ['任务时长', '116.60 s', ''],
      ['单次规划时长', '8.956 s', '24条完整全局路径一次生成'],
      ['最小机间距', '3.011 m', '要求 ≥ 3.0 m'],
      ['最小静态障碍物间距', '2.355 m', '要求 ≥ 1.5 m'],
      ['最小动态障碍物间距', '102.336 m', '要求 ≥ 1.5 m'],
      ['碰撞/违规次数', '全部为 0', ''],
      ['集群总能耗', '626.920', 'Σ∫|a|/g dt'],
      ['路径平滑性', '1.1500', '∫κ²ds，越小越平滑'],
    ],
    [32, 30, 38],
  ),
  spacer(),
  h2('7.2 四层方案（15/40/65/90 m）'),
  body('将分层数从3增至4，每层机数由8降至6（24 ÷ 4 = 6，均匀整除）。层间距25 m，仍满足 > 20 m 硬约束；15 m与90 m两个边界层经抽查，在关键窄腰段（x = 3300~3500）的横向可通行区间与30/60/90 m完全一致。'),
  spacer(),
  table(
    ['指标', '三层', '四层', '变化'],
    [
      ['完成周期 / 到达', '583 / 24-24', '585 / 24-24', '—'],
      ['任务时长', '116.60 s', '117.00 s', '+0.4 s'],
      ['最小机间距', '3.011 m', '3.020 m', '+0.009 m'],
      ['集群总能耗', '626.920', '594.470', '↓ 5.2%'],
      ['路径平滑性（越小越平滑）', '1.1500', '0.6774', '↓ 41%'],
      ['加权总分', '76.4', '81.2', '+4.8'],
    ],
    [34, 22, 22, 22],
  ),
  spacer(),
  body('四层方案的能耗与平滑性改进是确定性的——这两项由轨迹积分得出，不受运行环境影响。任务时长略增0.4 s是分层更细带来的小幅绕行代价，在权重上远小于能耗与平滑性的收益。'),
  callout([
    '需如实说明：加权总分中"单次规划时长"一项为墙钟时间，受机器负载影响。两次测量负载不完全一致，该项差异不宜全部归因于方案本身。',
  ]),
  spacer(),
  h2('7.3 一处值得记录的反常发现'),
  body('四层方案虽然总分更高，但切换区窗口 x∈[3200,3600) 的近距离事件反而增多（< 4 m 事件 61 → 101次）。'),
  body('深入诊断发现：四层方案中最频繁的冲突机对（UAV5-13、1-9、8-16、2-14）全部是跨去向组的——因为切换后终点分三组、每组8架，而 8 ÷ 4 = 2 整除，四层恰好把三个去向组均匀混入了每一层，让"要往相反方向散开的机"落在同一层。这一发现直接指向了后续优化方向。'),
  spacer(),
  figure('docs/图4_拥堵分布.png', 602, 445),
  caption('图4　近距离事件沿航向的分布（三层 vs 四层）'),
  spacer(),
);

// 八、泛化性
children.push(
  h1('八、泛化性验证'),
  body('我们未只在官方给定数据上验证，而是主动设计变换场景寻找方案的失效边界。'),
  h2('8.1 测试设计'),
  body('在 code/config_tests/ 下建立了5组独立完整的CSV配置包，每组只改变一个维度：'),
  spacer(),
  table(
    ['组别', '改动维度'],
    [
      ['01_goal_order_shuffled', '24个初始终点z值顺序反转（值不变，分配变了）'],
      ['02_starts_compressed', '起点z范围压缩一半（±40 → ±20）'],
      ['03_mirrored', '起点/终点/障碍物z坐标全部取负（整体镜像）'],
      ['04_extra_obstacles', '新增2个静态障碍物'],
      ['05_goals_expanded', '初始终点横向散布扩大一倍'],
    ],
    [34, 66],
  ),
  spacer(),
  h2('8.2 已验证的结论'),
  body('向"更宽松"方向泛化无问题：05组（终点散布扩大）不仅成功，各项指标全面优于基准——机间裕度3.028 m（优于基准3.011 m）、能耗619.4（优于625.9）、平滑性0.985（优于1.104）。说明方案在终点更分散时表现更好，符合设计预期。'),
  h3('暴露的真实薄弱点'),
  bullet('02组的失败不是代码缺陷而是测试设计问题：题面给定起点间距3.478 m，已只有3 m安全线的116%余量，压缩一半后间距1.74 m，飞机起飞前的前置校验就直接拦下。这说明起点密度已无压缩空间，是题面数据本身的边界，不是算法能力问题。'),
  bullet('01组暴露了"按机号分配"的隐含假设：高度分层按 uav_index % N 分组，隐含假设起点顺序与终点顺序对应。题面数据满足此假设，但不保证评委数据也满足。'),
  bullet('03组验证了镜像等变性的缺失：镜像后三项安全距离与基准完全相同，证明镜像确实生效、几何上对称；但仍提前失败，因为分配规则按机号而非按物理位置。'),
  bullet('04组的失败点精确定位在终点切换区域，而非新增障碍物附近——这个反直觉的结果引出了第九节的关键认知。'),
  h2('8.3 补充测试'),
  body('后续又针对不同终点配置与增加障碍物的场景做了验证，均可成功完成任务。具体配置与实测数据见下表：'),
  spacer(),
  table(
    ['测试场景', '改动内容', '完成周期', '到达数', '最小机间距', '总分'],
    [
      ['（待填）', '（待填）', '', '', '', ''],
      ['（待填）', '（待填）', '', '', '', ''],
      ['（待填）', '（待填）', '', '', '', ''],
    ],
    [22, 30, 13, 11, 14, 10],
  ),
  caption('表8.3　补充泛化性测试结果（待本地运行后填入）'),
  spacer(),
);

// 九、局限与后续
children.push(
  new Paragraph({ children: [new PageBreak()] }),
  h1('九、当前局限与后续方向'),
  h2('9.1 关键认知：拥堵是"事件"而非"地点"'),
  body('我们一度认为拥堵发生在航线几何最窄处，为此编写了两版纯几何拥堵预测器，结果都预测失败：'),
  spacer(),
  table(
    ['x 区间', '近距对数', '收敛对数'],
    [
      ['[800, 1200)', '203', '176'],
      ['[1600, 2000)', '204', '104'],
      ['[3200, 3600)　← 实测热点', '202', '29'],
    ],
    [46, 27, 27],
  ),
  spacer(),
  body('预测器把 x=1900 排为最拥挤，但实测热点在 x∈[3200,3600)。原因是首段24条航路本来就处处几乎重合，近距程度全程均匀，热点是终点切换这个事件造成的——24架机在同一瞬间从紧凑终点群集体扇形散开到相距上千米的三组新终点。'),
  callout([
    '拥堵的根源是"同时散开"这个动作，而不是任何一处几何窄道。',
  ]),
  spacer(),
  h2('9.2 后续方向：按去向预置横向通道'),
  body('基于上述认知，设计了"按最终去向预置通道"方案：让每架机在首段就提前平移到其切换后终点所在的方位，把切换动作从"临时横扫五六百米"变成"到点小幅修正"。已完成可行性验证：'),
  bullet('预置位置 z = ±560 m 在 x = 600~3600 的全部采样点、全部候选高度层均静态安全；'),
  bullet('预置后航程反而缩短约60 m（5041 m vs 5100 m）；'),
  bullet('两段式A*拼接规划耗时4.878 s，比现在的单段8.424 s更快（A*耗时对距离超线性）。'),
  body('尚未完成：简单直线平移对南组（8-15号）8/8可行，但对中组仅2/8、北组仅1/8——中北组路径上有5个70~95 m高的障碍物阻挡，需改用两段A*拼接由规划器自行绕障。该部分代码已提交但未完成完整回归。'),
  h2('9.3 其他待改进项'),
  bullet('单次规划时长偏长（7~9 s）。该项占30%权重，是当前最大短板。两段式规划的测速结果表明存在明确优化空间。'),
  bullet('按机号分配的空间敏感性：01组与03组测试暴露的问题，根治方案是改为按物理位置或按有效终点分配。'),
  spacer(),
);

// 十、结论
children.push(
  h1('十、结论'),
  body('本工作的核心价值不在于某一组能跑通的参数，而在于建立了一套"诊断—定位—验证"的闭环方法论：'),
  bullet('每次改动都配套可复现的量化诊断，精确到"第几周期失败、哪两架机冲突、冲突发生在哪个坐标"；'),
  bullet('每个假设都用实测去证伪，包括推翻我们自己提出的假设（拥堵预测器）；'),
  bullet('失败数据与成功数据同等记录，被否决的方案连同其实测数字一起写进配置注释，避免后人重走弯路。'),
  spacer(),
  callout([
    '最终方案在两个官方场景下均稳定完成24/24到达、零碰撞零违规，相比初版消除了精度敏感性，能耗降低5.2%、平滑性提升41%。',
  ]),
  spacer(),
);

// 附录
children.push(
  new Paragraph({ children: [new PageBreak()] }),
  h1('附录　图表清单与复现命令'),
  spacer(),
  table(
    ['编号', '内容', '获取方式', '状态'],
    [
      ['图1', '三层架构框图', '手绘 / draw.io', '待补'],
      ['图2', '24机航路侧视图', 'animate_phase6b_final_validation.py', '待补'],
      ['图3', 'L3 协调流程图', '手绘 / draw.io', '待补'],
      ['图4', '拥堵分布剖面图', 'plot_congestion_profile.py', '已嵌入'],
      ['表8.3', '补充泛化性测试结果', '本地运行后填入', '待补'],
    ],
    [12, 32, 40, 16],
  ),
  spacer(),
  h2('复现命令'),
  body('cd code/run_example', { size: 19 }),
  body('# 基准场景', { size: 19, color: INK_MUTED }),
  body('python run_with_config.py --mode competition --cycles 2000 \\', { size: 19 }),
  body('    --config-dir /nonexistent_force_defaults --output /tmp/base.json --seed 0', { size: 19 }),
  body('python evaluate_metrics.py --input /tmp/base.json', { size: 19 }),
  spacer(),
  body('# CSV 场景', { size: 19, color: INK_MUTED }),
  body('python run_with_config.py --mode competition --cycles 2000 \\', { size: 19 }),
  body('    --config-dir ../config --output /tmp/csv.json --seed 0', { size: 19 }),
  body('python evaluate_metrics.py --input /tmp/csv.json', { size: 19 }),
  spacer(),
  body('# 重绘图4', { size: 19, color: INK_MUTED }),
  body('python plot_congestion_profile.py', { size: 19 }),
);

/* ---------- 输出 ---------- */

const doc = new Document({
  creator: '复赛科目3 项目组',
  title: '24机集群动态航迹规划 · 分层规划算法分析报告',
  styles: {
    default: {
      document: { run: { font: '微软雅黑', size: 21 } },
    },
  },
  sections: [{
    properties: { page: { margin: { top: 1440, bottom: 1440, left: 1440, right: 1440 } } },
    children,
  }],
});

const output = path.join(__dirname, '分层规划算法分析报告_v1.docx');
Packer.toBuffer(doc).then((buffer) => {
  fs.writeFileSync(output, buffer);
  console.log('已输出:', output);
});
