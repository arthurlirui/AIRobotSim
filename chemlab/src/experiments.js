import * as THREE from 'three';
import { REAGENTS } from './chemistry.js';

// ============ 引导实验手册 ============
// 每个实验：目标、器材清单（自动摆放）、分步引导（含完成条件检测）

export const EXPERIMENTS = [
  {
    id: 'acid_base_titration',
    name: '酸碱中和滴定（NaOH 滴定 HCl）',
    level: '高中 · 定量实验',
    objective: '用已知浓度的 NaOH 溶液滴定未知浓度盐酸，掌握滴定操作与终点判断（酚酞变红且 30 秒不褪色）。',
    principles: 'HCl + NaOH → NaCl + H₂O；酚酞变色范围 pH 8.2~10.0。',
    setup: {
      stand: true, burette: 'NaOH', flask: 'HCl', indicator: 'phenolphthalein'
    },
    steps: [
      { text: '检查滴定管：点击「试剂架」中的 NaOH 溶液，选择填充 → 滴定管', check: (s) => s.buretteFilled },
      { text: '用移液管移取 20.00 mL 盐酸放入锥形瓶（点击试剂架 HCl → 锥形瓶）', check: (s) => s.flaskFilled },
      { text: '向锥形瓶滴入 2~3 滴酚酞指示剂', check: (s) => s.indicatorAdded },
      { text: '左手控制滴定管阀门（慢速滴加），右手不断摇动锥形瓶', check: (s) => s.dropped },
      { text: '当溶液变为红色且摇动后不褪色时，立即关闭阀门，读取体积', check: (s) => s.endPoint },
      { text: '记录数据，重复 2~3 次平行实验取平均值', check: (s) => s.endPoint && s.recorded }
    ]
  },
  {
    id: 'fe_oxyhydrogen',
    name: '铁与硫酸铜的置换反应',
    level: '初中 · 置换反应',
    objective: '观察铁与硫酸铜溶液的置换反应现象：铁表面析出红色铜，溶液由蓝色变浅绿色。',
    principles: 'Fe + CuSO₄ → FeSO₄ + Cu（金属活动性 Fe > Cu）。',
    setup: { beaker: 'CuSO4', solid: 's_Fe' },
    steps: [
      { text: '取一只烧杯，倒入适量硫酸铜溶液（蓝色）', check: (s) => s.reagentAdded },
      { text: '用镊子/药匙取铁粉加入烧杯', check: (s) => s.solidAdded },
      { text: '观察：铁表面析出红色固体，溶液颜色逐渐变浅绿', check: (s) => s.reacted },
      { text: '在观察日志中记录实验现象', check: (s) => s.logged }
    ]
  },
  {
    id: 'co2_generation',
    name: '实验室制取二氧化碳并检验',
    level: '初中 · 气体制备',
    objective: '用石灰石与稀盐酸制取 CO₂，并用澄清石灰水检验（变浑浊）。',
    principles: 'CaCO₃ + 2HCl → CaCl₂ + H₂O + CO₂↑；CO₂ + Ca(OH)₂ → CaCO₃↓ + H₂O。',
    setup: {},
    steps: [
      { text: '取试管，加入块状石灰石固体', check: (s) => s.solidAdded },
      { text: '向试管中加入稀盐酸，观察剧烈冒泡', check: (s) => s.gasProduced },
      { text: '将产生的气体通入澄清石灰水（将集气瓶装石灰水后用漏斗/导管组合，本平台简化为：向含 CO₂ 体系加入石灰水）', check: (s) => s.turbidity },
      { text: '观察到澄清石灰水变浑浊，记录现象', check: (s) => s.turbidity && s.logged }
    ]
  },
  {
    id: 'fe3_detection',
    name: 'Fe³⁺ 的检验（硫氰化钾法）',
    level: '高中 · 离子检验',
    objective: '用 KSCN 检验溶液中的 Fe³⁺：变血红色。',
    principles: 'Fe³⁺ + 3SCN⁻ → Fe(SCN)₃（血红色）。',
    setup: { beaker: 'FeCl3' },
    steps: [
      { text: '取试管，加入少量氯化铁溶液（黄色）', check: (s) => s.reagentAdded },
      { text: '滴加硫氰化钾溶液', check: (s) => s.scnAdded },
      { text: '观察溶液变为血红色 → 证明含 Fe³⁺', check: (s) => s.reacted },
      { text: '对比：向硫酸亚铁溶液中加 KSCN，无明显变化', check: (s) => s.contrastDone }
    ]
  },
  {
    id: 'cuoh2_prepare',
    name: '氢氧化铜的制备与分解',
    level: '初中·高中 · 沉淀反应',
    objective: '用 NaOH 和 CuSO₄ 制备蓝色 Cu(OH)₂ 沉淀，再加热分解为黑色 CuO。',
    principles: 'CuSO₄ + 2NaOH → Cu(OH)₂↓ + Na₂SO₄；Cu(OH)₂ —Δ→ CuO + H₂O。',
    setup: {},
    steps: [
      { text: '取试管，加入硫酸铜溶液', check: (s) => s.reagentAdded },
      { text: '滴加氢氧化钠溶液，生成蓝色絮状沉淀', check: (s) => s.precipitate },
      { text: '点燃酒精灯，加热试管（试管夹夹持，先预热）', check: (s) => s.heated },
      { text: '观察蓝色沉淀变黑（生成 CuO）', check: (s) => s.decomposed }
    ]
  },
  {
    id: 'free_explore',
    name: '自由探究：任何试剂都能试',
    level: '自由模式',
    objective: '从试剂架取用任意试剂与仪器，自由混合、加热、观察现象。',
    principles: '超过 40 种试剂、40+ 条反应规则，混合即见现象。',
    setup: {},
    steps: [
      { text: '从左侧「试剂架」点击试剂 → 选择目标容器加入', check: () => true },
      { text: '从右侧「仪器栏」添加仪器到台面', check: () => true },
      { text: '选中容器后可用「加热 / 搅拌 / 倾倒 / 检视」等操作', check: () => true }
    ]
  }
];

export function newExperimentState() {
  return {
    buretteFilled: false, flaskFilled: false, indicatorAdded: false, dropped: false,
    endPoint: false, recorded: false, reagentAdded: false, solidAdded: false, reacted: false,
    logged: false, gasProduced: false, turbidity: false, scnAdded: false, contrastDone: false,
    precipitate: false, heated: false, decomposed: false
  };
}
