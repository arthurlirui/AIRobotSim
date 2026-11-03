// ============ 化学引擎：试剂数据库 + 反应规则 + 现象模拟 ============

// 试剂数据库（常用教学/科研试剂）
// type: aqueous 水溶液 | solid 固体 | liquid 纯液体 | indicator 指示剂
export const REAGENTS = {
  H2O:      { name: '蒸馏水', type: 'liquid', color: 0xa8d4e6, conc: null, pH: 7.0 },
  HCl:      { name: '稀盐酸', type: 'aqueous', color: 0xbfe3ec, conc: '1 mol/L', pH: 0.5, hazard: ['腐蚀'] },
  H2SO4:    { name: '稀硫酸', type: 'aqueous', color: 0xcde9ef, conc: '1 mol/L', pH: 0.3, hazard: ['腐蚀'] },
  HNO3:     { name: '稀硝酸', type: 'aqueous', color: 0xd2ecf2, conc: '1 mol/L', pH: 0.5, hazard: ['腐蚀', '氧化性'] },
  CH3COOH:  { name: '醋酸', type: 'aqueous', color: 0xe8f2f4, conc: '1 mol/L', pH: 2.9 },
  NaOH:     { name: '氢氧化钠溶液', type: 'aqueous', color: 0xd8e8ea, conc: '1 mol/L', pH: 13.5, hazard: ['腐蚀'] },
  CaOH2:    { name: '澄清石灰水', type: 'aqueous', color: 0xe6f0f2, conc: '饱和', pH: 12.4 },
  NH3_H2O:  { name: '氨水', type: 'aqueous', color: 0xdcecef, conc: '1 mol/L', pH: 11.6, hazard: ['刺激'] },
  NaCl:     { name: '氯化钠溶液', type: 'aqueous', color: 0xdfeef2, conc: '1 mol/L', pH: 7.0 },
  Na2CO3:   { name: '碳酸钠溶液', type: 'aqueous', color: 0xdff0ea, conc: '1 mol/L', pH: 11.6 },
  NaHCO3:   { name: '碳酸氢钠溶液', type: 'aqueous', color: 0xe4f2ee, conc: '1 mol/L', pH: 8.3 },
  CuSO4:    { name: '硫酸铜溶液', type: 'aqueous', color: 0x2e86c8, conc: '0.5 mol/L', pH: 4.0, colorStrong: true },
  FeCl3:    { name: '氯化铁溶液', type: 'aqueous', color: 0xb8752a, conc: '0.5 mol/L', pH: 2.0, colorStrong: true },
  FeCl2:    { name: '氯化亚铁溶液', type: 'aqueous', color: 0x7fae8a, conc: '0.5 mol/L', pH: 4.5, colorStrong: true },
  FeSO4:    { name: '硫酸亚铁溶液', type: 'aqueous', color: 0x8fc4a0, conc: '0.5 mol/L', pH: 3.5, colorStrong: true },
  AgNO3:    { name: '硝酸银溶液', type: 'aqueous', color: 0xe4eef2, conc: '0.1 mol/L', pH: 5.5 },
  BaCl2:    { name: '氯化钡溶液', type: 'aqueous', color: 0xe0ecf0, conc: '0.5 mol/L', pH: 6.5 },
  KMnO4:    { name: '高锰酸钾溶液', type: 'aqueous', color: 0x7a1f6e, conc: '0.02 mol/L', pH: 7.0, colorStrong: true },
  K2Cr2O7:  { name: '重铬酸钾溶液', type: 'aqueous', color: 0xd88a20, conc: '0.1 mol/L', pH: 4.0, colorStrong: true, hazard: ['致癌'] },
  KI:       { name: '碘化钾溶液', type: 'aqueous', color: 0xe6f0ec, conc: '0.5 mol/L', pH: 6.5 },
  KSCN:     { name: '硫氰化钾溶液', type: 'aqueous', color: 0xeaf0f2, conc: '0.5 mol/L', pH: 6.5 },
  Pb_NO3_2: { name: '硝酸铅溶液', type: 'aqueous', color: 0xe6eef0, conc: '0.2 mol/L', pH: 4.5, hazard: ['毒性'] },
  ZnSO4:    { name: '硫酸锌溶液', type: 'aqueous', color: 0xe6f0f2, conc: '0.5 mol/L', pH: 5.0 },
  MgCl2:    { name: '氯化镁溶液', type: 'aqueous', color: 0xe6f0f2, conc: '0.5 mol/L', pH: 6.0 },
  CuCl2:    { name: '氯化铜溶液', type: 'aqueous', color: 0x2aa198, conc: '0.5 mol/L', pH: 4.0, colorStrong: true },
  Na2S:     { name: '硫化钠溶液', type: 'aqueous', color: 0xdfe8e2, conc: '0.2 mol/L', pH: 12.5, hazard: ['刺激'] },
  glucose:  { name: '葡萄糖溶液', type: 'aqueous', color: 0xeef6f4, conc: '10%', pH: 6.5 },
  starch:   { name: '淀粉溶液', type: 'aqueous', color: 0xe9eddc, conc: '1%', pH: 7.0 },
  H2O2:     { name: '过氧化氢溶液', type: 'aqueous', color: 0xe2eef2, conc: '3%', pH: 5.5, hazard: ['氧化性'] },
  phenolphthalein: { name: '酚酞指示剂', type: 'indicator', color: 0xf5f0fa, conc: '0.1% 乙醇', pH: null },
  methyl_orange:   { name: '甲基橙指示剂', type: 'indicator', color: 0xf0a860, conc: '0.1%', pH: null },
  litmus:          { name: '石蕊指示剂', type: 'indicator', color: 0x9a7fc0, conc: '0.5%', pH: null },
  // 固体
  s_CuSO4_5H2O: { name: '五水硫酸铜（胆矾）', type: 'solid', color: 0x2e86c8, crystal: true },
  s_NaOH:    { name: '氢氧化钠固体', type: 'solid', color: 0xf0f0ec, hazard: ['腐蚀'] },
  s_NaCl:    { name: '氯化钠（食盐）', type: 'solid', color: 0xf4f4f0 },
  s_Na2CO3:  { name: '碳酸钠（纯碱）', type: 'solid', color: 0xf2f4ee },
  s_CaCO3:   { name: '碳酸钙（石灰石）', type: 'solid', color: 0xe8e4d8 },
  s_Zn:      { name: '锌粒', type: 'solid', color: 0x9aa4ad, metal: true },
  s_Fe:      { name: '铁粉', type: 'solid', color: 0x6a6f75, metal: true },
  s_Cu:      { name: '铜片', type: 'solid', color: 0xb87333, metal: true },
  s_Mg:      { name: '镁条', type: 'solid', color: 0xc8ccd0, metal: true },
  s_S:       { name: '硫粉', type: 'solid', color: 0xd8cc30 },
  s_I2:      { name: '碘晶体', type: 'solid', color: 0x4a2a6a, crystal: true },
  s_KMnO4:   { name: '高锰酸钾固体', type: 'solid', color: 0x5a1550, crystal: true },
  s_CaO:     { name: '生石灰', type: 'solid', color: 0xece8e0 },
  s_NH4Cl:   { name: '氯化铵晶体', type: 'solid', color: 0xf2f2ee },
  s_MnO2:    { name: '二氧化锰（催化剂）', type: 'solid', color: 0x2a2a2a },
  s_KNO3:    { name: '硝酸钾', type: 'solid', color: 0xf2f0ec },
  alcohol:   { name: '无水乙醇', type: 'liquid', color: 0xe4f0f4, flammable: true },
  ether:     { name: '乙醚', type: 'liquid', color: 0xe8f0f2, flammable: true, hazard: ['易燃', '麻醉'] },
  acetone:   { name: '丙酮', type: 'liquid', color: 0xe8f2f4, flammable: true },
};

// 反应规则表：reactants 集合（已排序键）→ 产物与现象
// 每条: { keys: [..], result: { color?, precipitate?, gas?, pH?, tempDelta?, equation, note } }
export const REACTIONS = [
  // —— 酸碱中和 ——
  { keys: ['HCl', 'NaOH'], result: { color: 0xd8e8ea, tempDelta: 8, equation: 'HCl + NaOH → NaCl + H₂O', note: '中和放热，温度升高' } },
  { keys: ['H2SO4', 'NaOH'], result: { color: 0xd8e8ea, tempDelta: 9, equation: 'H₂SO₄ + 2NaOH → Na₂SO₄ + 2H₂O', note: '放热反应' } },
  { keys: ['CH3COOH', 'NaOH'], result: { color: 0xe4ece8, tempDelta: 3, equation: 'CH₃COOH + NaOH → CH₃COONa + H₂O', note: '生成醋酸钠，弱碱性' } },
  { keys: ['HCl', 'NH3_H2O'], result: { color: 0xdce8ea, tempDelta: 6, equation: 'HCl + NH₃·H₂O → NH₄Cl + H₂O', note: '白烟现象（浓时）' } },
  // —— 中和（指示剂颜色由 pH 计算，见 indicatorColor）——
  // —— 沉淀反应 ——
  { keys: ['CuSO4', 'NaOH'], result: { color: 0x88c0d0, precipitate: { color: 0x4a90c8, name: 'Cu(OH)₂ 蓝色絮状沉淀' }, equation: 'CuSO₄ + 2NaOH → Cu(OH)₂↓ + Na₂SO₄', note: '生成蓝色氢氧化铜沉淀' } },
  { keys: ['FeCl3', 'NaOH'], result: { color: 0xa87838, precipitate: { color: 0x8a4a18, name: 'Fe(OH)₃ 红褐色沉淀' }, equation: 'FeCl₃ + 3NaOH → Fe(OH)₃↓ + 3NaCl', note: '生成红褐色氢氧化铁沉淀' } },
  { keys: ['FeCl2', 'NaOH'], result: { color: 0x9ab890, precipitate: { color: 0x6a9a70, name: 'Fe(OH)₂ 白色→灰绿色沉淀' }, equation: 'FeCl₂ + 2NaOH → Fe(OH)₂↓ + 2NaCl', note: '白色沉淀迅速变灰绿（氧化）' } },
  { keys: ['MgCl2', 'NaOH'], result: { color: 0xd8e8ea, precipitate: { color: 0xf0f4f6, name: 'Mg(OH)₂ 白色沉淀' }, equation: 'MgCl₂ + 2NaOH → Mg(OH)₂↓ + 2NaCl', note: '生成白色氢氧化镁沉淀' } },
  { keys: ['AgNO3', 'HCl'], result: { color: 0xe0eaee, precipitate: { color: 0xf0f2f4, name: 'AgCl 白色沉淀（不溶于硝酸）' }, equation: 'AgNO₃ + HCl → AgCl↓ + HNO₃', note: '白色氯化银沉淀' } },
  { keys: ['AgNO3', 'NaCl'], result: { color: 0xe0eaee, precipitate: { color: 0xf0f2f4, name: 'AgCl 白色沉淀' }, equation: 'AgNO₃ + NaCl → AgCl↓ + NaNO₃', note: '检验氯离子的经典反应' } },
  { keys: ['BaCl2', 'H2SO4'], result: { color: 0xe0eaee, precipitate: { color: 0xf4f4f4, name: 'BaSO₄ 白色沉淀（不溶于酸）' }, equation: 'BaCl₂ + H₂SO₄ → BaSO₄↓ + 2HCl', note: '检验硫酸根离子' } },
  { keys: ['BaCl2', 'Na2CO3'], result: { color: 0xe0eee8, precipitate: { color: 0xf4f4f0, name: 'BaCO₃ 白色沉淀（溶于酸）' }, equation: 'BaCl₂ + Na₂CO₃ → BaCO₃↓ + 2NaCl', note: '白色碳酸钡沉淀' } },
  { keys: ['CuSO4', 'Na2S'], result: { color: 0x8aa8b8, precipitate: { color: 0x2a2a30, name: 'CuS 黑色沉淀' }, equation: 'CuSO₄ + Na₂S → CuS↓ + Na₂SO₄', note: '生成黑色硫化铜' } },
  { keys: ['Pb_NO3_2', 'KI'], result: { color: 0xe0e8ec, precipitate: { color: 0xd8c820, name: 'PbI₂ 黄色沉淀' }, equation: 'Pb(NO₃)₂ + 2KI → PbI₂↓ + 2KNO₃', note: '金黄色碘化铅沉淀（金色雨）' } },
  { keys: ['AgNO3', 'KI'], result: { color: 0xe0e8ec, precipitate: { color: 0xd8cca8, name: 'AgI 黄色沉淀' }, equation: 'AgNO₃ + KI → AgI↓ + KNO₃', note: '黄色碘化银沉淀' } },
  // —— 气体生成 ——
  { keys: ['HCl', 'Na2CO3'], result: { color: 0xdfeef0, gas: { name: 'CO₂', color: 0xd0e0e8, acidity: '使澄清石灰水变浑浊' }, equation: 'Na₂CO₃ + 2HCl → 2NaCl + H₂O + CO₂↑', note: '产生无色无味气体（二氧化碳）' } },
  { keys: ['HCl', 'NaHCO3'], result: { color: 0xe0f0ec, gas: { name: 'CO₂', color: 0xd0e0e8 }, equation: 'NaHCO₃ + HCl → NaCl + H₂O + CO₂↑', note: '冒气泡，产生二氧化碳' } },
  { keys: ['CaOH2', 'CO2'], result: { color: 0xe8f0ee, precipitate: { color: 0xf0f0ec, name: 'CaCO₃ 白色浑浊' }, equation: 'CO₂ + Ca(OH)₂ → CaCO₃↓ + H₂O', note: '澄清石灰水变浑浊' } },
  { keys: ['H2O2', 'MnO2_mix'], result: { color: 0xe0eef2, gas: { name: 'O₂', color: 0xe8f4f8 }, catalyst: 'MnO₂', equation: '2H₂O₂ —MnO₂→ 2H₂O + O₂↑', note: '迅速冒出大量气泡（氧气，使带火星木条复燃）' } },
  // —— 金属与酸 ——
  { keys: ['HCl', 's_Zn'], result: { color: 0xd4e6ec, gas: { name: 'H₂', color: 0xe8f2f8 }, tempDelta: 10, equation: 'Zn + 2HCl → ZnCl₂ + H₂↑', note: '锌粒表面冒气泡，产生氢气（点燃有爆鸣声）' } },
  { keys: ['H2SO4', 's_Zn'], result: { color: 0xd4e6ec, gas: { name: 'H₂', color: 0xe8f2f8 }, tempDelta: 10, equation: 'Zn + H₂SO₄ → ZnSO₄ + H₂↑', note: '生成氢气，试管发热' } },
  { keys: ['HCl', 's_Fe'], result: { color: 0xa8c8b8, gas: { name: 'H₂', color: 0xe8f2f8 }, tempDelta: 8, equation: 'Fe + 2HCl → FeCl₂ + H₂↑', note: '生成浅绿色氯化亚铁和氢气' } },
  { keys: ['HCl', 's_Mg'], result: { color: 0xd4e6ec, gas: { name: 'H₂', color: 0xe8f2f8 }, tempDelta: 15, equation: 'Mg + 2HCl → MgCl₂ + H₂↑', note: '反应剧烈，放出大量热' } },
  { keys: ['HCl', 's_Cu'], result: { color: 0xd4e6ec, equation: 'Cu + HCl ⟶ 不反应', note: '铜不与稀盐酸反应（金属活动性排在氢之后）' } },
  // —— 特征显色 ——
  { keys: ['FeCl3', 'KSCN'], result: { color: 0xb01830, colorStrong: true, equation: 'Fe³⁺ + 3SCN⁻ → Fe(SCN)₃', note: '溶液变血红色（检验 Fe³⁺）' } },
  { keys: ['FeCl3', 'KI'], result: { color: 0x8a6a3a, equation: '2FeCl₃ + 2KI → 2FeCl₂ + I₂ + 2KCl', note: '氧化还原，析出碘（遇淀粉变蓝）' } },
  { keys: ['starch', 'I2_mix'], result: { color: 0x1a2a6a, colorStrong: true, equation: 'I₂ 遇淀粉变蓝', note: '淀粉碘实验：溶液显深蓝色' } },
  { keys: ['KMnO4', 'H2O2'], result: { color: 0xd8dce4, gas: { name: 'O₂', color: 0xe8f4f8 }, equation: '2KMnO₄ + 5H₂O₂ + 3H₂SO₄ → ... ', note: '紫色褪去，产生氧气' } },
  { keys: ['KMnO4', 'Na2S'], result: { color: 0xd8dce4, precipitate: { color: 0x4a4a50, name: 'S 单质乳白色浑浊' }, equation: 'KMnO₄ 氧化 S²⁻ 析出硫', note: '紫色褪去，出现乳白色硫浑浊' } },
  { keys: ['CuSO4', 's_Fe'], result: { color: 0x9ab8a0, precipitate: { color: 0xb86a30, name: 'Cu 单质（红色）' }, equation: 'Fe + CuSO₄ → FeSO₄ + Cu↓', note: '铁表面析出红色铜，溶液蓝色变浅绿' } },
  { keys: ['CuSO4', 's_Zn'], result: { color: 0xc8d8d8, precipitate: { color: 0xb86a30, name: 'Cu 单质' }, equation: 'Zn + CuSO₄ → ZnSO₄ + Cu↓', note: '锌表面析出红色铜' } },
  { keys: ['AgNO3', 'Na2S'], result: { color: 0xd8dce4, precipitate: { color: 0x1a1a20, name: 'Ag₂S 黑色沉淀' }, equation: '2AgNO₃ + Na₂S → Ag₂S↓ + 2NaNO₃', note: '生成黑色硫化银' } },
  { keys: ['CuSO4', 'Na2CO3'], result: { color: 0x8ab8c8, precipitate: { color: 0x3a9ab8, name: 'Cu₂(OH)₂CO₃ 蓝绿色碱式碳酸铜' }, equation: '2CuSO₄ + 2Na₂CO₃ + H₂O → Cu₂(OH)₂CO₃↓ + ...', note: '蓝绿色沉淀' } },
  // —— 银镜 / 费林（简化）——
  { keys: ['CuSO4', 'NaOH', 'glucose'], result: { color: 0x88b8b0, precipitate: { color: 0xa83818, name: 'Cu₂O 砖红色沉淀' }, equation: '费林反应：2Cu(OH)₂ + C₆H₁₂O₆ → Cu₂O↓ + ...', note: '水浴加热生成砖红色氧化亚铜（检测还原糖）', requireHeat: 60 } },
  // —— 石灰石与盐酸 ——
  { keys: ['HCl', 's_CaCO3'], result: { color: 0xd4e6ec, gas: { name: 'CO₂', color: 0xd0e0e8 }, equation: 'CaCO₃ + 2HCl → CaCl₂ + H₂O + CO₂↑', note: '块状固体表面剧烈冒泡' } },
  // —— 乙酸 + 碳酸盐 ——
  { keys: ['CH3COOH', 'Na2CO3'], result: { color: 0xe0ece8, gas: { name: 'CO₂', color: 0xd0e0e8 }, equation: '2CH₃COOH + Na₂CO₃ → 2CH₃COONa + H₂O + CO₂↑', note: '证明乙酸酸性强于碳酸' } },
  // —— 氨的检验 ——
  { keys: ['NH3_H2O', 's_CaO'], result: { color: 0xe0e8ec, gas: { name: 'NH₃', color: 0xe8f0f4, odor: '刺激性氨味' }, tempDelta: 20, equation: 'NH₃·H₂O + CaO → NH₃↑ + Ca(OH)₂', note: '生石灰促使氨水放出氨气，放热' } },
  // —— 硫氰化钾 + 亚铁 ——
  { keys: ['FeSO4', 'KSCN'], result: { color: 0x9ac0a8, equation: 'Fe²⁺ + SCN⁻ 无明显现象', note: '亚铁离子与 SCN⁻ 无显色（对比 Fe³⁺）' } },
];

// 查找反应：传入试剂 key 数组（可含重复），返回匹配的反应（要求所有 keys 都存在且无多余活性试剂）
export function findReaction(reagentKeys, { heated = false } = {}) {
  const set = new Set(reagentKeys.filter(k => REAGENTS[k] && REAGENTS[k].type !== 'liquid' || k !== 'H2O'));
  const active = reagentKeys.filter(k => k !== 'H2O' && k !== 'alcohol' && k !== 'ether' && k !== 'acetone');
  if (active.length === 0) return null;
  const activeSet = new Set(active);
  for (const r of REACTIONS) {
    if (r.requireHeat && !heated) continue;
    if (r.keys.every(k => activeSet.has(k)) && active.every(k => r.keys.includes(k) || INERT_KEYS.has(k))) {
      return r;
    }
  }
  // 多组份：允许 3 元反应
  for (const r of REACTIONS) {
    if (r.keys.length !== 3 || (r.requireHeat && !heated)) continue;
    const rset = new Set(r.keys);
    if (r.keys.every(k => activeSet.has(k)) && active.every(k => rset.has(k))) return r;
  }
  return null;
}

export const INERT_KEYS = new Set(['starch_I2_products', 'phenolphthalein', 'methyl_orange', 'litmus']);

// 指示剂变色：按 pH 返回颜色，未加指示剂返回 null
export function indicatorColor(entries, pH) {
  const has = (k) => entries.some(e => e.key === k && e.volume > 0);
  if (has('phenolphthalein')) return pH >= 8.2 ? 0xe0457f : 0xf0f0f5;   // 酸无色 / 碱红
  if (has('methyl_orange')) return pH < 3.1 ? 0xe05030 : pH > 4.4 ? 0xe8a020 : 0xe08a30; // 红/黄
  if (has('litmus')) return pH < 5 ? 0xc04070 : pH > 8 ? 0x3858b0 : 0x8a6aa8; // 红/蓝/紫
  return null;
}

// 估算混合液颜色：体积加权（强色试剂权重放大）
export function mixColors(entries) {
  // entries: [{ key, volume }]
  let r = 0, g = 0, b = 0, w = 0;
  for (const { key, volume } of entries) {
    const re = REAGENTS[key];
    if (!re || volume <= 0) continue;
    const c = new (class { constructor(h) { this.r = (h >> 16 & 255) / 255; this.g = (h >> 8 & 255) / 255; this.b = (h & 255) / 255; } })(re.color);
    const weight = volume * (re.colorStrong ? 3 : 1) * (re.type === 'indicator' ? 2 : 1);
    r += c.r * weight; g += c.g * weight; b += c.b * weight; w += weight;
  }
  if (w === 0) return 0xa8d4e6;
  return (Math.round(r / w * 255) << 16) | (Math.round(g / w * 255) << 8) | Math.round(b / w * 255);
}

// 简易 pH 估算（体积加权，酸碱以 H+ / OH- 近似）
export function estimatePH(entries) {
  let acid = 0, base = 0, vol = 0;
  const concM = { HCl: 1, H2SO4: 1, HNO3: 1, CH3COOH: 0.02, NaOH: 1, CaOH2: 0.02, NH3_H2O: 0.01, Na2CO3: 0.05, NaHCO3: 0.005 };
  for (const { key, volume } of entries) {
    const re = REAGENTS[key];
    if (!re || volume <= 0) continue;
    vol += volume;
    const c = concM[key] ?? 0;
    if (['HCl', 'H2SO4', 'HNO3', 'CH3COOH'].includes(key)) acid += c * volume;
    if (['NaOH', 'CaOH2', 'NH3_H2O', 'Na2CO3', 'NaHCO3'].includes(key)) base += c * volume;
  }
  if (vol === 0) return 7;
  const net = (acid - base) / vol; // mol/L
  if (Math.abs(net) < 1e-4) return 7;
  if (net > 0) return Math.max(0, -Math.log10(net));
  return Math.min(14, 14 + Math.log10(-net));
}

// 容器内容状态
export class ContainerContent {
  constructor() {
    this.entries = [];        // [{key, volume}]  volume: mL
    this.temperature = 25;    // ℃
    this.precipitate = null;  // { color, name }
    this.precipitateAmount = 0;
    this.gasProducing = null; // { name, ... }
    this.lastReaction = null;
    this.stirred = false;
    this.dirty = false;
  }
  totalVolume() { return this.entries.reduce((s, e) => s + e.volume, 0); }
  add(key, volume) {
    const e = this.entries.find(e => e.key === key);
    if (e) e.volume += volume; else this.entries.push({ key, volume });
    this.dirty = true;
  }
  color() {
    const pH = this.pH();
    const ind = indicatorColor(this.entries, pH);
    if (ind !== null) return ind;
    if (this.lastReaction?.result?.colorStrong && this.totalVolume() > 0) return this.lastReaction.result.color;
    return mixColors(this.entries);
  }
  pH() {
    if (this.lastReaction?.result?.pH !== undefined) return this.lastReaction.result.pH;
    return estimatePH(this.entries);
  }
}
