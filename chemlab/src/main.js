import * as THREE from 'three';
import { ChemLabEngine } from './engine.js';
import { InteractionManager } from './interaction.js';
import { REAGENTS, ContainerContent } from './chemistry.js';
import { APPARATUS_FACTORIES, setContainerLiquid } from './apparatus.js';
import {
  buildRoom, buildCentralBench, buildSideBench, buildFumeHood, buildVentilation,
  buildReagentCabinet, buildHazmatCabinet, buildWashStation, buildEmergencyShower,
  buildFireExtinguisher, buildFirstAidKit, buildGasCylinder, buildSafetyBoard
} from './lab.js';
import { EXPERIMENTS, newExperimentState } from './experiments.js';

// ============ 主程序：场景布置 + UI 绑定 ============

// HMR 防重复初始化：热更新时整个场景由 Vite 重新加载页面
if (window.__chemlab) {
  window.location.reload();
}

const viewport = document.getElementById('viewport');
const engine = new ChemLabEngine(viewport);
engine._liquidSetter = setContainerLiquid;
const inter = new InteractionManager(engine);
engine.interaction = inter;

// ---------- 房间与固定设施布置（依据调研的实验室设计要点） ----------
buildRoom(engine.scene);
// 中央实验台（岛式）位于房间中心
const centralBench = buildCentralBench();
centralBench.position.set(0, 0, 0.5);
engine.scene.add(centralBench);
engine.register(centralBench);
// 仪器边台（靠后墙，放大型仪器）
const sideBench = buildSideBench();
sideBench.position.set(0, 0, -5.2);
engine.scene.add(sideBench);
engine.register(sideBench);
// 通风橱（左后角）
const hood = buildFumeHood();
hood.position.set(-6.0, 0, -4.8);
engine.scene.add(hood);
engine.register(hood);
// 通风系统管道与排风罩
const vent = buildVentilation();
engine.scene.add(vent.group);
// 试剂柜（右后）
const reagentCabinet = buildReagentCabinet();
reagentCabinet.position.set(4.6, 0, -5.5);
engine.scene.add(reagentCabinet);
engine.register(reagentCabinet);
// 危化品柜（右后角落）
const hazmat = buildHazmatCabinet();
hazmat.position.set(7.0, 0, -5.5);
engine.scene.add(hazmat);
engine.register(hazmat);
// 洗涤区（右墙）
const wash = buildWashStation();
wash.position.set(6.4, 0, 1.8);
wash.rotation.y = -Math.PI / 2;
engine.scene.add(wash);
engine.register(wash);
// 应急喷淋+洗眼器（右侧，靠近门）
const shower = buildEmergencyShower();
shower.position.set(5.2, 0, 4.6);
engine.scene.add(shower);
engine.register(shower);
// 灭火器 ×2
for (const [x, z] of [[7.2, 4.6], [-7.2, -5.6]]) {
  const fx = buildFireExtinguisher();
  fx.position.set(x, 0, z);
  engine.scene.add(fx);
  engine.register(fx);
}
// 急救箱（挂在右墙）
const aid = buildFirstAidKit();
aid.position.set(7.4, 1.5, 1.0);
engine.scene.add(aid);
engine.register(aid);
// 气瓶（氮气黑、氧气蓝）
const n2 = buildGasCylinder(0x2a2a2a, '氮气钢瓶（N₂）');
n2.position.set(-7.0, 0, -2.5);
engine.scene.add(n2); engine.register(n2);
const o2 = buildGasCylinder(0x2a5aa8, '氧气钢瓶（O₂）');
o2.position.set(-6.3, 0, -2.5);
engine.scene.add(o2); engine.register(o2);
// 安全守则板（后墙）
const board = buildSafetyBoard();
board.position.set(-3.6, 0, -5.85);
engine.scene.add(board);
engine.register(board);

// ---------- 台面预置仪器 ----------
const BENCH_Y = 0.95;
function placeOnCentralBench(mesh, x, z, label) {
  mesh.position.set(x, BENCH_Y, z);
  return engine.register(mesh);
}

// 滴定管液体：细长液体柱（沿用 refreshLiquid 通道）
(function () {
  const oldSetter = engine._liquidSetter;
  engine._liquidSetter = (mesh, vol, color) => {
    const ud = mesh.userData;
    if (ud.kind === 'burette') {
      const ratio = Math.min(1, vol / ud.capacity);
      const H = 1.3, R = ud.buretteInnerR;
      if (!ud._liquidMesh) {
        const geo = new THREE.CylinderGeometry(R, R, H, 20, 1);
        ud._liquidMesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ transparent: true, opacity: 0.8, roughness: 0.1 }));
        mesh.add(ud._liquidMesh);
      }
      const lm = ud._liquidMesh;
      lm.scale.y = Math.max(0.001, ratio);
      lm.position.y = 0.25 + (H * ratio) / 2;
      lm.material.color.set(color);
      lm.visible = vol > 0.5;
      return;
    }
    oldSetter(mesh, vol, color);
  };
})();

// 台面默认布局：试管架 + 烧杯 + 锥形瓶 + 酒精灯 + 三脚架 + 铁架台 + 滴定管
const items = {};
items.rack = placeOnCentralBench(APPARATUS_FACTORIES.tube_rack(), -1.6, 0.4);
items.stand = placeOnCentralBench(APPARATUS_FACTORIES.stand(), 1.8, 0.2);
items.burette = placeOnCentralBench(APPARATUS_FACTORIES.burette_50(), 1.92, 0.2);
items.burette.mesh.position.set(1.92, BENCH_Y + 0.6, 0.2);
// 锥形瓶放在滴定管正下方
items.flask = placeOnCentralBench(APPARATUS_FACTORIES.flask_conical_250(), 1.92, 0.2);
items.flask.mesh.position.set(1.92, BENCH_Y, 0.35);
items.beaker = placeOnCentralBench(APPARATUS_FACTORIES.beaker_250(), -0.6, 0.5);
items.testTube1 = placeOnCentralBench(APPARATUS_FACTORIES.test_tube(), -1.6, 0.4);
items.testTube1.mesh.position.set(-1.85, BENCH_Y + 0.32, 0.4);
items.stand = placeOnCentralBench(APPARATUS_FACTORIES.stand(), 1.8, 0.2);
items.burette = placeOnCentralBench(APPARATUS_FACTORIES.burette_50(), 1.92, 0.2);
items.burette.mesh.position.set(1.92, BENCH_Y + 0.6, 0.2);
items.lamp = placeOnCentralBench(APPARATUS_FACTORIES.alcohol_lamp(), -2.0, -0.4);
items.tripod = placeOnCentralBench(APPARATUS_FACTORIES.tripod(), -1.1, -0.45);
items.tripod.mesh.position.set(-1.1, BENCH_Y, -0.45);
// 石棉网放三脚架上
items.gauze = placeOnCentralBench(APPARATUS_FACTORIES.wire_gauze(), -1.1, -0.45);
items.gauze.mesh.position.set(-1.1, BENCH_Y + 0.33, -0.45);
// 仪器边台：磁力搅拌器、天平、pH 计
items.stirrer = engine.register(APPARATUS_FACTORIES.magnetic_stirrer());
items.stirrer.mesh.position.set(-1.8, BENCH_Y, -5.2);
items.balance = engine.register(APPARATUS_FACTORIES.balance());
items.balance.mesh.position.set(-0.2, BENCH_Y, -5.2);
items.phmeter = engine.register(APPARATUS_FACTORIES.ph_meter());
items.phmeter.mesh.position.set(1.6, BENCH_Y, -5.2);
items.washBottle = engine.register(APPARATUS_FACTORIES.wash_bottle());
items.washBottle.mesh.position.set(2.6, BENCH_Y, -5.2);

// 通风橱内：放一个进行中的"演示"烧杯（装硫酸铜溶液）
const hoodBeaker = engine.register(APPARATUS_FACTORIES.beaker_250());
hoodBeaker.mesh.position.set(-6.0, 0.95, -4.6);
hoodBeaker.content = new ContainerContent();
hoodBeaker.content.add('CuSO4', 100);
engine.refreshLiquid(hoodBeaker);

// ---------- 试剂架面板 ----------
const reagentList = document.getElementById('reagentList');
let currentReagent = null;
let currentTab = 'aqueous';
const tabs = document.querySelectorAll('.rtab');
tabs.forEach(t => t.addEventListener('click', () => {
  tabs.forEach(x => x.classList.remove('active'));
  t.classList.add('active');
  currentTab = t.dataset.tab;
  renderReagents();
}));

function renderReagents() {
  reagentList.innerHTML = '';
  for (const [key, re] of Object.entries(REAGENTS)) {
    if (re.type !== currentTab) continue;
    const div = document.createElement('div');
    div.className = 'reagent-item' + (currentReagent === key ? ' selected' : '');
    div.innerHTML = `
      <div class="swatch" style="background:#${re.color.toString(16).padStart(6, '0')}"></div>
      <div class="rname">${re.name}${re.conc ? `<small>${re.conc}</small>` : ''}</div>
      ${re.hazard ? `<div class="haz">${re.hazard.join('·')}</div>` : ''}`;
    div.addEventListener('click', () => {
      currentReagent = (currentReagent === key) ? null : key;
      renderReagents();
      if (currentReagent) toast(`已选中「${re.name}」→ 点击场景中的容器加入`);
    });
    reagentList.appendChild(div);
  }
}
renderReagents();

// ---------- 仪器栏 ----------
const apparatusList = document.getElementById('apparatusList');
const APPARATUS_LABELS = {
  beaker_50: '烧杯 50mL', beaker_250: '烧杯 250mL', beaker_500: '烧杯 500mL',
  flask_conical_250: '锥形瓶 250mL', flask_round_250: '圆底烧瓶', flask_flat_250: '平底烧瓶',
  test_tube: '试管', cylinder_100: '量筒 100mL', flask_volumetric_100: '容量瓶',
  burette_50: '滴定管 50mL', pipette_10: '移液管 10mL', dropper: '胶头滴管',
  watch_glass: '表面皿', evaporating_dish: '蒸发皿', crucible: '坩埚', mortar: '研钵和杵',
  funnel: '漏斗（含滤纸）', filter_flask: '抽滤瓶', buchner_funnel: '布氏漏斗',
  gas_jar: '集气瓶', desiccator: '干燥器', alcohol_lamp: '酒精灯', tripod: '三脚架',
  wire_gauze: '石棉网', stand: '铁架台', tube_clamp: '试管夹', tube_rack: '试管架',
  thermometer: '温度计', glass_rod: '玻璃棒', spatula: '药匙',
  magnetic_stirrer: '加热磁力搅拌器', stir_bar: '磁力搅拌子', balance: '电子天平',
  ph_meter: '台式 pH 计', wash_bottle: '洗瓶'
};
let addCount = 0;
for (const [key, label] of Object.entries(APPARATUS_LABELS)) {
  const div = document.createElement('div');
  div.className = 'app-item';
  div.textContent = label;
  div.addEventListener('click', () => {
    const mesh = APPARATUS_FACTORIES[key]();
    const angle = addCount * 0.9;
    const x = Math.cos(angle) * (0.6 + (addCount % 4) * 0.5);
    const z = 0.4 + Math.sin(angle) * (0.3 + (addCount % 3) * 0.4);
    mesh.position.set(x, BENCH_Y, z);
    const item = engine.register(mesh);
    addCount++;
    inter.select(item);
    toast(`已添加「${label}」`);
    updateActionBar(item);
  });
  apparatusList.appendChild(div);
}

// ---------- 日志 ----------
const logContent = document.getElementById('logContent');
function log(msg, cls = '') {
  const div = document.createElement('div');
  div.className = cls;
  div.textContent = msg;
  logContent.appendChild(div);
  logContent.scrollTop = logContent.scrollHeight;
  // 实验步骤状态联动
  syncExperimentState(msg);
}
inter.onLog = log;

function toast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.style.display = 'block';
  clearTimeout(t._tm);
  t._tm = setTimeout(() => t.style.display = 'none', 2600);
}

// ---------- 拾取与选中 ----------
const canvas = engine.renderer.domElement;
let downPos = null;
canvas.addEventListener('pointerdown', (e) => { downPos = [e.clientX, e.clientY]; });
canvas.addEventListener('pointerup', (e) => {
  if (!downPos) return;
  const moved = Math.hypot(e.clientX - downPos[0], e.clientY - downPos[1]);
  downPos = null;
  if (moved > 6) return; // 拖动视角不触发
  const hit = inter.raycast(e.clientX, e.clientY);
  if (hit && hit.item) {
    // 若选中了试剂：优先加试剂
    if (currentReagent && hit.item.mesh.userData.capacity !== undefined) {
      const re = REAGENTS[currentReagent];
      let vol;
      if (re.type === 'solid') vol = 5;
      else if (re.type === 'indicator') vol = 0.5;      // 指示剂 2~3 滴
      else vol = 20;
      if (inter.addReagentToContainer(hit.item, currentReagent, vol)) {
        expState.reagentAdded = true;
        if (re.type === 'solid') expState.solidAdded = true;
        if (re.type === 'indicator') expState.indicatorAdded = true;
        updateInspect(hit.item);
        checkSteps();
        return;
      }
    }
    inter.select(hit.item);
    updateActionBar(hit.item);
    updateInspect(hit.item);
  } else {
    inter.select(null);
    updateActionBar(null);
    document.getElementById('inspectPanel').classList.remove('open');
  }
});

// 双击 = 快捷操作（点燃/熄灭酒精灯、倾倒到下方容器）
canvas.addEventListener('dblclick', (e) => {
  const hit = inter.raycast(e.clientX, e.clientY);
  if (!hit || !hit.item) return;
  const ud = hit.item.mesh.userData;
  if (ud.ignitable) {
    inter.toggleLamp(hit.item);
    if (inter.flames.isLit(hit.item.mesh)) {
      expState.heated = true;
      // 检查 Cu(OH)2 分解
      for (const it of engine.registry) {
        if (it.content?.precipitate?.name?.includes('Cu(OH)₂') && engine.isHeated(it)) {
          it.content.precipitate = { color: 0x1a1a1a, name: 'CuO 黑色固体' };
          it.content.color = () => 0x333940;
          log('🔥 蓝色 Cu(OH)₂ 沉淀受热分解为黑色 CuO', 'rxn');
          expState.decomposed = true;
          engine.refreshLiquid(it);
        }
      }
    }
    checkSteps();
  } else if (hit.item.content && hit.item.content.totalVolume() > 0) {
    // 双击手持容器 → 若下方有容器则倾倒
    const below = findPourTargetBelow(hit.item);
    if (below) { inter.pour(hit.item, below); expState.dropped = true; updateInspect(below); checkSteps(); }
    else toast('下方没有可以倒入的容器');
  }
});

function findPourTargetBelow(item) {
  const box = new THREE.Box3().setFromObject(item.mesh);
  const cx = (box.min.x + box.max.x) / 2, cz = (box.min.z + box.max.z) / 2;
  let best = null, bestD = 0.5;
  for (const t of engine.registry) {
    if (t === item || t.mesh.userData.capacity === undefined) continue;
    const tb = new THREE.Box3().setFromObject(t.mesh);
    const tx = (tb.min.x + tb.max.x) / 2, tz = (tb.min.z + tb.max.z) / 2;
    const d = Math.hypot(tx - cx, tz - cz);
    if (d < bestD && tb.max.y < box.min.y + 0.05 && tb.max.y > box.min.y - 0.4) { best = t; bestD = d; }
  }
  return best;
}

// ---------- 操作按钮条 ----------
const actionBar = document.getElementById('actionBar');
const abLabel = document.getElementById('abLabel');
const abButtons = document.getElementById('abButtons');

function updateActionBar(item) {
  if (!item) { actionBar.classList.remove('open'); document.getElementById('buretteCtl').classList.remove('open'); return; }
  const ud = item.mesh.userData;
  abLabel.textContent = `选中：${ud.label || ud.kind}`;
  abButtons.innerHTML = '';
  const mkBtn = (text, fn, cls = '') => {
    const b = document.createElement('button');
    b.className = 'act ' + cls;
    b.textContent = text;
    b.addEventListener('click', fn);
    abButtons.appendChild(b);
    return b;
  };
  if (ud.ignitable) {
    mkBtn(inter.flames.isLit(item.mesh) ? '🚫 熄灭' : '🔥 点燃', () => {
      inter.toggleLamp(item);
      updateActionBar(item);
    }, 'hot');
  }
  if (ud.capacity !== undefined) {
    mkBtn('🫗 清空', () => {
      item.content = new ContainerContent();
      inter.gasEmitters.stop(item.mesh);
      engine.refreshLiquid(item);
      updateInspect(item);
      log(`🧽 已清空「${ud.label}」`);
    }, 'danger');
    if (item.content && item.content.totalVolume() > 0) {
      mkBtn('🌀 搅拌', () => { inter.stir(item); if (item.mesh.userData.kind === 'test_tube') expState.shaken = true; checkSteps(); });
      // 倾倒到下方
      const below = findPourTargetBelow(item);
      if (below) mkBtn(`🫗 倒入「${below.mesh.userData.label}」`, () => {
        inter.pour(item, below);
        expState.dropped = true;
        updateInspect(below); updateInspect(item);
        checkSteps();
      });
    }
  }
  if (ud.kind === 'burette') {
    mkBtn('💧 阀门控制', () => {
      document.getElementById('buretteCtl').classList.toggle('open');
    });
  }
  mkBtn('🗑 移除', () => {
    inter.gasEmitters.stop(item.mesh);
    if (ud.ignitable) inter.flames.extinguish(item.mesh);
    engine.unregister(item);
    inter.select(null);
    updateActionBar(null);
    document.getElementById('inspectPanel').classList.remove('open');
  }, 'danger');
  actionBar.classList.add('open');
}

// 滴定阀门按钮
document.querySelectorAll('#buretteCtl button').forEach(b => {
  b.addEventListener('click', () => {
    inter.setBuretteFlow(parseInt(b.dataset.flow));
    const names = ['关闭', '慢速滴加', '中速', '快速'];
    log(`💧 滴定管阀门：${names[parseInt(b.dataset.flow)]}`);
    if (parseInt(b.dataset.flow) > 0) expState.dropped = true;
    checkSteps();
  });
});

// ---------- 检视面板 ----------
function updateInspect(item) {
  const panel = document.getElementById('inspectPanel');
  if (!item) { panel.classList.remove('open'); return; }
  const ud = item.mesh.userData;
  document.getElementById('inspTitle').textContent = `检视：${ud.label || ud.kind}`;
  const body = document.getElementById('inspBody');
  const rows = [];
  if (ud.capacity !== undefined) rows.push(['容量', `${ud.capacity} mL`]);
  if (item.content) {
    const c = item.content;
    const vol = c.totalVolume();
    if (vol > 0) {
      rows.push(['液体体积', vol.toFixed(1) + ' mL']);
      rows.push(['温度', c.temperature.toFixed(1) + ' ℃']);
      rows.push(['pH（估算）', c.pH().toFixed(1)]);
      for (const e of c.entries) rows.push([REAGENTS[e.key]?.name || e.key, e.volume.toFixed(1) + ' mL']);
      if (c.precipitate) rows.push(['沉淀', c.precipitate.name]);
      if (c.gasProducing) rows.push(['逸出气体', c.gasProducing.name]);
      if (c.lastReaction) rows.push(['最近反应', c.lastReaction.result.equation]);
    } else {
      rows.push(['状态', '空容器']);
    }
  }
  if (ud.fuel !== undefined && ud.ignitable) rows.push(['酒精量', `${Math.round(ud.fuel * 100)}%`]);
  body.innerHTML = rows.map(([k, v]) => `<div class="kv"><span>${k}</span><span>${v}</span></div>`).join('');
  panel.classList.add('open');
}
inter.onInspect = updateInspect;

// ---------- 实验手册 ----------
const manualPanel = document.getElementById('manualPanel');
let currentExperiment = EXPERIMENTS[EXPERIMENTS.length - 1];
let expState = newExperimentState();
let stepIndex = 0;
let manualChosen = false; // 用户选择了具体实验后才做完成判定

function openDialog() {
  const dlg = document.getElementById('expDialog');
  const list = document.getElementById('expList');
  list.innerHTML = '';
  for (const exp of EXPERIMENTS) {
    const card = document.createElement('div');
    card.className = 'exp-card';
    card.innerHTML = `<div class="en">${exp.name}</div><div class="el">${exp.level}</div><div class="eo">${exp.objective}</div>`;
    card.addEventListener('click', () => {
      currentExperiment = exp;
      expState = newExperimentState();
      stepIndex = 0;
      manualChosen = true;
      dlg.classList.remove('open');
      manualPanel.classList.add('open');
      document.getElementById('expTitle').textContent = `📖 ${exp.name}`;
      document.getElementById('expObj').textContent = `${exp.level} · ${exp.objective}`;
      renderSteps();
      log(`▶ 开始实验：${exp.name}`);
      if (exp.setup) setupExperimentScene(exp);
      checkSteps();
    });
    list.appendChild(card);
  }
  dlg.classList.add('open');
}

function setupExperimentScene(exp) {
  // 按实验需求预置器材与试剂
  if (exp.setup.burette) {
    items.burette.content = new ContainerContent();
    items.burette.content.add(exp.setup.burette, 40);
    engine.refreshLiquid(items.burette);
    log(`✅ 已向滴定管注入 ${REAGENTS[exp.setup.burette].name} 40 mL`);
  }
  if (exp.setup.flask) {
    items.flask.content = new ContainerContent();
    items.flask.content.add(exp.setup.flask, 25);
    engine.refreshLiquid(items.flask);
  }
  if (exp.setup.beaker) {
    items.beaker.content = new ContainerContent();
    items.beaker.content.add(exp.setup.beaker, 50);
    engine.refreshLiquid(items.beaker);
    expState.reagentAdded = true;
  }
}

function renderSteps() {
  const box = document.getElementById('expSteps');
  box.innerHTML = '';
  currentExperiment.steps.forEach((s, i) => {
    const div = document.createElement('div');
    div.className = 'step' + (i < stepIndex ? ' done' : i === stepIndex ? ' current' : '');
    div.innerHTML = `<div class="num">${i < stepIndex ? '✓' : i + 1}</div><div>${s.text}</div>`;
    box.appendChild(div);
  });
}

function checkSteps() {
  if (!manualChosen) return; // 未选择实验前不做步骤判定
  let changed = false;
  while (stepIndex < currentExperiment.steps.length && currentExperiment.steps[stepIndex].check(expState)) {
    stepIndex++;
    changed = true;
  }
  if (changed) {
    renderSteps();
    if (stepIndex >= currentExperiment.steps.length) {
      log(`🎉 实验完成：${currentExperiment.name}`, 'rxn');
      toast('🎉 实验完成！可从「实验手册」开始下一个实验');
    }
  }
}

// 日志驱动的实验状态联动
function syncExperimentState(msg) {
  if (msg.includes('加入') && (msg.includes('铁粉') || msg.includes('铁') && msg.includes('固体'))) expState.solidAdded = true;
  if (msg.includes('滴定管')) expState.buretteFilled = true;
  if (msg.includes('锥形瓶')) expState.flaskFilled = true;
  if (msg.includes('酚酞') || msg.includes('石蕊') || msg.includes('甲基橙')) expState.indicatorAdded = true;
  if (msg.includes('硫氰化钾') && msg.includes('加入')) expState.scnAdded = true;
  if (msg.includes('现象：') || msg.includes('🧪')) expState.reacted = true;
  if (msg.includes('生成') || msg.includes('沉淀')) expState.precipitate = true;
  if (msg.includes('CO₂') || msg.includes('气泡')) { expState.gasProduced = true; }
  if (msg.includes('浑浊')) expState.turbidity = true;
  if (msg.includes('倒入') || msg.includes('移入')) expState.transferred = true;
  if (currentExperiment.id === 'fe3_detection' && msg.includes('无明显') && msg.includes('KSCN')) expState.contrastDone = true;
  // 酚酞变红（pH ≥ 8.2 且含酚酞）→ 滴定终点
  if (currentExperiment.id === 'acid_base_titration') {
    for (const it of engine.registry) {
      if (it.content && it.content.entries.some(e => e.key === 'phenolphthalein') && it.content.pH() >= 8.2) {
        expState.endPoint = true;
      }
    }
  }
  checkSteps();
}

document.getElementById('btnManual').addEventListener('click', () => {
  if (manualPanel.classList.contains('open')) manualPanel.classList.remove('open');
  else openDialog();
});
document.getElementById('btnReset').addEventListener('click', () => {
  for (const it of [...engine.registry]) {
    const ud = it.mesh.userData;
    if (ud._registered && !ud.isBench && !ud.isFumeHood && !['reagent_cabinet', 'hazmat_cabinet', 'wash_station', 'emergency_shower', 'fire_extinguisher', 'first_aid', 'gas_cylinder', 'safety_board'].includes(ud.kind)) {
      if (ud.kind === 'burette' || ud.kind === 'stand' || ['tube_rack'].includes(ud.kind) || ['alcohol_lamp', 'tripod', 'wire_gauze', 'magnetic_stirrer', 'balance', 'ph_meter', 'wash_bottle'].includes(ud.kind)) {
        it.content = null; inter.gasEmitters.stop(it.mesh);
        if (ud.ignitable) inter.flames.extinguish(it.mesh);
        engine.refreshLiquid(it);
      } else {
        inter.gasEmitters.stop(it.mesh);
        if (ud.ignitable) inter.flames.extinguish(it.mesh);
        engine.unregister(it);
      }
    }
  }
  log('↺ 台面已重置');
  expState = newExperimentState();
  stepIndex = 0;
  renderSteps();
});
document.getElementById('btnHelp').addEventListener('click', () => {
  toast('选中试剂→点容器加入；双击酒精灯点燃；选中容器→操作条倾倒/搅拌/清空；滴定管→阀门控制');
});

// 滴定 tick：动态解析当前滴定管，避免闭包引用过期对象
setInterval(() => {
  if (inter.buretteFlow > 0) {
    const dt = 0.1;
    const burette = engine.registry.find(i => i.mesh.userData.kind === 'burette' && i.content && i.content.totalVolume() > 0);
    if (burette) {
      inter.buretteTick(burette, dt);
      if (inter.selected && inter.selected.mesh.userData.kind !== 'burette') updateInspect(inter.selected);
    }
  }
}, 100);

// 窗口自适应
window.addEventListener('resize', () => engine.resize());

// 欢迎日志
log('👋 欢迎来到 3D 虚拟化学实验室');
log('   本实验室按真实化学实验室设计规范布置：通风橱（面风速 0.5 m/s）、微负压气流组织、');
log('   试剂分类柜与双人双锁危化品柜、洗涤区、紧急喷淋与洗眼器、CO₂ 灭火器、急救箱。');
log('   点击顶部「实验手册」选择引导实验，或从试剂架自由取用试剂开始探究。');

// 启动：显示实验选择
openDialog();

// 调试钩子（浏览器自动化测试用）
window.__chemlab = { engine, inter, items, REAGENTS, EXPERIMENTS, get expState() { return expState; }, log };
