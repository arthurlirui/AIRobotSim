// 波动与热学设备：发波水槽（双源干涉）、量热器（混合法测比热容）
import * as THREE from 'three';
import { EquipmentBase } from '../EquipmentBase.js';
import { registerCategory, registerEquipment } from '../registry.js';

registerCategory('waves', '波动');
registerCategory('thermal', '热学');

/* ============ 发波水槽：双源干涉 λ, d 可调 ============ */
export class RippleTank extends EquipmentBase {
  constructor(zone) { super(zone, getDef('ripple-tank')); this.initParams(); this.build(); }
  paramSchema() {
    return [
      { key: 'mode', label: '波源模式', type: 'select', options: [['single', '单波源'], ['double', '双波源干涉'], ['two-src', '两独立源']], value: 'double' },
      { key: 'freq', label: '频率 f (Hz)', min: 0.2, max: 2.0, step: 0.05, value: 0.8 },
      { key: 'lambda', label: '波长 λ (m)', min: 0.15, max: 0.6, step: 0.01, value: 0.3 },
      { key: 'd', label: '波源间距 d (m)', min: 0.1, max: 0.9, step: 0.05, value: 0.5 },
      { key: 'amp', label: '振幅', min: 0.1, max: 1, step: 0.05, value: 0.5 },
    ];
  }
  build() {
    const g = this.group;
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x39485c, roughness: 0.5 });
    // 水槽框
    const W = 1.3, D = 0.9;
    const frame = new THREE.Mesh(new THREE.BoxGeometry(W + 0.06, 0.1, D + 0.06), frameMat);
    frame.position.y = 0.05;
    g.add(frame);
    // 水面（动态顶点位移）
    const SEG = 64;
    this.waterGeo = new THREE.PlaneGeometry(W, D, SEG, Math.round(SEG * D / W));
    this.water = new THREE.Mesh(this.waterGeo, new THREE.MeshStandardMaterial({
      color: 0x2d8fd6, transparent: true, opacity: 0.82, roughness: 0.15, metalness: 0.1,
      emissive: 0x0a2a44, emissiveIntensity: 0.4,
    }));
    this.water.rotation.x = -Math.PI / 2;
    this.water.position.y = 0.1;
    this.water.userData.highlightable = false;
    g.add(this.water);
    // 波源振子
    this.srcSpheres = [];
    for (let i = 0; i < 2; i++) {
      const s = new THREE.Mesh(new THREE.SphereGeometry(0.03, 12, 10),
        new THREE.MeshStandardMaterial({ color: 0xffd93d, emissive: 0x886611 }));
      s.position.y = 0.13;
      s.visible = false;
      g.add(s);
      this.srcSpheres.push(s);
    }
    this.state.t = 0;
    this.onParamsChanged();
  }
  reset() { this.state.t = 0; }
  onParamsChanged() {
    const { mode, d } = this.params;
    this.srcSpheres[0].position.set(-d / 2, 0.13, 0);
    this.srcSpheres[1].position.set(d / 2, 0.13, 0);
    this.srcSpheres[0].visible = mode !== 'none';
    this.srcSpheres[1].visible = mode !== 'single';
    // 相位关系
    this._inPhase = mode !== 'two-src';
  }
  update(dt) {
    this.state.t += dt;
    const { freq, lambda, amp, mode, d } = this.params;
    const t = this.state.t;
    const k = 2 * Math.PI / lambda;
    const omega = 2 * Math.PI * freq;
    const pos = this.waterGeo.attributes.position;
    const W = 1.3;
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i), y = pos.getY(i); // 平面局部坐标（rotated 后 y 是深度）
      let h = 0;
      if (mode === 'single') {
        const r1 = Math.hypot(x + d / 2, y);
        h = amp * 0.045 * Math.sin(omega * t - k * r1) * Math.min(r1 * 4, 1);
      } else {
        const ph2 = this._inPhase ? 0 : Math.PI;
        const r1 = Math.hypot(x + d / 2, y);
        const r2 = Math.hypot(x - d / 2, y);
        h = amp * 0.03 * (Math.sin(omega * t - k * r1) * Math.min(r1 * 4, 1)
          + Math.sin(omega * t - k * r2 + ph2) * Math.min(r2 * 4, 1));
      }
      pos.setZ(i, h);
    }
    pos.needsUpdate = true;
    this.waterGeo.computeVertexNormals();
    // 波源振子上下振动
    const bob = Math.sin(omega * t) * 0.02 * amp;
    this.srcSpheres[0].position.y = 0.13 + bob;
    this.srcSpheres[1].position.y = 0.13 + (this._inPhase ? bob : -bob);
  }
  getReadouts() {
    return {
      'f (Hz)': this.params.freq,
      'λ (m)': this.params.lambda,
      'd (m)': this.params.d,
      'v = λf (m/s)': +(this.params.lambda * this.params.freq).toFixed(3),
    };
  }
}

/* ============ 量热器：混合法测比热容 ============ */
export class Calorimeter extends EquipmentBase {
  constructor(zone) { super(zone, getDef('calorimeter')); this.initParams(); this.build(); }
  paramSchema() {
    return [
      { key: 'mHot', label: '热水质量 (kg)', min: 0.05, max: 0.5, step: 0.05, value: 0.2 },
      { key: 'THot', label: '热水初温 (°C)', min: 40, max: 95, step: 1, value: 80 },
      { key: 'mCold', label: '冷水质量 (kg)', min: 0.05, max: 0.5, step: 0.05, value: 0.2 },
      { key: 'TCold', label: '冷水初温 (°C)', min: 5, max: 35, step: 1, value: 20 },
      { key: 'rate', label: '混合速率', min: 0.2, max: 3, step: 0.1, value: 1 },
    ];
  }
  build() {
    const g = this.group;
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xd8dde4, roughness: 0.4, metalness: 0.3 });
    const outer = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.15, 0.24, 24), bodyMat);
    outer.position.y = 0.12; outer.castShadow = true;
    g.add(outer);
    const lid = new THREE.Mesh(new THREE.CylinderGeometry(0.145, 0.145, 0.02, 24),
      new THREE.MeshStandardMaterial({ color: 0xb0b8c2 }));
    lid.position.y = 0.25;
    g.add(lid);
    // 温度计
    this.thermo = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.22, 8),
      new THREE.MeshStandardMaterial({ color: 0xe8f2ff, transparent: true, opacity: 0.7 }));
    this.thermo.position.set(0.04, 0.32, 0);
    g.add(this.thermo);
    this.thermoBulb = new THREE.Mesh(new THREE.SphereGeometry(0.016, 10, 8),
      new THREE.MeshStandardMaterial({ color: 0xd42040, emissive: 0x551015 }));
    this.thermoBulb.position.set(0.04, 0.2, 0);
    g.add(this.thermoBulb);
    // 温度曲线小屏
    this.plotSprite = null;
    this.reset();
  }
  reset() {
    const p = this.params;
    this.state = { t: 0, T: p.TCold, mixed: false, THotNow: p.THot, TColdNow: p.TCold, history: [] };
  }
  startMix() {
    if (this.state.mixed) { this.toast('已混合，请先重置'); return; }
    this.state.mixed = true;
    this.state.mixStart = this.state.t;
  }
  update(dt) {
    const s = this.state, p = this.params;
    s.t += dt;
    if (s.mixed) {
      // 两部分水换热：m_hot*c*(T_hot - T_final) = m_cold*c*(T_final - T_cold)
      const Tfinal = (p.mHot * p.THot + p.mCold * p.TCold) / (p.mHot + p.mCold);
      const tau = 3 / p.rate;
      const dtm = s.t - s.mixStart;
      const k = 1 - Math.exp(-dtm / tau);
      s.THotNow = p.THot + (Tfinal - p.THot) * k;
      s.TColdNow = p.TCold + (Tfinal - p.TCold) * k;
      s.T = p.mHot * s.THotNow + p.mCold * s.TColdNow;
      s.T /= (p.mHot + p.mCold);
    }
    // 记录温度历史（每 0.2s）
    if (!s._lastRec || s.t - s._lastRec > 0.2) {
      s._lastRec = s.t;
      s.history.push([+s.t.toFixed(2), +s.T.toFixed(2)]);
      if (s.history.length > 300) s.history.shift();
    }
    // 温度计液柱
    const frac = Math.max(0, Math.min(1, (s.T - 0) / 100));
    this.thermoBulb.material.emissiveIntensity = 0.4 + frac * 0.6;
  }
  getReadouts() {
    const s = this.state, p = this.params;
    const Tfinal = (p.mHot * p.THot + p.mCold * p.TCold) / (p.mHot + p.mCold);
    return {
      't (s)': +s.t.toFixed(1),
      'T_hot (°C)': +s.THotNow.toFixed(1),
      'T_cold (°C)': +s.TColdNow.toFixed(1),
      'T 显示 (°C)': +s.T.toFixed(1),
      '理论终温 (°C)': +Tfinal.toFixed(1),
    };
  }
  actions() { return [{ key: 'mix', label: '🫗 开始混合' }]; }
  doAction(key) { if (key === 'mix') this.startMix(); }
}

/* ============ 数显秒表（测量类） ============ */
export class Stopwatch extends EquipmentBase {
  constructor(zone) { super(zone, getDef('stopwatch')); this.initParams(); this.build(); }
  paramSchema() { return []; }
  build() {
    const g = this.group;
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.075, 0.025, 20),
      new THREE.MeshStandardMaterial({ color: 0x2a3442, roughness: 0.4 }));
    body.position.y = 0.013;
    g.add(body);
    this.display = new THREE.Mesh(new THREE.CircleGeometry(0.05, 20),
      new THREE.MeshBasicMaterial({ color: 0x0a2a18 }));
    this.display.rotation.x = -Math.PI / 2;
    this.display.position.y = 0.027;
    g.add(this.display);
    this.state = { running: false, t: 0, laps: [] };
    this.label = this.makeLabel('0.00 s', 0.12, '#5dd39e');
  }
  update(dt) { if (this.state.running) { this.state.t += dt; this._apply(); } }
  _apply() {
    if (this.label) this.label.material.map.dispose(), this.label = null;
    this.label = this.makeLabel(this.state.t.toFixed(2) + ' s', 0.12, '#5dd39e');
  }
  toggle() { this.state.running = !this.state.running; }
  lap() { this.state.laps.push(this.state.t); }
  resetTime() { this.state = { running: false, t: 0, laps: [] }; this._apply(); }
  actions() {
    return [
      { key: 'toggle', label: '⏯ 启动/暂停' },
      { key: 'lap', label: '🚩 计次' },
      { key: 'resetTime', label: '↺ 归零' },
    ];
  }
  doAction(key) { this[key](); }
  getReadouts() { return { '秒表 (s)': +this.state.t.toFixed(2), '计次数': this.state.laps.length }; }
}

/* ============ 光电门（自动测周期） ============ */
export class PhotoGate extends EquipmentBase {
  constructor(zone) { super(zone, getDef('photogate')); this.initParams(); this.build(); }
  paramSchema() { return []; }
  build() {
    const g = this.group;
    const mat = new THREE.MeshStandardMaterial({ color: 0x3a4552, roughness: 0.5 });
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.28, 0.05), mat);
    arm.position.y = 0.14;
    g.add(arm);
    this.gapTop = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.03, 0.04),
      new THREE.MeshStandardMaterial({ color: 0x4cc3ff, emissive: 0x115577 }));
    this.gapTop.position.set(0.035, 0.26, 0);
    g.add(this.gapTop);
    const baseM = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.02, 0.1), mat);
    baseM.position.y = 0.01;
    g.add(baseM);
    this.state = { t: 0, blocked: false, lastBlock: null, lastPeriod: null, count: 0 };
    this.label = this.makeLabel('光电门', 0.36, '#4cc3ff');
  }
  update(dt) {
    // 检测遮挡：与场景中单摆球等运动体求交由 App 广播位置，这里简化为读 EventBus
    this.state.t += dt;
  }
  // App 每帧调用：传入是否有物体遮挡
  setBlocked(b) {
    const s = this.state;
    if (b && !s.blocked) {
      if (s.lastBlock !== null) s.lastPeriod = s.t - s.lastBlock;
      s.lastBlock = s.t;
      s.count++;
    }
    s.blocked = b;
    this.gapTop.material.emissiveIntensity = b ? 0.2 : 1.0;
  }
  getReadouts() {
    const s = this.state;
    return {
      't (s)': +s.t.toFixed(2),
      '遮挡次数': s.count,
      '周期/间隔 (s)': s.lastPeriod === null ? '—' : +s.lastPeriod.toFixed(3),
    };
  }
}

const DEFS_WAVE = [
  { id: 'ripple-tank', name: '发波水槽', icon: '🌊', desc: '双源干涉 · 波长频率可调', make: RippleTank },
];
const DEFS_THERMAL = [
  { id: 'calorimeter', name: '量热器', icon: '🌡', desc: '混合法测比热容', make: Calorimeter },
];
const DEFS_MEAS = [
  { id: 'stopwatch', name: '数显秒表', icon: '⏱', desc: '计时/计次', make: Stopwatch },
  { id: 'photogate', name: '光电门', icon: '🔦', desc: '自动测周期', make: PhotoGate },
];
DEFS_WAVE.forEach(d => registerEquipment('waves', d));
DEFS_THERMAL.forEach(d => registerEquipment('thermal', d));
DEFS_MEAS.forEach(d => registerEquipment('measurement', d));
function getDef(id) {
  return [...DEFS_WAVE, ...DEFS_THERMAL, ...DEFS_MEAS].find(d => d.id === id);
}
