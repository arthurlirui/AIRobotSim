// 电学实验台：组件化电路（电池、开关、电阻、灯泡、变阻器、导线）
// 求解器：改进节点分析法（MNA），支持电阻 / 电压源 / 开关 / 灯泡（非线性电阻简单迭代）
import * as THREE from 'three';
import { EquipmentBase } from '../EquipmentBase.js';
import { registerCategory, registerEquipment } from '../registry.js';

registerCategory('electricity', '电学');

const KNOB_R = 0.028; // 接线柱半径（拾取热区）

/* ============ 电学实验台：拖入组件 + 点击接线柱连线 ============ */
export class CircuitBench extends EquipmentBase {
  constructor(zone) {
    super(zone, getDef('circuit-bench'));
    this.isCircuitHost = true;
    this.initParams(); this.build();
  }
  paramSchema() { return []; }
  build() {
    const g = this.group;
    // 面包板造型
    const board = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.03, 1.0),
      new THREE.MeshStandardMaterial({ color: 0xf5f0e6, roughness: 0.7 }));
    board.position.y = 0.015;
    board.receiveShadow = true;
    g.add(board);
    const gridMat = new THREE.MeshBasicMaterial({ color: 0xd8d0c0 });
    // 网格孔
    const holeGeo = new THREE.CircleGeometry(0.006, 8);
    const holes = new THREE.InstancedMesh(holeGeo, gridMat, 0);
    let idx = 0;
    const cols = 14, rows = 9;
    holes.count = cols * rows;
    const m = new THREE.Matrix4();
    for (let i = 0; i < cols; i++) for (let j = 0; j < rows; j++) {
      m.setPosition(-0.65 + i * 0.1, 0.032, -0.35 + j * 0.0875);
      holes.setMatrixAt(idx++, m);
    }
    g.add(holes);
    this.components = [];   // 电路组件
    this.wires = [];        // 导线 [{a: comp, b: comp}]
    this.state.t = 0;
    this.solution = null;
  }
  update(dt) {
    this.state.t += dt;
    this.solve();
    // 更新组件视觉（灯泡亮度、电流动画）
    for (const c of this.components) c.applySolution(this.solution, dt);
  }
  // 添加组件（由设备仓库拖入时，实际上电路台是一体设备）
  addComponent(kind) {
    const comp = createComponent(kind, this);
    // 摆放在板面空位
    const n = this.components.length;
    const slots = [[-0.5, -0.2], [0.0, -0.2], [0.5, -0.2], [-0.5, 0.22], [0.0, 0.22], [0.5, 0.22], [-0.25, 0.0], [0.25, 0.0]];
    const [sx, sz] = slots[n % slots.length];
    comp.group.position.set(sx, 0.03, sz);
    this.group.add(comp.group);
    this.components.push(comp);
    return comp;
  }
  removeComponent(comp) {
    const i = this.components.indexOf(comp);
    if (i >= 0) this.components.splice(i, 1);
    this.wires = this.wires.filter(w => w.a.comp !== comp && w.b.comp !== comp);
    this.group.remove(comp.group);
    this.refreshWires();
  }
  // 连接两接线柱
  connect(termA, termB) {
    if (termA === termB || termA.comp === termB.comp) return false;
    // 去重
    const exists = this.wires.some(w =>
      (w.a === termA && w.b === termB) || (w.a === termB && w.b === termA));
    if (exists) return false;
    this.wires.push({ a: termA, b: termB });
    this.refreshWires();
    return true;
  }
  disconnectWire(wire) {
    const i = this.wires.indexOf(wire);
    if (i >= 0) this.wires.splice(i, 1);
    this.refreshWires();
  }
  refreshWires() {
    if (this.wireGroup) {
      this.group.remove(this.wireGroup);
      this.wireGroup.traverse(c => { c.geometry?.dispose(); c.material?.dispose(); });
    }
    this.wireGroup = new THREE.Group();
    this.wireGroup.userData.highlightable = false;
    for (const w of this.wires) {
      const pa = w.a.getWorldPos(this.group), pb = w.b.getWorldPos(this.group);
      // 贝塞尔垂弧导线
      const mid = pa.clone().add(pb).multiplyScalar(0.5);
      const sag = pa.distanceTo(pb) * 0.14;
      mid.y += 0.02 + sag * 0.3;
      const curve = new THREE.QuadraticBezierCurve3(pa, mid, pb);
      const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, 20, 0.008, 6),
        new THREE.MeshStandardMaterial({ color: 0xc23b3b, roughness: 0.6 }));
      tube.userData.wire = w;
      tube.userData.highlightable = false;
      this.wireGroup.add(tube);
      // 电流流动粒子
      const arrowCount = Math.max(2, Math.round(pa.distanceTo(pb) * 8));
      w.flowGroup = new THREE.Group();
      for (let k = 0; k < arrowCount; k++) {
        const dot = new THREE.Mesh(new THREE.SphereGeometry(0.014, 8, 6),
          new THREE.MeshBasicMaterial({ color: 0xffe066 }));
        dot.userData.t = k / arrowCount;
        dot.userData.highlightable = false;
        w.flowGroup.add(dot);
      }
      this.wireGroup.add(w.flowGroup);
      w.curve = curve; w.flowParent = this.wireGroup;
    }
    this.group.add(this.wireGroup);
  }
  solve() {
    const sol = solveCircuit(this.components, this.wires);
    this.solution = sol;
    // 电流粒子动画
    for (const w of this.wires) {
      const i = sol.wireCurrents.get(w) || 0;
      const dir = Math.sign(i) || 1;
      const speed = Math.min(Math.abs(i) * 0.12, 0.6);
      w.flowGroup?.children.forEach(dot => {
        dot.userData.t = (dot.userData.t + dir * speed * 0.016) % 1;
        if (dot.userData.t < 0) dot.userData.t += 1;
        const p = w.curve.getPointAt(dot.userData.t);
        dot.position.copy(p);
        dot.visible = Math.abs(i) > 0.005;
      });
    }
  }
  actions() {
    return [
      { key: 'add-battery', label: '＋ 电池' },
      { key: 'add-switch', label: '＋ 开关' },
      { key: 'add-resistor', label: '＋ 电阻' },
      { key: 'add-bulb', label: '＋ 灯泡' },
      { key: 'add-rheostat', label: '＋ 变阻器' },
      { key: 'clear', label: '🗑 清空台面' },
    ];
  }
  doAction(key) {
    if (key.startsWith('add-')) {
      const kind = key.slice(4);
      if (this.components.length >= 8) { this.toast('台面组件已达上限（8 个）', 'warn'); return; }
      this.addComponent(kind);
      this.toast('已添加组件：' + { battery: '电池', switch: '开关', resistor: '电阻', bulb: '灯泡', rheostat: '变阻器' }[kind]);
    } else if (key === 'clear') {
      [...this.components].forEach(c => this.removeComponent(c));
    }
  }
  getReadouts() {
    const s = this.solution;
    if (!s) return { 状态: '未求解' };
    return {
      '总电流 (A)': +(s.totalCurrent || 0).toFixed(3),
      '工作组件': this.components.length,
      '导线数': this.wires.length,
    };
  }
  serialize() {
    const base = super.serialize();
    base.circuit = {
      components: this.components.map(c => ({ kind: c.kind, pos: c.group.position.toArray(), on: c.isOn?.() ?? true, value: c.params?.value })),
      wires: this.wires.map(w => [
        this.components.indexOf(w.a.comp) >= 0 && this.components.indexOf(w.a.comp) === this.components.indexOf(w.a.comp) ? null : null,
      ]),
    };
    return base;
  }
}

/* ---------- MNA 电路求解 ---------- */
// 图 = components（含端子）+ wires。简化：每个组件两端子。
export function solveCircuit(components, wires) {
  // 构建节点：每个组件端子是一个节点；导线合并节点（Union-Find）
  const parent = new Map();
  const find = (x) => { if (!parent.has(x)) { parent.set(x, x); return x; } let r = x; while (parent.get(r) !== r) r = parent.get(r); let c = x; while (parent.get(c) !== c) { const n = parent.get(c); parent.set(c, r); c = n; } return r; };
  const union = (a, b) => { const ra = find(a), rb = find(b); if (ra !== rb) parent.set(ra, rb); };

  for (const c of components) for (const t of c.terminals) find(t);
  for (const w of wires) union(w.a, w.b);

  // 收集不同节点
  const nodes = new Set();
  for (const t of parent.keys()) nodes.add(find(t));
  const nodeList = [...nodes];
  const idx = new Map(nodeList.map((n, i) => [n, i]));
  const N = nodeList.length;

  // 找电压源（电池）
  const batteries = components.filter(c => c.kind === 'battery' && c.isOn());
  if (N === 0 || batteries.length === 0) return { nodeVoltages: new Map(), wireCurrents: new Map(), totalCurrent: 0 };

  // MNA：未知量 = 节点电压(N-1, 以地为参考=电池负极) + 电压源电流
  // 接地：第一个电池的负极
  const gnd = find(batteries[0].terminals[1]);
  const gndIdx = idx.get(gnd);
  const size = N - 1 + batteries.length;
  if (size <= 0 || size > 60) return { nodeVoltages: new Map(), wireCurrents: new Map(), totalCurrent: 0 };
  const A = Array.from({ length: size }, () => new Float64Array(size));
  const z = new Float64Array(size);

  const nodeVar = (nodeIdx) => nodeIdx === gndIdx ? -1 : nodeIdx > gndIdx ? nodeIdx - 1 : nodeIdx;

  // 电导矩阵
  for (const c of components) {
    if (c.kind === 'battery') continue;
    const R = c.resistance();
    if (!(R > 0) || !isFinite(R)) continue;
    const na = find(c.terminals[0]), nb = find(c.terminals[1]);
    const ia = idx.get(na), ib = idx.get(nb);
    const va = nodeVar(ia), vb = nodeVar(ib);
    const g = 1 / R;
    if (va >= 0) A[va][va] += g;
    if (vb >= 0) A[vb][vb] += g;
    if (va >= 0 && vb >= 0) { A[va][vb] -= g; A[vb][va] -= g; }
  }
  // 电压源行
  batteries.forEach((b, bi) => {
    const row = N - 1 + bi;
    const na = find(b.terminals[0]), nb = find(b.terminals[1]);
    const ia = nodeVar(idx.get(na)), ib = nodeVar(idx.get(nb));
    if (ia >= 0) { A[row][ia] += 1; A[ia][row] += 1; }
    if (ib >= 0) { A[row][ib] -= 1; A[ib][row] -= 1; }
    z[row] = b.voltage();
  });

  const x = solveLinear(A, z);
  if (!x) return { nodeVoltages: new Map(), wireCurrents: new Map(), totalCurrent: 0 };

  // 节点电压
  const nodeVoltages = new Map();
  nodeList.forEach((n, i) => {
    const v = nodeVar(i) >= 0 ? x[nodeVar(i)] : 0;
    nodeVoltages.set(n, v);
  });
  // 每组件电流
  const compCurrent = new Map();
  for (const c of components) {
    if (c.kind === 'battery') continue;
    const na = find(c.terminals[0]), nb = find(c.terminals[1]);
    const va = nodeVoltages.get(na) ?? 0, vb = nodeVoltages.get(nb) ?? 0;
    const R = c.resistance();
    compCurrent.set(c, R > 0 && isFinite(R) ? (va - vb) / R : 0);
  }
  // 电压源电流（流出正极）
  const totalCurrent = batteries.map((b, bi) => x[N - 1 + bi]).reduce((a, v) => a + v, 0);
  // 每根导线电流：取其端子所属组件支路电流（电池端子取电压源电流）
  const wireCurrents = new Map();
  for (const w of wires) {
    const comp = w.a.comp;
    let i = compCurrent.get(comp);
    if (comp.kind === 'battery') {
      const bi = batteries.indexOf(comp);
      i = bi >= 0 ? x[N - 1 + bi] : 0;
    }
    wireCurrents.set(w, i || 0);
  }
  return { nodeVoltages, compCurrent, totalCurrent, wireCurrents };
}

function solveLinear(A, z) {
  const n = A.length;
  const M = A.map((r, i) => Float64Array.from([...r, z[i]]));
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-12) return null;
    [M[col], M[piv]] = [M[piv], M[col]];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col] / M[col][col];
      for (let c2 = col; c2 <= n; c2++) M[r][c2] -= f * M[col][c2];
    }
  }
  const x = new Float64Array(n);
  for (let i = 0; i < n; i++) x[i] = M[i][n] / M[i][i];
  return x;
}

/* ---------- 组件 ---------- */
export class CircuitComponent {
  constructor(kind, bench, name, defR) {
    this.kind = kind;
    this.bench = bench;
    this.name = name;
    this.defR = defR;
    this.group = new THREE.Group();
    this.group.userData.circuitComponent = this;
    this.terminals = []; // {comp, offset, getWorldPos}
    this.params = { value: defR };
    this.voltageAcross = 0;
    this.current = 0;
  }
  isOn() { return this._on !== false; }
  resistance() { return this.defR; }
  voltage() { return 0; }
  applySolution(sol, dt) {
    const i = sol.compCurrent?.get(this) || 0;
    this.current = i;
    const na = sol.nodeVoltages?.get(this.bench._findNode?.(this.terminals[0]));
    this.voltageAcross = 0;
  }
  addTerminal(dx, dy, dz) {
    const t = {
      comp: this, offset: new THREE.Vector3(dx, dy, dz),
      getWorldPos: (parentGroup) => {
        const wp = this.group.position.clone().add(t.offset);
        return wp;
      },
    };
    this.terminals.push(t);
    // 接线柱造型
    const knob = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.02, 0.03, 10),
      new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.7, roughness: 0.35 }));
    knob.position.set(dx, dy, dz);
    knob.userData.terminal = t;
    this.group.add(knob);
    const cap = new THREE.Mesh(new THREE.SphereGeometry(0.017, 10, 8),
      new THREE.MeshStandardMaterial({ color: 0xd23b3b, metalness: 0.6, roughness: 0.3 }));
    cap.position.set(dx, dy + 0.015, dz);
    cap.userData.terminal = t;
    this.group.add(cap);
    return t;
  }
}

class Battery extends CircuitComponent {
  constructor(bench) { super('battery', bench, '电池', 0); this.v = 3.0; this._on = true; this.build(); }
  voltage() { return this._on ? this.v : 0; }
  resistance() { return Infinity; }
  resistanceForSolver() { return Infinity; }
  build() {
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.16, 14),
      new THREE.MeshStandardMaterial({ color: 0x2f3b4c, roughness: 0.5 }));
    body.rotation.x = Math.PI / 2;
    this.group.add(body);
    const plus = new THREE.Mesh(new THREE.BoxGeometry(0.028, 0.02, 0.008),
      new THREE.MeshStandardMaterial({ color: 0xff4444, emissive: 0x551111 }));
    plus.position.set(0.026, 0, 0.078);
    this.group.add(plus);
    this.addTerminal(0, 0, 0.09);
    this.addTerminal(0, 0, -0.09);
  }
  applySolution(sol, dt) { this.current = sol.totalCurrent || 0; }
}
class Switch extends CircuitComponent {
  constructor(bench) { super('switch', bench, '开关', 0); this._on = false; this.build(); }
  resistance() { return this._on ? 0.001 : 1e12; }
  isOn() { return this._on; }
  build() {
    const base = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.02, 0.07),
      new THREE.MeshStandardMaterial({ color: 0xe8e2d0, roughness: 0.6 }));
    this.group.add(base);
    this.lever = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.012, 0.1),
      new THREE.MeshStandardMaterial({ color: 0x444a55, metalness: 0.6 }));
    this.lever.position.set(0, 0.02, 0.028);
    this.lever.rotation.x = -0.7;
    this.group.add(this.lever);
    this.addTerminal(0, 0.01, 0.062);
    this.addTerminal(0, 0.01, -0.062);
  }
  toggle() {
    this._on = !this._on;
    this.lever.rotation.x = this._on ? -0.05 : -0.7;
    this.lever.material.color.set(this._on ? 0x4dc26b : 0x444a55);
  }
  onClicked() { this.toggle(); }
}
class Resistor extends CircuitComponent {
  constructor(bench) { super('resistor', bench, '定值电阻', 10); this.R = 10; this.build(); }
  resistance() { return this.R; }
  build() {
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.11, 12),
      new THREE.MeshStandardMaterial({ color: 0xc9a06a, roughness: 0.7 }));
    body.rotation.z = Math.PI / 2;
    this.group.add(body);
    for (const s of [-1, 1]) {
      const band = new THREE.Mesh(new THREE.TorusGeometry(0.023, 0.004, 6, 14),
        new THREE.MeshStandardMaterial({ color: 0x993333 }));
      band.position.x = s * 0.025;
      this.group.add(band);
    }
    this.addTerminal(0.07, 0, 0);
    this.addTerminal(-0.07, 0, 0);
  }
}
class Bulb extends CircuitComponent {
  constructor(bench) { super('bulb', bench, '小灯泡', 8); this.R = 8; this.build(); }
  resistance() { return this.R; }
  build() {
    const glass = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 12),
      new THREE.MeshStandardMaterial({ color: 0xfff7d0, transparent: true, opacity: 0.5, roughness: 0.1 }));
    glass.position.y = 0.05;
    this.group.add(glass);
    this.filament = new THREE.Mesh(new THREE.SphereGeometry(0.016, 10, 8),
      new THREE.MeshStandardMaterial({ color: 0x886644, emissive: 0x000000 }));
    this.filament.position.y = 0.05;
    this.group.add(this.filament);
    const screw = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.03, 0.05, 12),
      new THREE.MeshStandardMaterial({ color: 0xb9c0c8, metalness: 0.8, roughness: 0.3 }));
    this.group.add(screw);
    this.addTerminal(0.045, 0, 0);
    this.addTerminal(-0.045, 0, 0);
  }
  applySolution(sol, dt) {
    const i = sol.compCurrent?.get(this) || 0;
    const P = i * i * this.R;
    const glow = Math.min(P / 1.2, 1);
    this.filament.material.emissive.setRGB(glow * 1.6, glow * 1.1, glow * 0.45);
    this.filament.material.color.setRGB(0.55 + glow * 0.45, 0.4 + glow * 0.5, 0.27 + glow * 0.3);
    this.current = i;
  }
}
class Rheostat extends CircuitComponent {
  constructor(bench) { super('rheostat', bench, '滑动变阻器', 20); this.R = 20; this.build(); }
  resistance() { return this.R; }
  setFraction(f) { // 0..1
    this.R = Math.max(0.5, Math.round(20 * (1 - f)));
    if (this.slider) this.slider.position.x = -0.07 + f * 0.14;
  }
  build() {
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.18, 12),
      new THREE.MeshStandardMaterial({ color: 0x5a6a7a, roughness: 0.5 }));
    body.rotation.z = Math.PI / 2;
    this.group.add(body);
    const rail = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.008, 0.02),
      new THREE.MeshStandardMaterial({ color: 0xcfd6de, metalness: 0.8 }));
    rail.position.y = 0.045;
    this.group.add(rail);
    this.slider = new THREE.Mesh(new THREE.BoxGeometry(0.024, 0.03, 0.03),
      new THREE.MeshStandardMaterial({ color: 0x4cc3ff, emissive: 0x114455 }));
    this.slider.position.y = 0.06;
    this.group.add(this.slider);
    this.addTerminal(0.1, 0, 0);
    this.addTerminal(-0.1, 0, 0);
    this.setFraction(0.5);
  }
  onClicked(ray) { return 'drag-slider'; }
}

export function createComponent(kind, bench) {
  switch (kind) {
    case 'battery': return new Battery(bench);
    case 'switch': return new Switch(bench);
    case 'resistor': return new Resistor(bench);
    case 'bulb': return new Bulb(bench);
    case 'rheostat': return new Rheostat(bench);
  }
}

const DEFS = [{ id: 'circuit-bench', name: '电学实验台', icon: '🔌', desc: '搭建电路 · 欧姆定律', make: (z) => new CircuitBench(z) }];
DEFS.forEach(d => registerEquipment('electricity', d));
function getDef(id) { return DEFS.find(d => d.id === id); }
