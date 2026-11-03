// 光学平台：光线步进仿真（光源 → 透镜/镜/棱镜 → 光屏）
// 物理模型：薄透镜折射（解析）、球面镜反射、平面镜反射、三棱镜色散（Cauchy 公式）
import * as THREE from 'three';
import { EquipmentBase } from '../EquipmentBase.js';
import { registerCategory, registerEquipment } from '../registry.js';

registerCategory('optics', '光学');

/* ---------- 光学导轨（所有光学器件的宿主） ---------- */
// 每个光学器件是独立设备，光线由「光源」设备每帧向 +x 方向发射，
// 与同台面的透镜/镜/光屏求交。

const BENCH_Y = 0; // 相对设备 group（group 在台面上）

// 光线步进：从光源出发，与台面上其他光学器件交互
// 返回折线段数组 [{points: [Vector3...], color, intensity}]
export function traceRays(source, elements) {
  const segments = [];
  const sx = source.group.position.x - source.zoneOrigin.x;
  const nRays = source.params.rays;
  const spread = source.params.spread * Math.PI / 180;
  const srcColor = source.params.color;

  for (let i = 0; i < nRays; i++) {
    const t = nRays === 1 ? 0 : (i / (nRays - 1) - 0.5);
    let ang = spread * t;
    if (source.params.type === 'laser') ang = 0;
    // 光线追踪（最多 12 次交互）；y 以光源发射点实际高度为基准
    let x = sx, y = source.params.height + source.params.offsetY, dir = ang;
    const pts = [new THREE.Vector2(x, y)];
    let color = new THREE.Color(srcColor), intensity = 1;
    let hitLensIds = new Set();
    let absorbed = false;
    for (let step = 0; step < 12; step++) {
      const hit = findNextHit(x, y, dir, elements, hitLensIds, source.params.height);
      if (!hit) {
        // 无器件：延伸到远处
        const far = 6;
        pts.push(new THREE.Vector2(x + Math.cos(dir) * far, y + Math.sin(dir) * far));
        break;
      }
      pts.push(new THREE.Vector2(hit.x, hit.y));
      if (hit.type === 'lens') {
        const lens = hit.el;
        const f = lens.params.focal * (lens.subType === 'concave' ? -1 : 1);
        // 薄透镜光路：出射斜率 = 入射斜率 - h/f（h 为打在透镜上的高度）
        const dydx = Math.tan(dir);
        const h = hit.y - source.params.height; // 相对主轴（主轴在光源高度）
        const dydx2 = dydx - h / f;
        dir = Math.atan(dydx2);
        x = hit.x; y = hit.y;
        hitLensIds.add(lens.uid);
      } else if (hit.type === 'mirror') {
        const el = hit.el;
        if (el.subType === 'plane') {
          dir = Math.PI - dir; // 平面镜（法线沿 x）
          x = hit.x - 0.001; y = hit.y;
        } else {
          // 球面镜：法线从镜面点指向球心
          const R = el.params.R * (el.subType === 'convex' ? -1 : 1);
          const cx = -R / 2; // 球心位置（凹面镜 R>0 时球心在镜前 -R/2）
          const dxn = x - cx, dyn = y - source.params.height;
          const nl = Math.hypot(dxn, dyn) || 1e-6;
          const sgn = el.subType === 'concave' ? -1 : 1;
          const Nx = sgn * dxn / nl * -1, Ny = sgn * dyn / nl * -1;
          // 反射: d - 2(d·N)N
          const dx = Math.cos(dir), dy = Math.sin(dir);
          const dot = dx * Nx + dy * Ny;
          dir = Math.atan2(dy - 2 * dot * Ny, dx - 2 * dot * Nx);
          x = hit.x - Math.sign(Math.cos(dir)) * 0.002; y = hit.y;
        }
      } else if (hit.type === 'prism') {
        // 三棱镜色散：一次折射出红绿蓝三色
        const el = hit.el;
        const base = 0.42 + el.params.angle * 0.004; // 偏折角随顶角增大
        for (const [extra, col] of [[0.0, 0xff4b4b], [0.03, 0x4bff6b], [0.06, 0x4b8bff]]) {
          const out = -base - extra;
          segments.push({ points: [new THREE.Vector2(hit.x, hit.y), new THREE.Vector2(hit.x + Math.cos(out) * 4, hit.y + Math.sin(out) * 4)], color: col, intensity: 0.85 });
        }
        absorbed = true;
        break;
      } else if (hit.type === 'screen') {
        hit.el.litAt(hit.y - source.params.height, color);
        break; // 光屏承接后光路终止；入射段仍绘制
      }
    }
    // 光路绘制（除被棱镜吸收分解外，入射光路均可见）
    if (pts.length > 1 && !absorbed) segments.push({ points: pts, color: color.getHex(), intensity });
  }
  return segments;
}

// 找光线下一个相交器件
function findNextHit(x, y, dir, elements, skipLensIds, axisY = 0) {
  let best = null;
  const dx = Math.cos(dir), dy = Math.sin(dir);
  for (const el of elements) {
    if (el.subType === 'screen') {
      const lx = el.group.position.x - el.zoneOrigin.x;
      const t = (lx - x) / dx;
      if (t > 0.01) {
        const hy = y + dy * t;
        if (Math.abs(hy - axisY) < 0.25) {
          if (!best || t < best.dist) best = { type: 'screen', el, x: lx, y: hy, dist: t };
        }
      }
    } else if (el.subType === 'prism') {
      const lx = el.group.position.x - el.zoneOrigin.x;
      const t = (lx - x) / dx;
      if (t > 0.01) {
        const hy = y + dy * t;
        if (Math.abs(hy - axisY) < 0.09 && (!best || t < best.dist)) best = { type: 'prism', el, x: lx, y: hy, dist: t };
      }
    } else if (el.isMirror) {
      const lx = el.group.position.x - el.zoneOrigin.x;
      const t = (lx - x) / dx;
      if (t > 0.01) {
        const hy = y + dy * t;
        const hmax = el.subType === 'plane' ? 0.22 : 0.2;
        if (Math.abs(hy - axisY) < hmax && (!best || t < best.dist)) best = { type: 'mirror', el, x: lx, y: hy, dist: t };
      }
    } else if (el.isLens && !skipLensIds.has(el.uid)) {
      const lx = el.group.position.x - el.zoneOrigin.x;
      const t = (lx - x) / dx;
      if (t > 0.01) {
        const hy = y + dy * t;
        const hmax = el.subType === 'convex' ? 0.16 : 0.13;
        if (Math.abs(hy - axisY) < hmax && (!best || t < best.dist)) best = { type: 'lens', el, x: lx, y: hy, dist: t };
      }
    }
  }
  return best;
}

// (refract 已内联到棱镜色散分支)

/* ---------- 光学器件基类 ---------- */
class OpticalElement extends EquipmentBase {
  constructor(zone, def) { super(zone, def); this.zoneOrigin = { x: 0, z: 0 }; }
  setZoneOrigin(o) { this.zoneOrigin = o; this.onParamsChanged(); }
}

/* ---------- 光源 ---------- */
export class LightSource extends OpticalElement {
  constructor(zone) { super(zone, getDef('light')); this.isSource = true; this.initParams(); this.build(); }
  paramSchema() {
    return [
      { key: 'type', label: '光源类型', type: 'select', options: [['laser', '激光（平行）'], ['point', '点光源（发散）'], ['beam5', '五线束']], value: 'laser' },
      { key: 'rays', label: '光线数', min: 1, max: 7, step: 2, value: 1 },
      { key: 'spread', label: '发散角 (°)', min: 0, max: 40, step: 1, value: 12 },
      { key: 'height', label: '高度 (m)', min: 0.1, max: 0.6, step: 0.01, value: 0.22 },
      { key: 'offsetY', label: '离轴高度 (m)', min: -0.1, max: 0.1, step: 0.005, value: 0 },
      { key: 'color', label: '颜色', type: 'color', value: '#ff5252' },
    ];
  }
  build() {
    const g = this.group;
    const boxMat = new THREE.MeshStandardMaterial({ color: 0x2c3540, roughness: 0.5 });
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.065, 0.18, 16), boxMat);
    body.rotation.z = Math.PI / 2;
    g.add(body);
    this.emitFace = new THREE.Mesh(new THREE.CircleGeometry(0.045, 16),
      new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xffffff, emissiveIntensity: 2 }));
    this.emitFace.rotation.y = Math.PI / 2;
    this.emitFace.position.x = 0.095;
    g.add(this.emitFace);
    this.rayGroup = new THREE.Group();
    g.add(this.rayGroup);
    this.state.segments = [];
    this._apply();
  }
  _apply() {
    this.group.children.forEach(c => { if (c !== this.rayGroup && c.position) { /* noop */ } });
    const h = this.params.height;
    this.group.children.forEach(c => {
      if (c === this.rayGroup || c === this._selectRing) return;
      c.position.y = h;
    });
    this.emitFace.material.emissive.set(this.params.color);
  }
  onParamsChanged() {
    if (this.params.type === 'laser') { this.params.rays = 1; this.params.spread = 0; }
    if (this.params.type === 'point') this.params.rays = 5;
    if (this.params.type === 'beam5') this.params.rays = 5;
    this._apply();
  }
  setSegments(segs) {
    // 清除旧光线
    while (this.rayGroup.children.length) {
      const c = this.rayGroup.children.pop();
      c.geometry?.dispose(); c.material?.dispose();
    }
    for (const seg of segs) {
      const pts3 = seg.points.map(p => new THREE.Vector3(p.x, p.y, 0));
      if (pts3.length < 2) continue;
      const geo = new THREE.BufferGeometry().setFromPoints(pts3);
      const mat = new THREE.LineBasicMaterial({ color: seg.color, transparent: true, opacity: 0.9 * seg.intensity });
      const line = new THREE.Line(geo, mat);
      line.userData.highlightable = false;
      this.rayGroup.add(line);
    }
  }
  getReadouts() { return { '光线数': this.params.rays }; }
}

/* ---------- 凸/凹透镜 ---------- */
export class Lens extends OpticalElement {
  constructor(zone, subType) {
    super(zone, subType === 'convex' ? getDef('lens-convex') : getDef('lens-concave'));
    this.isLens = true; this.subType = subType;
    this.initParams(); this.build();
  }
  paramSchema() {
    const s = [{ key: 'focal', label: '焦距 |f| (m)', min: 0.08, max: 0.8, step: 0.01, value: this.subType === 'convex' ? 0.25 : 0.3 }];
    return s;
  }
  build() {
    const g = this.group;
    const color = 0x8fd0ff;
    const mat = new THREE.MeshPhysicalMaterial({
      color, transparent: true, opacity: 0.4, roughness: 0.05, metalness: 0,
      transmission: 0.9, thickness: 0.02, ior: 1.5,
    });
    if (this.subType === 'convex') {
      // 双凸：两半球扣合
      const halfGeo = new THREE.SphereGeometry(0.26, 24, 24, 0, Math.PI * 2, 0, Math.PI / 2);
      const m1 = new THREE.Mesh(halfGeo, mat);
      const m2 = new THREE.Mesh(halfGeo, mat);
      m2.rotation.x = Math.PI;
      this.lensMesh = new THREE.Group();
      this.lensMesh.add(m1, m2);
      this.lensMesh.scale.set(1, 0.6, 0.5);
    } else {
      this.lensMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.012, 24), mat);
    }
    g.add(this.lensMesh);
    // 框架
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x4a5563, metalness: 0.6, roughness: 0.4 });
    this.frame = new THREE.Mesh(new THREE.TorusGeometry(0.17, 0.012, 8, 32), frameMat);
    g.add(this.frame);
    this.stand = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.22, 8), frameMat);
    this.stand.position.y = -0.11;
    g.add(this.stand);
    const baseM = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.08, 0.02, 16), frameMat);
    baseM.position.y = -0.22;
    g.add(baseM);
    this._apply();
  }
  _apply() {
    const h = 0.22;
    this.lensMesh.position.y = h;
    this.frame.position.y = h;
    const f = this.params.focal;
    // 焦点标记 F
    if (!this.fMark) {
      this.fMark = [];
      for (const s of [-1, 1]) {
        const dot = new THREE.Mesh(new THREE.SphereGeometry(0.012, 10, 8),
          new THREE.MeshBasicMaterial({ color: 0xffd93d }));
        dot.userData.highlightable = false;
        this.group.add(dot); this.fMark.push({ dot, s });
      }
    }
    for (const { dot, s } of this.fMark) {
      dot.position.set(s * f * (this.subType === 'concave' ? 1 : 1), 0.22, 0);
      dot.visible = true;
    }
  }
  onParamsChanged() { this._apply(); }
  getReadouts() { return { 'f (m)': this.params.focal, '类型': this.subType === 'convex' ? '凸透镜' : '凹透镜' }; }
}

/* ---------- 平面镜 / 曲面镜 ---------- */
export class Mirror extends OpticalElement {
  constructor(zone, subType) {
    super(zone, subType === 'plane' ? getDef('mirror-plane') : getDef('mirror-curved'));
    this.isMirror = true; this.subType = subType;
    this.initParams(); this.build();
  }
  paramSchema() {
    if (this.subType === 'plane') return [];
    return [{ key: 'R', label: '曲率半径 R (m)', min: 0.2, max: 1.2, step: 0.02, value: 0.6 }];
  }
  build() {
    const g = this.group;
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x39424e, metalness: 0.5, roughness: 0.5 });
    if (this.subType === 'plane') {
      const glass = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.44, 0.3),
        new THREE.MeshStandardMaterial({ color: 0xcfe4ff, metalness: 0.95, roughness: 0.05 }));
      glass.position.y = 0.24;
      g.add(glass);
      const back = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.44, 0.3), frameMat);
      back.position.set(-0.018, 0.24, 0);
      g.add(back);
    } else {
      const concave = this.subType === 'concave';
      const geo = new THREE.SphereGeometry(0.55, 24, 20, 0, Math.PI * 2, 0, Math.PI / 2.6);
      const glass = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
        color: 0xd6e8ff, metalness: 0.95, roughness: 0.06, side: THREE.DoubleSide,
      }));
      glass.rotation.z = concave ? Math.PI / 2 : -Math.PI / 2;
      glass.rotation.y = Math.PI / 2;
      glass.scale.set(0.55, 0.4, 0.62);
      glass.position.y = 0.22;
      g.add(glass);
    }
    const baseM = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.08, 0.02, 16), frameMat);
    baseM.position.y = 0.01;
    g.add(baseM);
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.22, 8), frameMat);
    pole.position.y = 0.11;
    g.add(pole);
    this._apply();
  }
  _apply() {}
  onParamsChanged() { this._apply(); }
  getReadouts() {
    return this.subType === 'plane' ? { '类型': '平面镜' } : { 'R (m)': this.params.R, 'f (m)': +(this.params.R / 2).toFixed(3) };
  }
}

/* ---------- 光屏 ---------- */
export class Screen extends OpticalElement {
  constructor(zone) { super(zone, getDef('screen')); this.subType = 'screen'; this.initParams(); this.build(); }
  paramSchema() { return []; }
  build() {
    const g = this.group;
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x4a5563, roughness: 0.6 });
    // 白屏
    this.canvasMesh = new THREE.Mesh(new THREE.PlaneGeometry(0.84, 0.5),
      new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.9, side: THREE.DoubleSide }));
    this.canvasMesh.position.y = 0.3;
    this.canvasMesh.rotation.y = Math.PI / 2;
    g.add(this.canvasMesh);
    // 网格
    const grid = new THREE.GridHelper(0.5, 5, 0xbbbbbb, 0xcccccc);
    grid.rotation.z = Math.PI / 2;
    grid.position.y = 0.3;
    g.add(grid);
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.56, 8), frameMat);
    pole.position.y = 0.28;
    g.add(pole);
    const baseM = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.08, 0.02, 16), frameMat);
    baseM.position.y = 0.01;
    g.add(baseM);
    this.hitMarks = [];
  }
  // 被光照射时打点（由光源 trace 调用）
  litAt(y, color) {
    this.hitMarks.push({ y, color: color.getHex(), life: 1 });
  }
  update(dt) {
    // 更新光斑：用小发光面片表示
    for (const m of this.hitMarks) m.life -= dt * 0.8;
    this.hitMarks = this.hitMarks.filter(m => m.life > 0);
    if (!this.spotGroup) {
      this.spotGroup = new THREE.Group();
      this.group.add(this.spotGroup);
    }
    while (this.spotGroup.children.length) { const c = this.spotGroup.children.pop(); c.geometry?.dispose(); c.material?.dispose(); }
    for (const m of this.hitMarks) {
      const sp = new THREE.Mesh(new THREE.CircleGeometry(0.018, 12),
        new THREE.MeshBasicMaterial({ color: m.color, transparent: true, opacity: Math.min(m.life, 1) }));
      sp.rotation.y = Math.PI / 2;
      sp.position.set(0.005, 0.3 + m.y, 0);
      sp.userData.highlightable = false;
      this.spotGroup.add(sp);
    }
  }
  getReadouts() { return { '光斑数': this.hitMarks.length }; }
}

/* ---------- 三棱镜 ---------- */
export class Prism extends OpticalElement {
  constructor(zone) { super(zone, getDef('prism')); this.subType = 'prism'; this.initParams(); this.build(); }
  paramSchema() {
    return [{ key: 'angle', label: '顶角 A (°)', min: 20, max: 70, step: 1, value: 45 }];
  }
  build() {
    const g = this.group;
    const shape = new THREE.Shape();
    const A = this.params.angle * Math.PI / 180;
    const r = 0.11;
    shape.moveTo(0, Math.tan(A / 2) * r);
    shape.lineTo(r, 0);
    shape.lineTo(0, -Math.tan(A / 2) * r);
    shape.closePath();
    const geo = new THREE.ExtrudeGeometry(shape, { depth: 0.16, bevelEnabled: false });
    geo.rotateY(Math.PI / 2);
    geo.rotateX(Math.PI / 2);
    this.prismMesh = new THREE.Mesh(geo, new THREE.MeshPhysicalMaterial({
      color: 0xbfe0ff, transparent: true, opacity: 0.38, roughness: 0.02, transmission: 0.85, thickness: 0.05, ior: 1.52,
    }));
    this.prismMesh.position.y = 0.22;
    g.add(this.prismMesh);
    const baseM = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.08, 0.02, 16),
      new THREE.MeshStandardMaterial({ color: 0x4a5563 }));
    baseM.position.y = 0.01;
    g.add(baseM);
  }
  onParamsChanged() { this.buildChildren(); }
  buildChildren() {
    // 重建棱镜形状
    this.group.remove(this.prismMesh);
    this.prismMesh.geometry.dispose();
    const shape = new THREE.Shape();
    const A = this.params.angle * Math.PI / 180;
    const r = 0.11;
    shape.moveTo(0, Math.tan(A / 2) * r);
    shape.lineTo(r, 0);
    shape.lineTo(0, -Math.tan(A / 2) * r);
    shape.closePath();
    const geo = new THREE.ExtrudeGeometry(shape, { depth: 0.16, bevelEnabled: false });
    geo.rotateY(Math.PI / 2);
    geo.rotateX(Math.PI / 2);
    this.prismMesh = new THREE.Mesh(geo, this.prismMesh.material);
    this.prismMesh.position.y = 0.22;
    this.group.add(this.prismMesh);
  }
  getReadouts() { return { '顶角 A (°)': this.params.angle }; }
}

const DEFS = [
  { id: 'light', name: '光源', icon: '🔦', desc: '激光 / 点光源 / 五线束', make: (z) => new LightSource(z) },
  { id: 'lens-convex', name: '凸透镜', icon: '🔍', desc: '会聚透镜，f 可调', make: (z) => new Lens(z, 'convex') },
  { id: 'lens-concave', name: '凹透镜', icon: '🕳', desc: '发散透镜', make: (z) => new Lens(z, 'concave') },
  { id: 'mirror-plane', name: '平面镜', icon: '🪞', desc: '反射定律', make: (z) => new Mirror(z, 'plane') },
  { id: 'mirror-curved', name: '球面镜', icon: '🥣', desc: '凹/凸面镜成像', make: (z) => new Mirror(z, 'concave') },
  { id: 'screen', name: '光屏', icon: '⬜', desc: '承接实像/光斑', make: (z) => new Screen(z) },
  { id: 'prism', name: '三棱镜', icon: '🔺', desc: '色散彩虹', make: (z) => new Prism(z) },
];
DEFS.forEach(d => registerEquipment('optics', d));
function getDef(id) { return DEFS.find(d => d.id === id); }
