// 力学设备：单摆、牛顿摆、斜面滑块、平抛发射器、弹簧振子
import * as THREE from 'three';
import { EquipmentBase } from '../EquipmentBase.js';
import { registerCategory, registerEquipment } from '../registry.js';

registerCategory('mechanics', '力学');

/* ============ 单摆：T = 2π√(L/g)，解析积分（RK4）保证精度 ============ */
export class Pendulum extends EquipmentBase {
  constructor(zone) { super(zone, getDef('pendulum')); this.initParams(); this.build(); }
  paramSchema() {
    return [
      { key: 'L', label: '摆长 L (m)', min: 0.3, max: 2.0, step: 0.01, value: 1.0 },
      { key: 'theta0', label: '初始摆角 θ₀ (°)', min: 5, max: 60, step: 1, value: 15 },
      { key: 'g', label: '重力 g (m/s²)', min: 1.6, max: 24.8, step: 0.1, value: 9.8 },
      { key: 'damping', label: '空气阻尼', min: 0, max: 0.5, step: 0.01, value: 0 },
    ];
  }
  build() {
    const g = this.group;
    // 支架
    const standMat = new THREE.MeshStandardMaterial({ color: 0x8892a0, metalness: 0.7, roughness: 0.35 });
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 1.3, 10), standMat);
    pole.position.set(-0.35, 0.65, 0); pole.castShadow = true; g.add(pole);
    const base = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.05, 0.34), standMat);
    base.position.set(-0.35, 0.025, 0); base.castShadow = true; g.add(base);
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.72, 0.03, 0.03), standMat);
    arm.position.set(0, 1.3, 0); g.add(arm);
    // 摆线 + 摆球
    this.string = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 1, 6),
      new THREE.MeshStandardMaterial({ color: 0xcccccc }));
    g.add(this.string);
    this.bob = new THREE.Mesh(new THREE.SphereGeometry(0.055, 20, 16),
      new THREE.MeshStandardMaterial({ color: 0xd9a441, metalness: 0.75, roughness: 0.3 }));
    this.bob.castShadow = true;
    g.add(this.bob);
    // 角度盘
    this.protractor = new THREE.Mesh(new THREE.RingGeometry(0.13, 0.155, 32, 1, Math.PI / 2, Math.PI),
      new THREE.MeshBasicMaterial({ color: 0x9fe8ff, transparent: true, opacity: 0.55, side: THREE.DoubleSide }));
    this.protractor.position.set(0, 1.3, 0.006);
    g.add(this.protractor);
    this.state.t = 0;
    this.reset();
    this.onParamsChanged();
  }
  reset() {
    this.state.theta = this.params.theta0 * Math.PI / 180;
    this.state.omega = 0;
    this.state.t = 0;
    this.state.T = null;
    this.state.lastZero = null;
    this.state.zeroCount = 0;
    this._apply();
  }
  onParamsChanged() { this.reset(); }
  update(dt) {
    const { L, g, damping } = this.params;
    const s = this.state;
    s.t += dt;
    // RK4 for θ'' = -(g/L)sinθ - c·θ'
    const acc = (th, om) => -(g / L) * Math.sin(th) - damping * om;
    const k1t = s.omega, k1o = acc(s.theta, s.omega);
    const k2t = s.omega + k1o * dt / 2, k2o = acc(s.theta + k1t * dt / 2, s.omega + k1o * dt / 2);
    const k3t = s.omega + k2o * dt / 2, k3o = acc(s.theta + k2t * dt / 2, s.omega + k2o * dt / 2);
    const k4t = s.omega + k3o * dt, k4o = acc(s.theta + k3t * dt, s.omega + k3o * dt);
    const prevTheta = s.theta;
    s.theta += dt / 6 * (k1t + 2 * k2t + 2 * k3t + k4t);
    s.omega += dt / 6 * (k1o + 2 * k2o + 2 * k3o + k4o);
    // 过零检测测周期：同向过零（θ 由负转正）间隔 = 一个完整周期 T
    if (prevTheta < 0 && s.theta >= 0) {
      s.zeroCount++;
      if (s.zeroCount === 1) {
        s.lastZero = s.t;
      } else {
        s.T = s.t - s.lastZero;
        s.lastZero = s.t;
      }
    }
    this._apply();
  }
  _apply() {
    const { L } = this.params, th = this.state.theta;
    this.string.scale.y = L;
    this.string.position.set(Math.sin(th) * L / 2, 1.3 - Math.cos(th) * L / 2, 0);
    this.string.rotation.z = -th;
    this.bob.position.set(Math.sin(th) * L, 1.3 - Math.cos(th) * L, 0);
  }
  getReadouts() {
    return {
      't (s)': +this.state.t.toFixed(2),
      'θ (°)': +(this.state.theta * 180 / Math.PI).toFixed(1),
      'ω (rad/s)': +this.state.omega.toFixed(3),
      'T 测量 (s)': this.state.T === null ? '—' : +this.state.T.toFixed(3),
      'T 理论 (s)': +(2 * Math.PI * Math.sqrt(this.params.L / this.params.g)).toFixed(3),
    };
  }
}

/* ============ 牛顿摆（5 球，动量/能量传递演示） ============ */
export class NewtonsCradle extends EquipmentBase {
  constructor(zone) { super(zone, getDef('newtons-cradle')); this.initParams(); this.build(); }
  paramSchema() {
    return [
      { key: 'n', label: '拉起球数', min: 1, max: 4, step: 1, value: 1 },
      { key: 'angle', label: '拉起角度 (°)', min: 10, max: 60, step: 1, value: 40 },
      { key: 'e', label: '恢复系数 e', min: 0.5, max: 1.0, step: 0.01, value: 0.98 },
    ];
  }
  build() {
    const g = this.group;
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x506070, metalness: 0.6, roughness: 0.4 });
    const base = new THREE.Mesh(new THREE.BoxGeometry(0.72, 0.04, 0.3), frameMat);
    base.position.y = 0.02; g.add(base);
    for (const [x, z] of [[-0.32, -0.12], [0.32, -0.12], [-0.32, 0.12], [0.32, 0.12]]) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.5, 0.035), frameMat);
      leg.position.set(x, 0.27, z); leg.castShadow = true; g.add(leg);
    }
    const top = new THREE.Mesh(new THREE.BoxGeometry(0.72, 0.035, 0.3), frameMat);
    top.position.y = 0.54; g.add(top);
    // 5 个球
    this.balls = [];
    this.strings = [];
    const R = 0.05, gap = 2 * R;
    const ballMat = new THREE.MeshStandardMaterial({ color: 0xb8c4d0, metalness: 0.9, roughness: 0.15 });
    for (let i = 0; i < 5; i++) {
      const ball = new THREE.Mesh(new THREE.SphereGeometry(R, 24, 18), ballMat.clone());
      ball.castShadow = true;
      ball.userData.x0 = (i - 2) * gap;
      ball.userData.theta = 0; ball.userData.omega = 0; ball.userData.active = false;
      g.add(ball); this.balls.push(ball);
      const str = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.44, 6),
        new THREE.MeshStandardMaterial({ color: 0xbbb }));
      g.add(str); this.strings.push(str);
    }
    this.state.t = 0;
    this.reset();
  }
  reset() {
    const { n, angle } = this.params;
    const th = angle * Math.PI / 180;
    this.balls.forEach((b, i) => {
      b.userData.theta = 0; b.userData.omega = 0;
      b.userData.active = i < n;      // 左侧 n 个球被拉起
      b.userData.theta = b.userData.active ? th : 0;
      b.userData.side = -1;           // 拉向 -x
    });
    this.state.t = 0;
    this._apply();
  }
  onParamsChanged() { this.reset(); }
  update(dt) {
    const g = 9.8, Lc = 0.44, e = this.params.e;
    this.state.t += dt;
    // 球摆动（小角度近似 sinθ≈θ 误差可接受，这里用完整非线性）
    for (const b of this.balls) {
      if (Math.abs(b.userData.theta) > 1e-4 || Math.abs(b.userData.omega) > 1e-4) {
        const a = -(g / Lc) * Math.sin(b.userData.theta);
        b.userData.omega += a * dt;
        b.userData.theta += b.userData.omega * dt;
      }
    }
    // 碰撞检测：相邻球间距 < 2R 且相向运动 -> 交换速度（等质量弹性碰撞）
    for (let pass = 0; pass < 3; pass++) {
      for (let i = 0; i < 4; i++) {
        const a = this.balls[i], c = this.balls[i + 1];
        const xa = a.userData.x0 + Lc * Math.sin(a.userData.theta);
        const xc = c.userData.x0 + Lc * Math.sin(c.userData.theta);
        const va = a.userData.omega * Lc * Math.cos(a.userData.theta);
        const vc = c.userData.omega * Lc * Math.cos(c.userData.theta);
        if (xc - xa < 2 * 0.05 - 0.002 && va > vc) {
          const va2 = va * e, vc2 = vc * e;
          a.userData.omega = va2 / (Lc * Math.cos(a.userData.theta) || 1e-6);
          c.userData.omega = vc2 / (Lc * Math.cos(c.userData.theta) || 1e-6);
          // 弹性碰撞：交换速度
          const t = a.userData.omega; a.userData.omega = c.userData.omega * e; c.userData.omega = t * e;
        }
      }
    }
    this._apply();
  }
  _apply() {
    const Lc = 0.44;
    this.balls.forEach((b, i) => {
      const th = b.userData.theta;
      b.position.set(b.userData.x0 + Lc * Math.sin(th), 0.54 - Lc * Math.cos(th) - 0.05, 0);
      const str = this.strings[i];
      str.position.set(b.userData.x0 + Lc * Math.sin(th) / 2, 0.54 - Lc * Math.cos(th) / 2, 0);
      str.rotation.z = -th;
      str.scale.y = Lc;
    });
  }
  getReadouts() { return { 't (s)': +this.state.t.toFixed(2) }; }
}

/* ============ 斜面滑块：测摩擦因数 / 验证牛顿第二定律 ============ */
export class InclinePlane extends EquipmentBase {
  constructor(zone) { super(zone, getDef('incline')); this.initParams(); this.build(); }
  paramSchema() {
    return [
      { key: 'angle', label: '倾角 θ (°)', min: 5, max: 45, step: 0.5, value: 25 },
      { key: 'mu', label: '摩擦因数 μ', min: 0, max: 1.0, step: 0.01, value: 0.3 },
      { key: 'm', label: '滑块质量 (kg)', min: 0.2, max: 5, step: 0.1, value: 1.0 },
      { key: 'v0', label: '初速度 (m/s)', min: 0, max: 3, step: 0.1, value: 0 },
    ];
  }
  build() {
    const g = this.group;
    this.ramp = new THREE.Group();
    const rampMat = new THREE.MeshStandardMaterial({ color: 0x7d8a96, roughness: 0.6 });
    this.rampMesh = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.035, 0.32), rampMat);
    this.rampMesh.castShadow = true; this.rampMesh.receiveShadow = true;
    this.ramp.add(this.rampMesh);
    g.add(this.ramp);
    this.block = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.09, 0.16),
      new THREE.MeshStandardMaterial({ color: 0xe07b39, roughness: 0.5 }));
    this.block.castShadow = true;
    g.add(this.block);
    // 力矢量箭头
    this.arrowW = makeArrow(0x4cc3ff); // 重力
    this.arrowF = makeArrow(0xff6b6b); // 摩擦力
    this.arrowN = makeArrow(0x5dd39e); // 支持力
    g.add(this.arrowW, this.arrowN, this.arrowF);
    this.state = { s: 0, v: this.params.v0, t: 0, done: false };
    this.onParamsChanged();
  }
  reset() { this.state = { s: 0, v: this.params.v0, t: 0, done: false }; this._apply(); }
  onParamsChanged() {
    const th = this.params.angle * Math.PI / 180;
    const L = 1.15;
    this.ramp.rotation.z = th;
    this.ramp.position.set(-Math.cos(th) * L / 2 + 0.1, Math.sin(th) * L / 2 + 0.02, 0);
    this.reset();
  }
  update(dt) {
    const { angle, mu, m } = this.params;
    const th = angle * Math.PI / 180;
    const s = this.state;
    if (s.done) return;
    s.t += dt;
    const a = 9.8 * (Math.sin(th) - mu * Math.cos(th));
    s.v += a * dt;
    s.s += s.v * dt;
    if (s.s >= 1.0 || s.v < 0 && s.s <= 0) { s.done = true; if (s.s < 0) { s.s = 0; s.v = 0; } }
    this._apply();
  }
  _apply() {
    const th = this.params.angle * Math.PI / 180;
    const s = this.state;
    const L = 1.15;
    // 沿斜面位置 -> 世界坐标（group 局部）
    const px = 0.1 - Math.cos(th) * (L - 0.06 - s.s);
    const py = Math.sin(th) * (L - 0.06 - s.s) + 0.02 + 0.018 / Math.cos(th) + 0.045;
    this.block.position.set(px, py, 0);
    this.block.rotation.z = th;
    // 力箭头
    const { m, mu } = this.params;
    const W = m * 9.8;
    setArrow(this.arrowW, this.block.position, new THREE.Vector3(0, -W * 0.045, 0), 'G');
    const N = W * Math.cos(th);
    setArrow(this.arrowN, this.block.position, new THREE.Vector3(Math.sin(th), Math.cos(th), 0).multiplyScalar(N * 0.045), 'N');
    const moving = !this.state.done;
    const F = moving ? mu * N : Math.min(W * Math.sin(th), mu * N);
    const fdir = new THREE.Vector3(-Math.cos(th), Math.sin(th), 0);
    setArrow(this.arrowF, this.block.position, fdir.multiplyScalar(F * 0.045 * (moving ? 1 : 0)), 'f');
  }
  getReadouts() {
    const th = this.params.angle * Math.PI / 180;
    const a = 9.8 * (Math.sin(th) - this.params.mu * Math.cos(th));
    return {
      't (s)': +this.state.t.toFixed(2),
      's (m)': +this.state.s.toFixed(3),
      'v (m/s)': +this.state.v.toFixed(3),
      'a (m/s²)': +a.toFixed(3),
    };
  }
}

/* ============ 平抛/斜抛发射器 ============ */
export class ProjectileLauncher extends EquipmentBase {
  constructor(zone) { super(zone, getDef('projectile')); this.initParams(); this.build(); }
  paramSchema() {
    return [
      { key: 'v0', label: '初速度 v₀ (m/s)', min: 0.5, max: 6, step: 0.1, value: 2.5 },
      { key: 'angle', label: '仰角 (°)', min: 0, max: 80, step: 1, value: 0 },
      { key: 'h', label: '发射高度 (m)', min: 0.15, max: 1.2, step: 0.05, value: 0.6 },
      { key: 'g', label: '重力 g (m/s²)', min: 1.6, max: 24.8, step: 0.1, value: 9.8 },
    ];
  }
  build() {
    const g = this.group;
    this.tower = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: 0x607080, metalness: 0.5, roughness: 0.4 });
    const column = new THREE.Mesh(new THREE.BoxGeometry(0.09, 1, 0.09), mat);
    column.position.y = 0.5; column.castShadow = true;
    this.tower.add(column);
    g.add(this.tower);
    // 发射筒
    this.barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.045, 0.3, 12),
      new THREE.MeshStandardMaterial({ color: 0x39485a, metalness: 0.6, roughness: 0.35 }));
    this.barrel.castShadow = true;
    g.add(this.barrel);
    // 小球
    this.ball = new THREE.Mesh(new THREE.SphereGeometry(0.035, 18, 14),
      new THREE.MeshStandardMaterial({ color: 0xff6b6b, roughness: 0.4 }));
    this.ball.castShadow = true;
    g.add(this.ball);
    // 轨迹线
    this.trajGeo = new THREE.BufferGeometry();
    this.traj = new THREE.Line(this.trajGeo, new THREE.LineBasicMaterial({ color: 0xffd93d }));
    this.traj.userData.highlightable = false;
    g.add(this.traj);
    this.trajPoints = [];
    this.onParamsChanged();
  }
  reset() {
    this.state = { t: 0, flying: false, x: 0, y: 0, vx: 0, vy: 0 };
    this.trajPoints = [];
    this._apply();
  }
  onParamsChanged() { this.reset(); }
  launch() {
    const { v0, angle } = this.params;
    const th = angle * Math.PI / 180;
    this.state = { t: 0, flying: true, x: 0, y: this.params.h, vx: v0 * Math.cos(th), vy: v0 * Math.sin(th) };
    this.trajPoints = [];
  }
  update(dt) {
    const s = this.state;
    if (!s.flying) return;
    s.t += dt;
    s.x += s.vx * dt;
    s.y += s.vy * dt;
    s.vy -= this.params.g * dt;
    if (s.y <= 0) { s.y = 0; s.flying = false; }
    this.trajPoints.push(new THREE.Vector3(s.x, s.y + BENCH_TOP(), 0));
    if (this.trajPoints.length > 3000) this.trajPoints.shift();
    this.trajGeo.setFromPoints(this.trajPoints);
    this._apply();
  }
  _apply() {
    const s = this.state;
    const th = this.params.angle * Math.PI / 180;
    this.tower.scale.y = this.params.h;
    this.barrel.position.set(Math.cos(th) * 0.16, this.params.h, 0);
    this.barrel.rotation.z = -Math.PI / 2 + th;
    if (!s.flying && s.t === 0) {
      this.ball.position.set(Math.cos(th) * 0.28, this.params.h, 0);
    } else {
      this.ball.position.set(s.x + Math.cos(th) * 0.28, s.y + BENCH_TOP(), 0);
    }
  }
  getReadouts() {
    const s = this.state;
    const range = s.t > 0 ? (s.flying ? s.x : s.x) : 0;
    return {
      't (s)': +s.t.toFixed(3),
      'x (m)': +s.x.toFixed(3),
      'y (m)': +s.y.toFixed(3),
      'vx (m/s)': +s.vx.toFixed(2),
      'vy (m/s)': +s.vy.toFixed(2),
    };
  }
  actions() { return [{ key: 'launch', label: '🚀 发射' }]; }
  doAction(key) { if (key === 'launch') { this.reset(); this.launch(); } }
}

/* ============ 弹簧振子：T = 2π√(m/k) ============ */
export class SpringOscillator extends EquipmentBase {
  constructor(zone) { super(zone, getDef('spring')); this.initParams(); this.build(); }
  paramSchema() {
    return [
      { key: 'k', label: '劲度系数 k (N/m)', min: 5, max: 100, step: 1, value: 25 },
      { key: 'm', label: '质量 m (kg)', min: 0.1, max: 2, step: 0.05, value: 0.5 },
      { key: 'A', label: '振幅 A (m)', min: 0.05, max: 0.3, step: 0.01, value: 0.15 },
      { key: 'damping', label: '阻尼', min: 0, max: 2, step: 0.05, value: 0 },
    ];
  }
  build() {
    const g = this.group;
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x7a8694, metalness: 0.6, roughness: 0.4 });
    const bar = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.035, 0.3), frameMat);
    bar.position.y = 0.9; g.add(bar);
    for (const [x, z] of [[-0.42, -0.13], [0.42, -0.13], [-0.42, 0.13], [0.42, 0.13]]) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.9, 0.03), frameMat);
      leg.position.set(x, 0.45, z); g.add(leg);
    }
    // 弹簧（螺旋线，动态重建）
    this.springMesh = new THREE.Mesh(new THREE.TubeGeometry(this._springCurve(0.4), 64, 0.012, 6),
      new THREE.MeshStandardMaterial({ color: 0xaebbcc, metalness: 0.8, roughness: 0.3 }));
    g.add(this.springMesh);
    // 砝码盘
    this.weight = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.1, 20),
      new THREE.MeshStandardMaterial({ color: 0x4a5a6a, metalness: 0.7, roughness: 0.35 }));
    this.weight.castShadow = true;
    g.add(this.weight);
    this.reset();
  }
  _springCurve(len) {
    const pts = [], coils = 8, r = 0.05, n = 120;
    for (let i = 0; i <= n; i++) {
      const t = i / n;
      pts.push(new THREE.Vector3(Math.cos(t * coils * Math.PI * 2) * r, -t * len, Math.sin(t * coils * Math.PI * 2) * r));
    }
    return new THREE.CatmullRomCurve3(pts);
  }
  reset() {
    this.state = { t: 0, x: this.params.A, v: 0, T: null, lastExt: null, extCount: 0 };
    this._apply();
  }
  update(dt) {
    const { k, m, damping, A } = this.params;
    const s = this.state;
    s.t += dt;
    const omega2 = k / m;
    const a = -omega2 * s.x - damping * s.v;
    const vx = s.v + a * dt / 2;
    s.x += vx * dt;
    s.v = vx + (-omega2 * s.x - damping * vx) * dt / 2;
    // 周期检测：速度过零 + 同向
    if (s.v > 0 && this._pv <= 0) {
      s.extCount++;
      if (s.lastExt !== null && s.extCount >= 2) s.T = s.t - s.lastExt;
      s.lastExt = s.t;
    }
    this._pv = s.v;
    this._apply();
  }
  _apply() {
    const x = this.state.x;
    const natural = 0.4;
    const len = natural + x;
    this.springMesh.geometry.dispose();
    this.springMesh.geometry = new THREE.TubeGeometry(this._springCurve(len), 80, 0.012, 6);
    this.weight.position.set(0, 0.9 - len - 0.05, 0);
  }
  getReadouts() {
    const s = this.state;
    return {
      't (s)': +s.t.toFixed(2),
      'x (m)': +s.x.toFixed(3),
      'v (m/s)': +s.v.toFixed(3),
      'T 测量 (s)': s.T === null ? '—' : +s.T.toFixed(3),
      'T 理论 (s)': +(2 * Math.PI * Math.sqrt(this.params.m / this.params.k)).toFixed(3),
    };
  }
}

/* ---- 工具函数 ---- */
function BENCH_TOP() { return 0; } // 设备 group 已位于台面高度，内部相对台面
function makeArrow(color) {
  const dir = new THREE.Vector3(0, 1, 0);
  const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), 0.2, color, 0.07, 0.04);
  arrow.userData.highlightable = false;
  return arrow;
}
function setArrow(arrow, origin, vec) {
  arrow.position.copy(origin);
  arrow.setDirection(vec.clone().normalize());
  arrow.setLength(Math.max(vec.length(), 0.02), 0.05, 0.03);
}

const DEFS = [
  { id: 'pendulum', name: '单摆装置', icon: '🧭', desc: '测重力加速度 T=2π√(L/g)', make: Pendulum },
  { id: 'newtons-cradle', name: '牛顿摆', icon: '⚖️', desc: '动量守恒演示', make: NewtonsCradle },
  { id: 'incline', name: '斜面滑块', icon: '📐', desc: '摩擦因数与牛顿定律', make: InclinePlane },
  { id: 'projectile', name: '平抛发射器', icon: '🎯', desc: '抛体运动轨迹研究', make: ProjectileLauncher },
  { id: 'spring', name: '弹簧振子', icon: '🌀', desc: '简谐振动 T=2π√(m/k)', make: SpringOscillator },
];
DEFS.forEach(d => registerEquipment('mechanics', d));
function getDef(id) { return DEFS.find(d => d.id === id); }
