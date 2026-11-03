// 相机机位：轨道控制 + 分区视角预设 + 平滑飞行
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ZONES, BENCH } from './LabScene.js';

export class CameraRig {
  constructor(camera, dom) {
    this.camera = camera;
    this.controls = new OrbitControls(camera, dom);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.03;
    this.controls.minDistance = 1.2;
    this.controls.maxDistance = 26;
    this.controls.target.set(0, 0.9, 0);
    this._fly = null;
    this.VIEWS = {
      overview:   { pos: [0, 8.5, 10.5],  tgt: [0, 0.8, 0] },
      mechanics:  { pos: [ZONES.mechanics.x, 2.6, ZONES.mechanics.z + 3.4],  tgt: [ZONES.mechanics.x, BENCH.H + 0.25, ZONES.mechanics.z] },
      optics:     { pos: [ZONES.optics.x, 1.9, ZONES.optics.z + 2.8],       tgt: [ZONES.optics.x, BENCH.H + 0.2, ZONES.optics.z] },
      electricity:{ pos: [ZONES.electricity.x, 2.5, ZONES.electricity.z + 3.2], tgt: [ZONES.electricity.x, BENCH.H + 0.25, ZONES.electricity.z] },
      waves:      { pos: [ZONES.waves.x, 2.9, ZONES.waves.z + 2.4],         tgt: [ZONES.waves.x, BENCH.H + 0.15, ZONES.waves.z] },
      thermal:    { pos: [ZONES.thermal.x, 2.4, ZONES.thermal.z + 2.8],     tgt: [ZONES.thermal.x, BENCH.H + 0.2, ZONES.thermal.z] },
    };
    this.setView('overview', true);
  }

  setView(name, immediate = false) {
    const v = this.VIEWS[name] || this.VIEWS.overview;
    const pos = new THREE.Vector3(...v.pos), tgt = new THREE.Vector3(...v.tgt);
    if (immediate) {
      this.camera.position.copy(pos);
      this.controls.target.copy(tgt);
      this.controls.update();
    } else {
      this._fly = { t: 0, from: this.camera.position.clone(), to: pos, fromT: this.controls.target.clone(), toT: tgt };
    }
  }

  // 聚焦某设备
  focusTo(position) {
    const dir = new THREE.Vector3().subVectors(this.camera.position, this.controls.target).normalize();
    const dist = 2.2;
    const pos = position.clone().add(dir.multiplyScalar(dist));
    pos.y = Math.max(position.y + 0.8, 1.2);
    this._fly = { t: 0, from: this.camera.position.clone(), to: pos, fromT: this.controls.target.clone(), toT: position.clone() };
  }

  update(dt) {
    if (this._fly) {
      this._fly.t += dt * 1.6;
      const k = Math.min(this._fly.t, 1);
      const e = k < 0.5 ? 4 * k ** 3 : 1 - (-2 * k + 2) ** 3 / 2; // easeInOutCubic
      this.camera.position.lerpVectors(this._fly.from, this._fly.to, e);
      this.controls.target.lerpVectors(this._fly.fromT, this._fly.toT, e);
      if (k >= 1) this._fly = null;
    }
    this.controls.update();
  }
}
