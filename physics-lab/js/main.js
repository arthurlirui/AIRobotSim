// 三维物理实验室：应用总装
import * as THREE from 'three';
import { LabScene, ZONES, BENCH } from './scene/LabScene.js';
import { CameraRig } from './scene/CameraRig.js';
import { EventBus, EV } from './core/EventBus.js';
import { UI } from './ui/UI.js';
import { Interaction } from './utils/selection.js';
import { serializeScene, deserializeScene } from './utils/serializer.js';
import { getEquipmentDef } from './equipment/registry.js';

// 设备模块（自注册）
import './equipment/mechanics/index.js';
import './equipment/optics/index.js';
import { traceRays } from './equipment/optics/index.js';
import './equipment/electricity/index.js';
import './equipment/waves/index.js';

class App {
  constructor() {
    this.equipments = [];
    this.playing = true;
    this.speed = 1;
    this.simTime = 0;
    this._initRenderer();
    this.labScene = new LabScene();
    this.scene = this.labScene.scene;
    this.rig = new CameraRig(camera, renderer.domElement);
    this.interaction = new Interaction(this.camera, renderer.domElement, this._labAdapter());
    this.ui = new UI(this);
    this._bindToolbar();
    this._bindInteraction();
    this._bindResize();
    this._setupDefaultScene();
    this._clock = new THREE.Clock();
    this._fpsCount = 0; this._fpsTime = 0;
    this._loop();
  }

  _initRenderer() {
    const canvas = document.getElementById('scene-canvas');
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(innerWidth, innerHeight);
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.05, 100);
  }

  _labAdapter() {
    const self = this;
    return {
      get scene() { return self.scene; },
      get equipments() { return self.equipments; },
      ZONES,
      equipGroups: () => self.equipments.map(e => e.group),
      toast: (m, k) => self.ui?.toast(m, k),
    };
  }

  /* ---------- 设备生命周期 ---------- */
  spawnEquipment(typeId, zoneKey) {
    const def = getEquipmentDef(typeId);
    if (!def) { console.warn('unknown equipment', typeId); return null; }
    const zone = zoneKey || (['light', 'lens-convex', 'lens-concave', 'mirror-plane', 'mirror-curved', 'screen', 'prism'].includes(typeId) ? 'optics'
      : def.cat === 'measurement' ? 'mechanics' : def.cat);
    const eq = def.make.prototype ? new def.make(zone) : def.make(zone);
    // 避免重叠：找台面空位
    const zoneDef = ZONES[zone];
    const used = this.equipments.filter(e => e.zoneKey === zone).map(e => [e.group.position.x, e.group.position.z]);
    let px = 0, pz = 0, bestD = -1;
    for (const [cx, cz] of [[-1.1, 0], [-0.55, 0.35], [0, 0], [0.55, -0.35], [1.1, 0], [-0.55, -0.35], [0.55, 0.35], [0, 0.45], [0, -0.45]]) {
      const wx = zoneDef.x + cx, wz = zoneDef.z + cz;
      let d = Infinity;
      for (const [ux, uz] of used) d = Math.min(d, Math.hypot(ux - wx, uz - wz));
      if (d > bestD) { bestD = d; px = cx; pz = cz; }
    }
    eq.group.position.set(zoneDef.x + px, BENCH.H, zoneDef.z + pz);
    this.scene.add(eq.group);
    this.equipments.push(eq);
    EventBus.emit(EV.EQUIPMENT_ADDED, eq);
    this.ui.setStatus(`已放置：${def.name}（${{ mechanics: '力学区', optics: '光学区', electricity: '电学区', waves: '波学区', thermal: '热学区' }[zone]}）`);
    return eq;
  }

  removeEquipment(eq) {
    const i = this.equipments.indexOf(eq);
    if (i >= 0) this.equipments.splice(i, 1);
    this.scene.remove(eq.group);
    eq.group.traverse(c => { c.geometry?.dispose(); if (c.material) (Array.isArray(c.material) ? c.material : [c.material]).forEach(m => m.dispose()); });
    if (this.interaction.selected === eq) { this.interaction.select(null); }
    EventBus.emit(EV.EQUIPMENT_REMOVED, eq);
  }

  setupGuide(guide) {
    // 清理同区旧设备，布置实验器材
    for (const s of guide.setup) {
      const exists = this.equipments.find(e => e.def.id === s.type);
      if (exists) this.removeEquipment(exists);
    }
    const created = guide.setup.map(s => {
      const eq = this.spawnEquipment(s.type, guide.zone === 'measurement' ? 'mechanics' : guide.zone);
      if (eq && s.params) { Object.assign(eq.params, s.params); eq.onParamsChanged(); }
      return eq;
    });
    this.cameraToZone(guide.zone);
    return created;
  }

  cameraToZone(zone) { this.rig.setView(zone); }

  /* ---------- 光线追踪（每帧对光学区光源） ---------- */
  _traceOptics() {
    const opticsEqs = this.equipments.filter(e => e.zoneKey === 'optics');
    const sources = opticsEqs.filter(e => e.isSource);
    const elements = opticsEqs.filter(e => e.isLens || e.isMirror || e.subType === 'screen' || e.subType === 'prism');
    // zoneOrigin：光学台中心
    const zo = { x: ZONES.optics.x, z: ZONES.optics.z };
    for (const el of elements) el.zoneOrigin = zo;
    for (const src of sources) {
      src.zoneOrigin = zo;
      // 清空光屏光斑标记
      for (const el of elements) if (el.subType === 'screen') { }
      const segs = traceRays(src, elements);
      src.setSegments(segs);
    }
  }

  /* ---------- 工具栏 ---------- */
  _bindToolbar() {
    document.getElementById('btn-play').addEventListener('click', () => this.togglePlay());
    document.getElementById('speed-select').addEventListener('change', e => { this.speed = +e.target.value; });
    document.getElementById('btn-reset').addEventListener('click', () => this.resetAll());
    document.querySelectorAll('#view-presets .tb-btn').forEach(b => b.addEventListener('click', () => this.rig.setView(b.dataset.view)));
    document.getElementById('btn-dark').addEventListener('click', () => {
      this.labScene.setDarkMode(!this.labScene.darkMode);
      this.ui.setDark(this.labScene.darkMode);
    });
    document.getElementById('btn-save').addEventListener('click', () => this.saveScene());
    document.getElementById('btn-load').addEventListener('click', () => this.loadScene());
  }

  togglePlay() {
    this.playing = !this.playing;
    this.ui.setPlaying(this.playing);
  }

  resetAll() {
    for (const eq of this.equipments) eq.reset();
    this.simTime = 0;
    this.ui.toast('已重置全部设备');
  }

  saveScene() {
    const json = serializeScene(this.equipments, { note: '物理实验室' });
    localStorage.setItem('physics-lab-scene', json);
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([json], { type: 'application/json' }));
    a.download = `物理实验室场景_${new Date().toISOString().slice(0, 10)}.json`;
    a.click();
    this.ui.toast('场景已保存（本地 + 文件下载）');
  }

  loadScene() {
    const json = localStorage.getItem('physics-lab-scene');
    if (!json) { this.ui.toast('没有已保存的场景，请先保存', 'warn'); return; }
    try {
      for (const eq of [...this.equipments]) this.removeEquipment(eq);
      const { equipments } = deserializeScene(json, this.equipments, (type, zone) => this.spawnEquipment(type, zone));
      this.ui.toast(`已加载场景（${equipments.length} 个设备）`);
    } catch (err) {
      console.error(err);
      this.ui.toast('加载失败：' + err.message, 'err');
    }
  }

  _bindInteraction() {
    this.interaction.onSelect = eq => { eq ? this.ui.showProps(eq) : this.ui.clearProps(); };
    this.interaction.onDelete = eq => this.removeEquipment(eq);
    this.interaction.onTogglePlay = () => this.togglePlay();
    this.interaction.onRequestDeleteWire = wire => {
      const bench = this.equipments.find(e => e.isCircuitHost && e.wires.includes(wire));
      if (bench) { bench.disconnectWire(wire); this.ui.toast('已拆除导线'); }
    };
  }

  _bindResize() {
    addEventListener('resize', () => {
      camera.aspect = innerWidth / innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(innerWidth, innerHeight);
    });
  }

  /* ---------- 初始场景 ---------- */
  _setupDefaultScene() {
    // 每个区放一个代表设备，用户可自由增删
    this.spawnEquipment('pendulum', 'mechanics');
    this.spawnEquipment('newtons-cradle', 'mechanics');
    this.spawnEquipment('light', 'optics');
    this.spawnEquipment('lens-convex', 'optics');
    this.spawnEquipment('screen', 'optics');
    this.spawnEquipment('circuit-bench', 'electricity');
    this.spawnEquipment('ripple-tank', 'waves');
    this.spawnEquipment('calorimeter', 'thermal');
  }

  /* ---------- 主循环 ---------- */
  // 双保险驱动：优先 rAF；若被环境节流（后台标签/嵌入式 webview），看门狗降级为 setTimeout 30FPS
  _loop() {
    const now = performance.now();
    this._frameAt = now;
    this._schedule();
    const dt = Math.min(this._clock.getDelta(), 0.05);
    try {
      this._fpsCount++; this._fpsTime += dt;
      if (this._fpsTime >= 0.5) {
        this.ui.setFPS(Math.round(this._fpsCount / this._fpsTime));
        this._fpsCount = 0; this._fpsTime = 0;
      }

      if (this.playing) {
        // 固定子步进（1/120s）保证数值稳定
        const total = dt * this.speed;
        const step = 1 / 120;
        let acc = total;
        while (acc > 0) {
          const h = Math.min(step, acc);
          for (const eq of this.equipments) eq.update(h);
          this.simTime += h;
          acc -= h;
        }
      }
      // 光线追踪与 UI 读数（每帧）
      this._traceOptics();
      this._photogateCheck();
      this.rig.update(dt);
      this.ui.setClock(this.simTime);
      if (this._uiTick === undefined) this._uiTick = 0;
      this._uiTick += dt;
      if (this._uiTick > 0.15) { this._uiTick = 0; this.ui._refreshReadouts(); }

      renderer.render(this.scene, camera);
    } catch (e) {
      // 循环内异常不能中断渲染链；记录并继续
      if (!this._loopErrLogged) {
        this._loopErrLogged = true;
        console.error('[主循环异常]', e);
        window.__loopError = e.stack || String(e);
      }
    }
  }

  _schedule() {
    if (!this._watchdog) {
      // 看门狗：400ms 无新帧则降级 setTimeout 驱动
      this._watchdog = setInterval(() => {
        if (performance.now() - (this._frameAt || 0) > 400) {
          if (this._rafMode !== 'timeout') {
            this._rafMode = 'timeout';
            this._timeoutTick();
          }
        }
      }, 300);
    }
    if (this._rafMode === 'timeout') {
      // 已降级：后台由 _timeoutTick 驱动；同时保持一个 rAF 探测，恢复前台时切回高帧率
      if (!this._rafProbe) {
        this._rafProbe = requestAnimationFrame(() => {
          this._rafProbe = null;
          if (this._rafMode === 'timeout') {
            this._rafMode = 'raf'; // 前台恢复，rAF 重新可用
          }
        });
      }
      return;
    }
    requestAnimationFrame(() => this._loop());
  }

  _timeoutTick() {
    if (this._rafMode === 'timeout') {
      setTimeout(() => { this._loop(); this._timeoutTick(); }, 33);
    }
    // raf 模式恢复后由 rAF 回调驱动，此链自然终止
  }

  // 光电门：检测与单摆球/滑块的遮挡
  _photogateCheck() {
    const gates = this.equipments.filter(e => e.def?.id === 'photogate');
    if (gates.length === 0) return;
    const bobs = this.equipments.filter(e => e.def?.id === 'pendulum');
    for (const gate of gates) {
      let blocked = false;
      for (const bob of bobs) {
        // 在世界坐标里比较摆球与光电门位置
        const bobW = bob.bob.getWorldPosition(new THREE.Vector3());
        const gateW = gate.gapTop.getWorldPosition(new THREE.Vector3());
        if (bobW.distanceTo(gateW) < 0.09) blocked = true;
      }
      gate.setBlocked(blocked);
    }
  }
}

// 全局引用（resize 闭包用）
let renderer, camera;
const app = new App();
window.__lab = app; // 调试与测试钩子
window.__labReady = true;
