import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ============ ChemLab 引擎主体 ============
export class ChemLabEngine {
  constructor(container) {
    this.container = container;
    this.registry = [];       // { id, mesh, content?, kind, label }
    this.itemByMesh = new Map();
    this._id = 0;

    // 渲染器
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.6;
    container.appendChild(this.renderer.domElement);

    // 场景与相机
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x18202a);
    this.scene.fog = new THREE.Fog(0x18202a, 30, 60);
    this.camera = new THREE.PerspectiveCamera(55, container.clientWidth / container.clientHeight, 0.1, 100);
    this.camera.position.set(7.5, 6.5, 9.5);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 1.2, 0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.02;
    this.controls.minDistance = 2;
    this.controls.maxDistance = 28;

    this._setupLights();
    this._animate();
  }

  _setupLights() {
    const hemi = new THREE.HemisphereLight(0xf0f6fa, 0x404a56, 1.35);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xfff4e0, 2.2);
    sun.position.set(6, 10, 4);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -12; sun.shadow.camera.right = 12;
    sun.shadow.camera.top = 12; sun.shadow.camera.bottom = -12;
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0xbcd0e8, 0.9);
    fill.position.set(-8, 6, -6);
    this.scene.add(fill);
    this.lampLight = new THREE.PointLight(0xffc070, 0, 8);
    this.lampLight.position.set(0, 2, 0);
    this.scene.add(this.lampLight);
  }

  register(mesh, content = null) {
    mesh.userData._registered = true;
    const item = { id: this._id++, mesh, content, kind: mesh.userData.kind, label: mesh.userData.label };
    this.registry.push(item);
    this.itemByMesh.set(mesh, item);
    this.scene.add(mesh);
    return item;
  }

  unregister(item) {
    const i = this.registry.indexOf(item);
    if (i >= 0) this.registry.splice(i, 1);
    this.itemByMesh.delete(item.mesh);
    if (item.mesh.parent) item.mesh.parent.remove(item.mesh);
    // 移除粒子发射器等
  }

  // 更新容器液体显示
  refreshLiquid(item) {
    if (!item) return;
    if (this._liquidSetter) {
      const vol = item.content ? item.content.totalVolume() : 0;
      this._liquidSetter(item.mesh, vol, item.content ? item.content.color() : 0xa8d4e6);
    }
  }

  isHeated(item) {
    // 加热判定：火焰中心水平距离 1.0m 内、且容器底面在火焰上方 0.9m 以内
    if (!this.interaction) return false;
    const flames = this.interaction.flames;
    const box = new THREE.Box3().setFromObject(item.mesh);
    const cx = (box.min.x + box.max.x) / 2, cz = (box.min.z + box.max.z) / 2;
    for (const f of flames.flames.values()) {
      const p = f.lamp.localToWorld(f.anchor.clone());
      const d = Math.hypot(p.x - cx, p.z - cz);
      if (d < 1.0 && box.min.y > p.y - 0.2 && box.min.y < p.y + 0.9) return true;
    }
    return false;
  }

  findBelowBurette(buretteItem) {
    const tip = buretteItem.mesh.localToWorld(new THREE.Vector3(0, 0.1, 0));
    let best = null, bestD = 0.75;
    for (const item of this.registry) {
      if (item === buretteItem) continue;
      const ud = item.mesh.userData;
      if (ud.capacity === undefined || ud.kind === 'burette') continue;
      const box = new THREE.Box3().setFromObject(item.mesh);
      const cx = (box.min.x + box.max.x) / 2, cz = (box.min.z + box.max.z) / 2;
      const d = Math.hypot(cx - tip.x, cz - tip.z);
      if (d < bestD && tip.y > box.max.y - 0.1 && tip.y < box.max.y + 1.4) { best = item; bestD = d; }
    }
    return best;
  }

  // 拖拽辅助：将物品放到台面/支撑点上
  findSupport(item, x, z) {
    let y = 0;
    for (const b of this.registry) {
      const ud = b.mesh.userData;
      if (!ud.isBench && !ud.isRingSupport && !ud.isPanSupport) continue;
      if (b === item) continue;
      const box = new THREE.Box3().setFromObject(b.mesh);
      if (x > box.min.x - 0.1 && x < box.max.x + 0.1 && z > box.min.z - 0.1 && z < box.max.z + 0.1) {
        const top = box.max.y;
        if (top > y) y = top;
      }
    }
    return y;
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  _animate() {
    const loop = () => {
      requestAnimationFrame(loop);
      const dt = Math.min(0.05, this._clockGetDt());
      if (this.interaction) this.interaction.update(dt);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    this._clock = new THREE.Clock();
    this._clockGetDt = () => this._clock.getDelta();
    loop();
  }

  screenshot() {
    this.renderer.render(this.scene, this.camera);
    return this.renderer.domElement.toDataURL('image/png');
  }
}
