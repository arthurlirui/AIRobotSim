import * as THREE from 'three';
import { REAGENTS, findReaction, ContainerContent } from './chemistry.js';

// ============ 交互系统：拾取、拖拽、倾倒、加热、搅拌、滴定 ============

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

// 液滴/气泡/火焰粒子池
export class ParticleSystem {
  constructor(scene) {
    this.scene = scene;
    this.particles = [];
    const geo = new THREE.SphereGeometry(1, 6, 5);
    this.pool = [];
    for (let i = 0; i < 200; i++) {
      const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.85 }));
      m.visible = false;
      scene.add(m);
      this.pool.push(m);
    }
  }
  spawn(pos, color, scale, vel, life, { gravity = -3, fade = true, buoyant = false } = {}) {
    const m = this.pool.find(p => !p.visible);
    if (!m) return;
    m.visible = true;
    m.position.copy(pos);
    m.scale.setScalar(scale);
    m.material.color.set(color);
    m.userData = { vel: vel.clone(), life, maxLife: life, gravity: gravity * (buoyant ? -1 : 1), fade };
  }
  update(dt) {
    for (const m of this.pool) {
      if (!m.visible) continue;
      const ud = m.userData;
      ud.life -= dt;
      if (ud.life <= 0) { m.visible = false; continue; }
      ud.vel.y += ud.gravity * dt;
      m.position.addScaledVector(ud.vel, dt);
      if (ud.fade) m.material.opacity = 0.85 * (ud.life / ud.maxLife);
      m.scale.multiplyScalar(1 - 0.3 * dt);
    }
  }
}

// 气泡发射器（挂到容器上）
export class GasEmitter {
  constructor(particles) {
    this.particles = particles;
    this.emitters = new Map(); // object.uuid -> { anchor, color, rate, acc }
  }
  start(obj, anchor, color, rate = 18) {
    this.emitters.set(obj.uuid, { anchor: anchor.clone(), color, rate, acc: 0 });
  }
  stop(obj) { this.emitters.delete(obj.uuid); }
  update(dt) {
    for (const [uuid, e] of this.emitters) {
      e.acc += e.rate * dt;
      while (e.acc >= 1) {
        e.acc -= 1;
        const p = e.anchor.clone();
        p.x += (Math.random() - 0.5) * 0.12;
        p.z += (Math.random() - 0.5) * 0.12;
        this.particles.spawn(p, 0xffffff, 0.02 + Math.random() * 0.012,
          new THREE.Vector3((Math.random() - 0.5) * 0.12, 0.6 + Math.random() * 0.5, (Math.random() - 0.5) * 0.12),
          1.0, { gravity: 0.3, buoyant: true, fade: false });
      }
    }
  }
}

// 火焰（酒精灯）
export class FlameManager {
  constructor(particles) {
    this.particles = particles;
    this.flames = new Map();
  }
  ignite(lamp, anchor) {
    if (this.flames.has(lamp.uuid)) return;
    // 内焰锥 + 外焰锥（锚点为灯本地坐标，直接挂在灯上，随灯移动）
    const inner = new THREE.Mesh(
      new THREE.ConeGeometry(0.035, 0.16, 10),
      new THREE.MeshBasicMaterial({ color: 0x58b8ff, transparent: true, opacity: 0.95, depthWrite: false })
    );
    inner.position.copy(anchor); inner.position.y += 0.08;
    const outer = new THREE.Mesh(
      new THREE.ConeGeometry(0.06, 0.24, 10),
      new THREE.MeshBasicMaterial({ color: 0xff9828, transparent: true, opacity: 0.75, depthWrite: false })
    );
    outer.position.copy(anchor); outer.position.y += 0.11;
    const light = new THREE.PointLight(0xffa050, 1.2, 2.5);
    light.position.copy(anchor); light.position.y += 0.15;
    lamp.add(inner, outer, light);
    this.flames.set(lamp.uuid, { inner, outer, light, anchor: anchor.clone(), lamp, t: 0 });
  }
  extinguish(lamp) {
    const f = this.flames.get(lamp.uuid);
    if (!f) return;
    f.inner.removeFromParent(); f.outer.removeFromParent(); f.light.removeFromParent();
    this.flames.delete(lamp.uuid);
  }
  isLit(lamp) { return this.flames.has(lamp.uuid); }
  update(dt) {
    for (const f of this.flames.values()) {
      f.t += dt;
      const flick = 1 + Math.sin(f.t * 18) * 0.08 + Math.sin(f.t * 31) * 0.05;
      f.inner.scale.set(flick, 1 + (flick - 1) * 1.6, flick);
      f.outer.scale.set(flick, 1 + (flick - 1) * 1.2, flick);
      f.light.intensity = 1.0 + Math.sin(f.t * 24) * 0.25;
    }
  }
}

// 交互管理器
export class InteractionManager {
  constructor(engine) {
    this.engine = engine; // ChemLabEngine
    this.scene = engine.scene;
    this.camera = engine.camera;
    this.renderer = engine.renderer;
    this.particles = new ParticleSystem(this.scene);
    this.gasEmitters = new GasEmitter(this.particles);
    this.flames = new FlameManager(this.particles);
    this.selected = null;      // 当前选中对象
    this.held = null;          // 手持对象
    this.heldIsPouring = false;
    this.dragging = false;
    this.onSelect = null;      // 回调 UI
    this.onLog = null;         // 日志回调
    this.onInspect = null;     // 检视面板回调
    this.hovered = null;
    this._highlight = null;
    this._makeHighlight();
    this.buretteFlow = 0;      // 0 关闭 1 慢 2 中 3 快
    this.clock = new THREE.Clock();
  }

  _makeHighlight() {
    const geo = new THREE.SphereGeometry(1, 16, 12);
    this._highlight = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x44ff88, transparent: true, opacity: 0.25 }));
    this._highlight.visible = false;
    this.scene.add(this._highlight);
  }

  log(msg) { this.onLog && this.onLog(msg); }

  // 所有可交互对象（遍历 registry）
  pickables() { return this.engine.registry; }

  setPointer(clientX, clientY) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    pointer.x = ((clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  }

  raycast(clientX, clientY) {
    this.setPointer(clientX, clientY);
    raycaster.setFromCamera(pointer, this.camera);
    const objs = [];
    for (const item of this.pickables()) objs.push(item.mesh);
    const hits = raycaster.intersectObjects(objs, true);
    if (hits.length === 0) return null;
    // 向上查找注册对象
    let o = hits[0].object;
    while (o) {
      if (o.userData._registered) return { item: this.engine.itemByMesh.get(o), point: hits[0].point };
      o = o.parent;
    }
    return null;
  }

  select(item) {
    this.selected = item;
    if (this.onSelect) this.onSelect(item);
    if (item) {
      const box = new THREE.Box3().setFromObject(item.mesh);
      const size = box.getSize(new THREE.Vector3());
      const center = box.getCenter(new THREE.Vector3());
      const r = Math.max(size.x, size.y, size.z) * 0.65 + 0.03;
      this._highlight.position.copy(center);
      this._highlight.scale.setScalar(r);
      this._highlight.visible = true;
    } else {
      this._highlight.visible = false;
    }
  }

  // —— 核心操作：向容器加入试剂 ——
  addReagentToContainer(item, reagentKey, volume) {
    const ud = item.mesh.userData;
    if (ud.capacity === undefined) { this.log(`⚠ ${ud.label} 不是容器，无法加入试剂`); return false; }
    if (!item.content) item.content = new ContainerContent();
    const re = REAGENTS[reagentKey];
    if (!re) return false;
    const c = item.content;
    const space = ud.capacity - c.totalVolume();
    const add = Math.min(volume, Math.max(0, space));
    if (add <= 0) { this.log(`⚠ ${ud.label} 已满`); return false; }
    c.add(reagentKey, add);
    this.applyReaction(item, { heated: this.engine.isHeated(item) });
    this.log(`➕ 向「${ud.label}」加入 ${re.name} ${add.toFixed(1)} mL`);
    return true;
  }

  // 应用反应规则
  applyReaction(item, { heated = false } = {}) {
    const c = item.content;
    if (!c) return;
    const keys = c.entries.map(e => e.key);
    const r = findReaction(keys, { heated });
    const ud = item.mesh.userData;
    if (r) {
      c.lastReaction = r;
      const res = r.result;
      if (res.tempDelta) c.temperature = Math.min(110, c.temperature + res.tempDelta);
      if (res.gas) {
        c.gasProducing = res.gas;
        const anchor = this.liquidSurfaceWorld(item);
        if (anchor) this.gasEmitters.start(item.mesh, anchor, res.gas.color || 0xd0e0e8, 22);
      }
      if (res.precipitate) {
        c.precipitate = res.precipitate;
        c.precipitateAmount = Math.min(1, c.precipitateAmount + 0.5);
      }
      this.log(`🧪 ${res.equation}`);
      this.log(`   现象：${res.note}`);
      if (res.requireHeat && !heated) this.log(`   ⚠ 该反应需要加热（当前 ${c.temperature.toFixed(0)}℃）`);
    } else {
      c.lastReaction = null;
      c.gasProducing = null;
      this.gasEmitters.stop(item.mesh);
    }
    this.engine.refreshLiquid(item);
    if (this.onInspect) this.onInspect(item);
  }

  liquidSurfaceWorld(item) {
    const lm = item.mesh.userData._liquidMesh;
    if (lm && lm.visible) {
      const p = new THREE.Vector3(0, lm.position.y + lm.scale.y / 2, 0);
      return item.mesh.localToWorld(p.clone());
    }
    const ud = item.mesh.userData;
    if (ud.fillBase !== undefined) {
      return item.mesh.localToWorld(new THREE.Vector3(0, ud.fillBase + 0.02, 0));
    }
    return null;
  }

  // —— 倾倒：把 source 的液体倒入 target ——
  pour(source, target) {
    const sc = source.content, tc = target.content;
    if (!sc || sc.totalVolume() <= 0) { this.log(`⚠「${source.mesh.userData.label}」是空的`); return false; }
    if (!tc) target.content = new ContainerContent();
    const tud = target.mesh.userData;
    if (tud.capacity === undefined) { this.log(`⚠ ${tud.label} 不能盛装液体`); return false; }
    const rate = 8; // mL/s
    const move = Math.min(rate * 0.2, sc.totalVolume(), tud.capacity - target.content.totalVolume());
    if (move <= 0) { this.log(`⚠「${tud.label}」已满`); return false; }
    for (const e of sc.entries) {
      const part = (e.volume / sc.totalVolume()) * move;
      target.content.add(e.key, part);
      e.volume -= part;
    }
    sc.entries = sc.entries.filter(e => e.volume > 0.01);
    if (sc.precipitate && !target.content.precipitate) {
      target.content.precipitate = sc.precipitate;
      target.content.precipitateAmount = sc.precipitateAmount * 0.7;
    }
    this.applyReaction(target, { heated: this.engine.isHeated(target) });
    this.engine.refreshLiquid(source);
    this.log(`🫗 将「${source.mesh.userData.label}」倒入「${tud.label}」(${move.toFixed(1)} mL)`);
    return true;
  }

  // —— 滴定阀门控制 ——
  setBuretteFlow(level) { this.buretteFlow = level; }

  buretteTick(item, dt) {
    if (!this.buretteFlow || !item || item.mesh.userData.kind !== 'burette') return;
    if (!item.content || item.content.totalVolume() <= 0) return;
    const rates = [0, 1.2, 4, 12]; // mL/s
    const v = rates[this.buretteFlow] * dt;
    const below = this.engine.findBelowBurette(item);
    if (below) {
      this.pourBuretteDrop(item, below, v);
    } else {
      // 滴到台面
      item.content.entries.forEach(e => e.volume = Math.max(0, e.volume - v));
      item.content.entries = item.content.entries.filter(e => e.volume > 0.01);
      this.engine.refreshLiquid(item);
    }
  }

  pourBuretteDrop(burette, target, volume) {
    const c = burette.content;
    if (c.totalVolume() <= 0) return;
    if (!target.content) target.content = new ContainerContent();
    const move = Math.min(volume, c.totalVolume(), target.mesh.userData.capacity - target.content.totalVolume());
    if (move <= 0) return;
    for (const e of c.entries) {
      const part = (e.volume / c.totalVolume()) * move;
      target.content.add(e.key, part);
      e.volume -= part;
    }
    c.entries = c.entries.filter(e => e.volume > 0.01);
    this.applyReaction(target, { heated: this.engine.isHeated(target) });
    this.engine.refreshLiquid(burette);
    // 滴落粒子
    const tipW = burette.mesh.localToWorld(new THREE.Vector3(0, 0.12, 0));
    const surfT = this.liquidSurfaceWorld(target) || target.mesh.getWorldPosition(new THREE.Vector3());
    this.particles.spawn(tipW, c.color(), 0.014, new THREE.Vector3(0, -0.3, 0), 0.35, { gravity: -2 });
  }

  // —— 加热 ——
  toggleLamp(lamp) {
    if (this.flames.isLit(lamp.mesh)) {
      this.flames.extinguish(lamp.mesh);
      this.log(`🔥 熄灭「${lamp.mesh.userData.label}」`);
    } else {
      if ((lamp.mesh.userData.fuel ?? 1) <= 0) { this.log('⚠ 酒精不足，请先添加酒精'); return; }
      const anchor = lamp.mesh.localToWorld(lamp.mesh.userData.flameAnchor.clone());
      // 转换到 lamp 父级坐标（FlameManager add 到 parent）
      this.flames.ignite(lamp.mesh, lamp.mesh.userData.flameAnchor.clone());
      this.log(`🔥 点燃「${lamp.mesh.userData.label}」（外焰温度最高，约 500℃）`);
    }
  }

  // —— 搅拌 ——
  stir(item) {
    if (!item.content) return;
    item.content.stirred = true;
    const lm = item.mesh.userData._liquidMesh;
    if (lm) {
      const orig = { x: lm.position.x, z: lm.position.z };
      let t = 0;
      const anim = () => {
        t += 0.12;
        lm.position.x = orig.x + Math.sin(t * 5) * 0.02;
        lm.position.z = orig.z + Math.cos(t * 5) * 0.02;
        if (t < 1.2) requestAnimationFrame(anim);
        else { lm.position.x = orig.x; lm.position.z = orig.z; }
      };
      anim();
    }
    this.log(`🌀 搅拌「${item.mesh.userData.label}」`);
  }

  update(dt) {
    this.particles.update(dt);
    this.gasEmitters.update(dt);
    this.flames.update(dt);
    // 持续气体随液面更新锚点
    for (const item of this.engine.registry) {
      if (item.content?.gasProducing) {
        const a = this.liquidSurfaceWorld(item);
        if (a) {
          const e = this.gasEmitters.emitters.get(item.mesh.uuid);
          if (e) e.anchor.copy(a);
        }
      }
    }
  }
}
