// 设备基类：所有实验设备的自包含抽象
// 生命周期: constructor(zone) -> build() -> added to scene -> update(dt) 每仿真步 -> serialize/deserialize
import * as THREE from 'three';
import { ZONES, BENCH, makeTextSprite } from '../scene/LabScene.js';
import { EventBus, EV } from '../core/EventBus.js';

let UID = 1;

export class EquipmentBase {
  constructor(zoneKey, def) {
    this.uid = UID++;
    this.zoneKey = zoneKey;
    this.def = def;              // registry 中的定义 {id,name,icon,desc,make}
    this.group = new THREE.Group();
    this.group.userData.equipment = this;
    this.params = {};            // 运行参数（由 paramSchema 初始化）
    this.state = {};             // 仿真状态（时间、角度等）
    this.selected = false;
    this._selectRing = null;
    this._running = true;
    // 默认放在所在分区台面中心附近（叠加少量偏移避免重叠）
    const z = ZONES[zoneKey];
    this.group.position.set(z.x, BENCH.H, z.z);
  }

  // 子类覆盖：声明可调参数 -> 自动生成属性面板 UI
  paramSchema() { return []; }
  // 子类覆盖：本设备的实时读数 -> 数据面板/读数卡
  getReadouts() { return {}; }
  // 子类覆盖：每仿真步进
  update(dt) {}
  // 子类覆盖：构建 3D 造型（在构造后调用一次）
  build() {}
  // 子类覆盖：恢复初始状态（重置本设备）
  reset() { this.state = {}; this.onParamsChanged(); }
  // 参数变更后的重建/刷新
  onParamsChanged() {}
  // 数据记录列（供数据表）
  dataColumns() { return Object.keys(this.getReadouts()); }
  // 序列化
  serialize() {
    return { type: this.def.id, zone: this.zoneKey, pos: this.group.position.toArray(), params: { ...this.params } };
  }
  deserialize(d) {
    if (d.pos) this.group.position.fromArray(d.pos);
    if (d.params) Object.assign(this.params, d.params);
    this.onParamsChanged();
  }

  initParams() {
    for (const p of this.paramSchema()) this.params[p.key] = p.value;
  }

  setSelected(on) {
    this.selected = on;
    if (on && !this._selectRing) {
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.34, 0.4, 40),
        new THREE.MeshBasicMaterial({ color: 0x4cc3ff, transparent: true, opacity: 0.9, side: THREE.DoubleSide })
      );
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = 0.005;
      this.group.add(ring);
      this._selectRing = ring;
    }
    if (this._selectRing) this._selectRing.visible = on;
    this.group.traverse(c => {
      if (c.material?.emissive && c.userData?.highlightable !== false) {
        if (on) { c.userData._origEmissive = c.userData._origEmissive ?? c.material.emissive.getHex(); c.material.emissive.setHex(0x184a63); }
        else if (c.userData._origEmissive !== undefined) { c.material.emissive.setHex(c.userData._origEmissive); }
      }
    });
  }

  makeLabel(text, y = 0.5, color = '#e8eef7') {
    const sp = makeTextSprite(text, color, 40);
    sp.position.set(0, y, 0);
    sp.scale.set(0.9, 0.26, 1);
    sp.userData.highlightable = false;
    this.group.add(sp);
    return sp;
  }

  toast(msg, kind) { EventBus.emit(EV.TOAST, { msg, kind }); }
}
