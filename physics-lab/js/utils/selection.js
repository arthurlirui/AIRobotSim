// 交互：射线拾取、设备拖拽（贴台面）、电路接线柱点击连线
import * as THREE from 'three';
import { BENCH } from '../scene/LabScene.js';

export class Interaction {
  constructor(camera, canvas, lab) {
    this.camera = camera;
    this.canvas = canvas;
    this.lab = lab;           // { scene, equipments: [], zoneBenches, ZONES }
    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    this.selected = null;
    this.dragging = null;
    this.dragOffset = new THREE.Vector3();
    this.wiringFrom = null;   // 正在接线的端子
    this.onSelect = null;     // 回调
    this.onDelete = null;
    this.onRequestDeleteWire = null;
    this._downPos = null;
    this._moved = false;
    this._bindEvents();
  }

  _bindEvents() {
    const c = this.canvas;
    c.addEventListener('pointerdown', e => this._down(e));
    c.addEventListener('pointermove', e => this._move(e));
    c.addEventListener('pointerup', e => this._up(e));
    window.addEventListener('keydown', e => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA') return;
      if (e.code === 'Delete' && this.selected) { this.onDelete?.(this.selected); }
      if (e.code === 'Space') { e.preventDefault(); this.onTogglePlay?.(); }
    });
    this.onTogglePlay = null;
  }

  _ndc(e) {
    const r = this.canvas.getBoundingClientRect();
    this.pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
    this.pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
  }

  _pick(e) {
    this._ndc(e);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    // 1) 先查接线柱（电路连线模式）
    const knobHits = this.raycaster.intersectObjects(this._terminals(), true);
    // 2) 设备本体
    const hits = this.raycaster.intersectObjects(this.lab.equipGroups(), true);
    return { knobHits, hits };
  }

  _terminals() {
    const arr = [];
    for (const eq of this.lab.equipments) {
      if (eq.isCircuitHost) {
        for (const comp of eq.components) {
          comp.group.children.forEach(ch => { if (ch.userData.terminal) arr.push(ch); });
        }
      }
    }
    return arr;
  }

  _down(e) {
    if (e.button !== 0) return;
    this._downPos = { x: e.clientX, y: e.clientY };
    this._moved = false;
    const { knobHits, hits } = this._pick(e);

    // 接线柱优先：连线
    if (knobHits.length > 0) {
      const term = knobHits[0].object.userData.terminal;
      this._handleTerminal(term);
      return;
    }
    // 导线点击 -> 询问删除
    if (hits.length > 0) {
      let obj = hits[0].object;
      while (obj && !obj.userData.wire && !obj.userData.equipment && !obj.userData.circuitComponent && obj.parent) obj = obj.parent;
      if (obj?.userData.wire) { this.onRequestDeleteWire?.(obj.userData.wire); return; }
      // 电路组件（开关切换/滑动变阻器）
      let comp = hits[0].object;
      while (comp && !comp.userData.circuitComponent && comp.parent) comp = comp.parent;
      if (comp?.userData.circuitComponent?.onClicked) {
        comp.userData.circuitComponent.onClicked();
        return;
      }
      // 设备
      let eqObj = hits[0].object;
      while (eqObj && !eqObj.userData.equipment && eqObj.parent) eqObj = eqObj.parent;
      if (eqObj?.userData.equipment) {
        const eq = eqObj.userData.equipment;
        this.select(eq);
        // 开始拖拽
        this.dragging = eq;
        const zone = this.lab.ZONES[eq.zoneKey];
        const wp = eq.group.position;
        this.dragOffset.set(wp.x - zone.x, 0, wp.z - zone.z);
        this.canvas.style.cursor = 'grabbing';
        return;
      }
    }
    // 点空白：取消选择
    this.select(null);
  }

  _handleTerminal(term) {
    if (!this.wiringFrom) {
      this.wiringFrom = term;
      this.toast('已选择接线柱，点击另一个接线柱完成连线');
    } else {
      const bench = this._findBench(term.comp);
      if (bench && bench.connect(this.wiringFrom, term)) {
        this.toast('✓ 连线成功');
      } else {
        this.toast('无法连接（同组件或已存在）', 'warn');
      }
      this.wiringFrom = null;
    }
  }

  _findBench(comp) {
    return this.lab.equipments.find(eq => eq.isCircuitHost && eq.components.includes(comp));
  }

  _move(e) {
    if (this._downPos && (Math.abs(e.clientX - this._downPos.x) > 3 || Math.abs(e.clientY - this._downPos.y) > 3)) this._moved = true;
    if (!this.dragging) return;
    this._ndc(e);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    // 与台面高度水平面求交
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -(BENCH.H + 0.001));
    const pt = new THREE.Vector3();
    if (this.raycaster.ray.intersectPlane(plane, pt)) {
      const zone = this.lab.ZONES[this.dragging.zoneKey];
      const halfW = 1.7 / 2 - 0.15, halfD = 0.95 - 0.15;
      const nx = THREE.MathUtils.clamp(pt.x - this.dragOffset.x, zone.x - halfW, zone.x + halfW);
      const nz = THREE.MathUtils.clamp(pt.z - this.dragOffset.z, zone.z - halfD, zone.z + halfD);
      this.dragging.group.position.x = nx;
      this.dragging.group.position.z = nz;
    }
  }

  _up(e) {
    this.dragging = null;
    this.canvas.style.cursor = 'default';
    this._downPos = null;
  }

  select(eq) {
    if (this.selected === eq) return;
    if (this.selected) this.selected.setSelected(false);
    this.selected = eq;
    if (eq) eq.setSelected(true);
    this.onSelect?.(eq);
  }

  toast(msg, kind) { this.lab.toast(msg, kind); }
}
