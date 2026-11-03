import * as THREE from 'three';
import { makeStandardMaterial, makeGlassMaterial, makePorcelainMaterial, makeMetalMaterial, lathePoints, makeLathe, makeCylinder, makeBox } from './utils.js';

// ============ 玻璃仪器与设备程序化建模 ============
// 每个工厂函数返回 THREE.Group，userData: { kind, label, capacity_mL, liquid: {volume, color, ...} }

const GLASS = () => makeGlassMaterial();
const PORCELAIN = () => makePorcelainMaterial();
const METAL = (c, r) => makeMetalMaterial(c, r);

// --- 液体网格：在容器内部渲染液面 ---
export function createLiquidMesh(radiusTop, radiusBottom, height, color) {
  const geo = new THREE.CylinderGeometry(radiusTop, radiusBottom, height, 24, 1);
  const mat = new THREE.MeshStandardMaterial({
    color: new THREE.Color(color), transparent: true, opacity: 0.95, roughness: 0.2,
    emissive: new THREE.Color(color).multiplyScalar(0.25)
  });
  const mesh = new THREE.Mesh(geo, mat);
  return mesh;
}

// 烧杯
export function makeBeaker(capacity = 250) {
  const g = new THREE.Group();
  const R = capacity >= 500 ? 0.42 : capacity >= 250 ? 0.32 : 0.25;
  const H = capacity >= 500 ? 0.95 : capacity >= 250 ? 0.72 : 0.55;
  const r = R * 0.94, h = H * 0.92;
  // 器壁（Lathe，带厚度）
  const pts = lathePoints([[0.012, 0], [r, 0.012], [r, h], [r * 0.995, h + 0.02]]);
  const wall = makeLathe(pts, GLASS());
  // 侧嘴（用一个小楔形环模拟）
  const spout = makeCylinder(0.012, 0.02, 0.03, GLASS(), 12);
  spout.rotation.z = Math.PI / 3;
  spout.position.set(r + 0.015, h + 0.01, 0);
  g.add(wall, spout);
  g.userData = { kind: 'beaker', label: `${capacity}mL 烧杯`, capacity, fillRadius: r * 0.92, fillBase: 0.03, fillMax: h - 0.03 };
  return g;
}

// 锥形瓶
export function makeConicalFlask(capacity = 250) {
  const g = new THREE.Group();
  const pts = lathePoints([[0.012, 0], [0.34, 0.02], [0.28, 0.28], [0.12, 0.55], [0.11, 0.62], [0.115, 0.75]]);
  const wall = makeLathe(pts, GLASS());
  g.add(wall);
  g.userData = { kind: 'flask_conical', label: `${capacity}mL 锥形瓶`, capacity, fillRadius: 0.2, fillBase: 0.03, fillMax: 0.55, coneFill: true };
  return g;
}

// 圆底烧瓶
export function makeRoundFlask(capacity = 250) {
  const g = new THREE.Group();
  const sph = new THREE.Mesh(new THREE.SphereGeometry(0.34, 32, 24), GLASS());
  sph.position.y = 0.34;
  const neck = makeCylinder(0.1, 0.1, 0.3, GLASS(), 24, true);
  neck.position.y = 0.72;
  g.add(sph, neck);
  g.userData = { kind: 'flask_round', label: `${capacity}mL 圆底烧瓶`, capacity, fillRadius: 0.3, fillBase: 0.06, fillMax: 0.6, sphereFill: true };
  return g;
}

// 平底烧瓶
export function makeFlatFlask(capacity = 250) {
  const g = new THREE.Group();
  const pts = lathePoints([[0.012, 0], [0.33, 0.015], [0.33, 0.3], [0.13, 0.55], [0.12, 0.72]]);
  const wall = makeLathe(pts, GLASS());
  g.add(wall);
  g.userData = { kind: 'flask_flat', label: `${capacity}mL 平底烧瓶`, capacity, fillRadius: 0.28, fillBase: 0.03, fillMax: 0.5, coneFill: true };
  return g;
}

// 试管（含帽）
export function makeTestTube() {
  const g = new THREE.Group();
  const pts = lathePoints([[0.008, 0], [0.095, 0.012], [0.095, 0.5], [0.09, 0.52]]);
  const wall = makeLathe(pts, GLASS());
  g.add(wall);
  g.userData = { kind: 'test_tube', label: '试管', capacity: 20, fillRadius: 0.08, fillBase: 0.02, fillMax: 0.44 };
  return g;
}

// 量筒
export function makeMeasuringCylinder(capacity = 100) {
  const g = new THREE.Group();
  const R = 0.115, H = 0.68;
  const pts = lathePoints([[0.05, 0], [R, 0.02], [R, H - 0.04], [R * 0.9, H], [R * 0.65, H + 0.02]]);
  const wall = makeLathe(pts, GLASS());
  // 底座
  const base = makeCylinder(0.16, 0.17, 0.035, GLASS(), 24);
  base.position.y = -0.018;
  g.add(wall, base);
  // 刻度（用白色细环组模拟）
  for (let i = 1; i <= 5; i++) {
    const ring = makeCylinder(R + 0.004, R + 0.004, 0.006, makeStandardMaterial(0x224466, { transparent: true, opacity: 0.85 }), 24, true);
    ring.position.y = 0.04 + (H - 0.1) * (i / 6);
    g.add(ring);
  }
  g.userData = { kind: 'cylinder_graduated', label: `${capacity}mL 量筒`, capacity, fillRadius: R * 0.92, fillBase: 0.035, fillMax: H - 0.08, isVolumetric: true };
  return g;
}

// 容量瓶
export function makeVolumetricFlask(capacity = 100) {
  const g = new THREE.Group();
  const pts = lathePoints([[0.012, 0], [0.2, 0.015], [0.2, 0.22], [0.06, 0.36], [0.055, 0.6], [0.062, 0.62]]);
  const wall = makeLathe(pts, GLASS());
  g.add(wall);
  g.userData = { kind: 'flask_volumetric', label: `${capacity}mL 容量瓶`, capacity, fillRadius: 0.17, fillBase: 0.03, fillMax: 0.34, coneFill: true, isVolumetric: true };
  return g;
}

// 滴定管（带聚四氟乙烯旋塞 + 刻度），挂载于滴定台
export function makeBurette(capacity = 50) {
  const g = new THREE.Group();
  const R = 0.055, H = 1.3;
  const tube = makeCylinder(R, R, H, GLASS(), 24, true);
  tube.position.y = H / 2 + 0.25;
  const tip = makeCylinder(0.008, 0.018, 0.16, GLASS(), 12);
  tip.position.y = 0.17;
  const stopcock = makeCylinder(0.045, 0.045, 0.06, makeStandardMaterial(0xf5f5f0, { roughness: 0.4 }), 16);
  stopcock.rotation.z = Math.PI / 2;
  stopcock.position.y = 0.22;
  const handle = makeCylinder(0.014, 0.014, 0.14, makeStandardMaterial(0xff4444, { roughness: 0.4 }), 12);
  handle.rotation.z = Math.PI / 2;
  handle.position.set(0.11, 0.22, 0);
  g.add(tube, tip, stopcock, handle);
  for (let i = 1; i <= 10; i++) {
    const y = 0.32 + (H - 0.1) * (i / 11);
    const ring = makeCylinder(R + 0.003, R + 0.003, 0.005, makeStandardMaterial(0x224466, { transparent: true, opacity: 0.9 }), 20, true);
    ring.position.y = y;
    g.add(ring);
  }
  g.userData = { kind: 'burette', label: `${capacity}mL 滴定管`, capacity, buretteInnerR: R * 0.9, isVolumetric: true };
  return g;
}

// 移液管
export function makePipette(capacity = 10) {
  const g = new THREE.Group();
  const bulb = makeLathe(lathePoints([[0.02, 0], [0.09, 0.05], [0.09, 0.14], [0.02, 0.2]]), GLASS());
  const top = makeCylinder(0.02, 0.02, 0.3, GLASS(), 16, true);
  top.position.y = 0.37;
  const bottom = makeCylinder(0.018, 0.018, 0.26, GLASS(), 16, true);
  bottom.position.y = 0.09;
  const tip = makeCylinder(0.006, 0.014, 0.06, GLASS(), 12);
  tip.position.y = -0.04;
  g.add(bulb, top, bottom, tip);
  g.userData = { kind: 'pipette', label: `${capacity}mL 移液管`, capacity, isVolumetric: true };
  return g;
}

// 胶头滴管
export function makeDropper() {
  const g = new THREE.Group();
  const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), makeStandardMaterial(0xbb7777, { roughness: 0.6 }));
  bulb.scale.y = 1.3;
  bulb.position.y = 0.42;
  const tube = makeCylinder(0.02, 0.02, 0.34, GLASS(), 12, true);
  tube.position.y = 0.22;
  const tip = makeCylinder(0.007, 0.016, 0.07, GLASS(), 10);
  tip.position.y = 0.015;
  g.add(bulb, tube, tip);
  g.userData = { kind: 'dropper', label: '胶头滴管', capacity: 1 };
  return g;
}

// 表面皿
export function makeWatchGlass() {
  const g = new THREE.Group();
  const dome = new THREE.Mesh(new THREE.SphereGeometry(0.22, 32, 16, 0, Math.PI * 2, 0, Math.PI / 4.5), GLASS());
  dome.rotation.x = Math.PI;
  dome.position.y = 0.045;
  g.add(dome);
  g.userData = { kind: 'watch_glass', label: '表面皿', capacity: 5 };
  return g;
}

// 蒸发皿
export function makeEvaporatingDish() {
  const g = new THREE.Group();
  const pts = lathePoints([[0.01, 0], [0.24, 0.02], [0.27, 0.1]]);
  const wall = makeLathe(pts, PORCELAIN());
  g.add(wall);
  g.userData = { kind: 'evaporating_dish', label: '蒸发皿', capacity: 60, fillRadius: 0.2, fillBase: 0.025, fillMax: 0.08 };
  return g;
}

// 坩埚
export function makeCrucible() {
  const g = new THREE.Group();
  const pts = lathePoints([[0.01, 0], [0.13, 0.015], [0.14, 0.14], [0.15, 0.16]]);
  const wall = makeLathe(pts, PORCELAIN(0xf2efe9));
  const lid = makeCylinder(0.155, 0.16, 0.018, PORCELAIN(0xf2efe9), 24);
  lid.position.y = 0.17;
  g.add(wall, lid);
  g.userData = { kind: 'crucible', label: '坩埚（带盖）', capacity: 30, fillRadius: 0.11, fillBase: 0.02, fillMax: 0.12 };
  return g;
}

// 研钵和杵
export function makeMortar() {
  const g = new THREE.Group();
  const pts = lathePoints([[0.28, 0], [0.3, 0.02], [0.3, 0.18], [0.31, 0.2]]);
  const bowl = makeLathe(pts, PORCELAIN(0xf0eee8));
  const pestle = makeCylinder(0.03, 0.045, 0.4, PORCELAIN(0xf0eee8), 16);
  pestle.rotation.z = 0.5;
  pestle.position.set(0.08, 0.26, 0);
  g.add(bowl, pestle);
  g.userData = { kind: 'mortar', label: '研钵和杵', capacity: 50, fillRadius: 0.22, fillBase: 0.03, fillMax: 0.14, crushable: true };
  return g;
}

// 漏斗 + 滤纸
export function makeFunnel(withPaper = true) {
  const g = new THREE.Group();
  const cone = makeCylinder(0.24, 0.03, 0.22, GLASS(), 28, true);
  cone.position.y = 0.2;
  const stem = makeCylinder(0.015, 0.015, 0.24, GLASS(), 12, true);
  stem.position.y = 0.03;
  g.add(cone, stem);
  if (withPaper) {
    const paper = makeCylinder(0.22, 0.035, 0.19, makeStandardMaterial(0xfdfdf5, { roughness: 0.9 }), 24, true);
    paper.position.y = 0.19;
    g.add(paper);
    g.userData = { hasFilterPaper: true };
  }
  g.userData = { kind: 'funnel', label: withPaper ? '漏斗（含滤纸）' : '漏斗', capacity: 100, isFilter: true, ...g.userData };
  return g;
}

// 抽滤瓶（过滤瓶）
export function makeFilterFlask() {
  const g = new THREE.Group();
  const pts = lathePoints([[0.012, 0], [0.28, 0.015], [0.28, 0.44], [0.1, 0.56], [0.1, 0.66]]);
  const wall = makeLathe(pts, GLASS());
  const sideTube = makeCylinder(0.03, 0.03, 0.16, GLASS(), 12, true);
  sideTube.rotation.z = Math.PI / 2.4;
  sideTube.position.set(0.3, 0.35, 0);
  g.add(wall, sideTube);
  g.userData = { kind: 'filter_flask', label: '抽滤瓶', capacity: 500, fillRadius: 0.24, fillBase: 0.03, fillMax: 0.4 };
  return g;
}

// 布氏漏斗
export function makeBuchnerFunnel() {
  const g = new THREE.Group();
  const cup = makeCylinder(0.24, 0.22, 0.14, PORCELAIN(), 28);
  cup.position.y = 0.24;
  const stem = makeCylinder(0.03, 0.03, 0.2, PORCELAIN(), 12);
  stem.position.y = 0.07;
  g.add(cup, stem);
  g.userData = { kind: 'buchner_funnel', label: '布氏漏斗', capacity: 100, fillRadius: 0.2, fillBase: 0.2, fillMax: 0.32, isFilter: true };
  return g;
}

// 集气瓶
export function makeGasJar() {
  const g = new THREE.Group();
  const pts = lathePoints([[0.012, 0], [0.28, 0.015], [0.28, 0.5], [0.285, 0.52]]);
  const wall = makeLathe(pts, GLASS());
  const lid = makeCylinder(0.3, 0.3, 0.015, makeStandardMaterial(0xd8e4ea, { transparent: true, opacity: 0.7, roughness: 0.3 }), 28);
  lid.position.y = 0.53;
  g.add(wall, lid);
  g.userData = { kind: 'gas_jar', label: '集气瓶', capacity: 250, fillRadius: 0.25, fillBase: 0.03, fillMax: 0.46 };
  return g;
}

// 干燥器
export function makeDesiccator() {
  const g = new THREE.Group();
  const bowl = makeLathe(lathePoints([[0.01, 0], [0.42, 0.02], [0.42, 0.3], [0.43, 0.32]]), GLASS({ opacity: 0.45 }));
  const lid = makeLathe(lathePoints([[0.01, 0], [0.44, 0.06], [0.45, 0.1]]), GLASS({ opacity: 0.45 }));
  lid.position.y = 0.3;
  const plate = makeCylinder(0.36, 0.36, 0.02, PORCELAIN(), 32);
  plate.position.y = 0.16;
  g.add(bowl, lid, plate);
  g.userData = { kind: 'desiccator', label: '干燥器', capacity: 1000 };
  return g;
}

// 酒精灯（灯芯 + 可点燃火焰粒子锚点）
export function makeAlcoholLamp() {
  const g = new THREE.Group();
  const base = makeCylinder(0.16, 0.19, 0.06, GLASS(), 24);
  base.position.y = 0.03;
  const pts = lathePoints([[0.02, 0], [0.13, 0.03], [0.12, 0.3], [0.125, 0.32]]);
  const body = makeLathe(pts, GLASS());
  body.position.y = 0.05;
  const neck = makeCylinder(0.06, 0.06, 0.05, makeMetalMaterial(0x888888, 0.3), 16);
  neck.position.y = 0.39;
  const wick = makeCylinder(0.012, 0.012, 0.09, makeStandardMaterial(0xf5f0e0), 8);
  wick.position.y = 0.45;
  const cap = makeCylinder(0.075, 0.075, 0.035, makeMetalMaterial(0x777777, 0.3), 16);
  cap.position.y = 0.47;
  g.add(base, body, neck, wick, cap);
  g.userData = { kind: 'alcohol_lamp', label: '酒精灯', capacity: 100, fuel: 0.66, flameAnchor: new THREE.Vector3(0, 0.52, 0), ignitable: true, fillRadius: 0.1, fillBase: 0.09, fillMax: 0.3, fuelColor: 0x9fd8ff };
  return g;
}

// 三脚架
export function makeTripod() {
  const g = new THREE.Group();
  const ring = makeCylinder(0.34, 0.34, 0.02, METAL(0x3a3f45, 0.5), 28, true);
  ring.position.y = 0.32;
  g.add(ring);
  for (let i = 0; i < 3; i++) {
    const a = (i / 3) * Math.PI * 2 + 0.5;
    const leg = makeCylinder(0.018, 0.018, 0.34, METAL(0x3a3f45, 0.5), 10);
    leg.position.set(Math.cos(a) * 0.28, 0.16, Math.sin(a) * 0.28);
    leg.rotation.z = Math.cos(a) * 0.1; leg.rotation.x = -Math.sin(a) * 0.1;
    g.add(leg);
  }
  g.userData = { kind: 'tripod', label: '三脚架' };
  return g;
}

// 石棉网
export function makeWireGauze() {
  const g = new THREE.Group();
  const mat = makeBox(0.5, 0.012, 0.5, makeStandardMaterial(0xcfc8b8, { roughness: 0.9 }));
  g.add(mat);
  g.userData = { kind: 'wire_gauze', label: '石棉网' };
  return g;
}

// 铁架台（带铁圈 + 双爪夹）
export function makeStand() {
  const g = new THREE.Group();
  const base = makeBox(0.55, 0.04, 0.35, METAL(0x2e3338, 0.6));
  base.position.y = 0.02;
  const pole = makeCylinder(0.022, 0.022, 1.7, METAL(0x8b959e, 0.35), 12);
  pole.position.set(-0.2, 0.87, 0);
  g.add(base, pole);
  // 铁圈
  const ring = makeCylinder(0.11, 0.11, 0.012, METAL(0x555b62, 0.45), 20, true);
  const ringArm = makeBox(0.3, 0.025, 0.025, METAL(0x555b62, 0.45));
  ring.position.set(0.12, 0.75, 0);
  ringArm.position.set(-0.05, 0.75, 0);
  // 夹爪
  const clawArm = makeBox(0.3, 0.03, 0.03, METAL(0x555b62, 0.45));
  clawArm.position.set(-0.05, 1.15, 0);
  const clawL = makeBox(0.1, 0.02, 0.06, METAL(0x666c73, 0.4));
  clawL.position.set(0.14, 1.15, 0.05);
  const clawR = clawL.clone(); clawR.position.z = -0.05;
  g.add(ring, ringArm, clawArm, clawL, clawR);
  g.userData = { kind: 'stand', label: '铁架台（带铁圈和夹）', gripAnchor: new THREE.Vector3(0.12, 1.15, 0), ringAnchor: new THREE.Vector3(0.12, 0.75, 0) };
  ring.userData.isRingSupport = true;
  return g;
}

// 试管夹
export function makeTestTubeClamp() {
  const g = new THREE.Group();
  const handle = makeCylinder(0.014, 0.014, 0.28, makeStandardMaterial(0x8a6a4a, { roughness: 0.7 }), 10);
  handle.rotation.z = Math.PI / 2;
  const jaw = makeBox(0.06, 0.025, 0.06, makeMetalMaterial(0x999fA6, 0.4));
  jaw.position.x = 0.17;
  g.add(handle, jaw);
  g.userData = { kind: 'tube_clamp', label: '试管夹' };
  return g;
}

// 试管架
export function makeTestTubeRack() {
  const g = new THREE.Group();
  const woodMat = makeStandardMaterial(0x9a7b55, { roughness: 0.85 });
  const top = makeBox(0.7, 0.03, 0.16, woodMat);
  top.position.y = 0.3;
  const bottom = makeBox(0.7, 0.03, 0.2, woodMat);
  bottom.position.y = 0.05;
  g.add(top, bottom);
  for (let i = 0; i < 2; i++) {
    const side = makeBox(0.03, 0.32, 0.16, woodMat);
    side.position.set(-0.335 + i * 0.67, 0.19, 0);
    g.add(side);
  }
  for (let i = 0; i < 6; i++) {
    const hole = makeCylinder(0.052, 0.052, 0.035, makeStandardMaterial(0x000000, { transparent: true, opacity: 0.4 }), 12, true);
    hole.position.set(-0.25 + i * 0.1, 0.305, 0);
    g.add(hole);
  }
  g.userData = { kind: 'tube_rack', label: '试管架', slots: 6, rackAnchor: new THREE.Vector3(0, 0.32, 0) };
  return g;
}

// 温度计
export function makeThermometer() {
  const g = new THREE.Group();
  const stem = makeCylinder(0.014, 0.014, 0.4, GLASS(), 12, true);
  stem.position.y = 0.2;
  const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.024, 12, 10), GLASS());
  bulb.position.y = -0.012;
  const mercury = makeCylinder(0.006, 0.006, 0.3, makeStandardMaterial(0xcc3333), 8);
  mercury.position.y = 0.22;
  g.add(stem, bulb, mercury);
  g.userData = { kind: 'thermometer', label: '温度计（-10~110℃）', isThermometer: true };
  return g;
}

// 玻璃棒
export function makeGlassRod() {
  const g = new THREE.Group();
  const rod = makeCylinder(0.012, 0.012, 0.5, GLASS(), 10, true);
  rod.position.y = 0.25;
  g.add(rod);
  g.userData = { kind: 'glass_rod', label: '玻璃棒', stirrable: true };
  return g;
}

// 药匙
export function makeSpatula() {
  const g = new THREE.Group();
  const handle = makeCylinder(0.012, 0.012, 0.26, makeMetalMaterial(0xb8bec4, 0.3), 10);
  handle.rotation.z = Math.PI / 2;
  const scoop = makeLathe(lathePoints([[0.001, 0], [0.035, 0.008], [0.045, 0.02]]), makeMetalMaterial(0xc8ced4, 0.25), 20);
  scoop.rotation.z = -Math.PI / 2;
  scoop.position.x = 0.17;
  g.add(handle, scoop);
  g.userData = { kind: 'spatula', label: '药匙', solidCarrier: true };
  return g;
}

// 表面皿式 pH 计 / 电子台秤 / 加热磁力搅拌器 —— 台面设备
export function makeMagneticStirrer() {
  const g = new THREE.Group();
  const body = makeBox(0.55, 0.14, 0.4, makeStandardMaterial(0x2f3540, { roughness: 0.5, metalness: 0.3 }));
  body.position.y = 0.07;
  const topPlate = makeBox(0.5, 0.015, 0.36, makeStandardMaterial(0xe8e2d5, { roughness: 0.3 }));
  topPlate.position.y = 0.148;
  const knob = makeCylinder(0.045, 0.045, 0.03, makeStandardMaterial(0x4a90d9, { roughness: 0.4 }), 20);
  knob.position.set(0.12, 0.16, 0.12);
  const knob2 = makeCylinder(0.045, 0.045, 0.03, makeStandardMaterial(0xd94a4a, { roughness: 0.4 }), 20);
  knob2.position.set(-0.12, 0.16, 0.12);
  const disp = makeBox(0.22, 0.07, 0.015, makeStandardMaterial(0x112233, { emissive: 0x0a2a4a, emissiveIntensity: 0.9 }));
  disp.position.set(0, 0.09, 0.205);
  g.add(body, topPlate, knob, knob2, disp);
  g.userData = { kind: 'magnetic_stirrer', label: '加热磁力搅拌器', stirring: false, heating: 0, stirAnchor: new THREE.Vector3(0, 0.16, 0) };
  return g;
}

// 磁力搅拌子
export function makeStirBar() {
  const g = new THREE.Group();
  const bar = makeCylinder(0.035, 0.035, 0.012, makeStandardMaterial(0xf0f4f8, { roughness: 0.3 }), 16);
  bar.rotation.z = Math.PI / 2;
  g.add(bar);
  g.userData = { kind: 'stir_bar', label: '磁力搅拌子', isStirBar: true };
  return g;
}

// 电子天平
export function makeBalance() {
  const g = new THREE.Group();
  const body = makeBox(0.5, 0.1, 0.42, makeStandardMaterial(0x3a4048, { roughness: 0.45, metalness: 0.25 }));
  body.position.y = 0.05;
  const pan = makeCylinder(0.17, 0.17, 0.012, makeMetalMaterial(0xd5dade, 0.2), 32);
  pan.position.y = 0.155;
  const disp = makeBox(0.24, 0.08, 0.015, makeStandardMaterial(0x0c1e14, { emissive: 0x0f5a2e, emissiveIntensity: 1.0 }));
  disp.position.set(0, 0.07, 0.215);
  g.add(body, pan, disp);
  g.userData = { kind: 'balance', label: '电子天平（0.001g）', isBalance: true, panAnchor: new THREE.Vector3(0, 0.161, 0) };
  pan.userData.isPanSupport = true;
  return g;
}

// pH 计（台式）
export function makePHMeter() {
  const g = new THREE.Group();
  const body = makeBox(0.34, 0.26, 0.1, makeStandardMaterial(0x333a44, { roughness: 0.5 }));
  body.position.y = 0.13;
  const disp = makeBox(0.26, 0.14, 0.012, makeStandardMaterial(0x0a1522, { emissive: 0x0d3a66, emissiveIntensity: 1.0 }));
  disp.position.set(0, 0.16, 0.052);
  const armPole = makeCylinder(0.015, 0.015, 0.4, makeMetalMaterial(0x9aa2ab, 0.3), 10);
  armPole.position.set(0.18, 0.2, 0);
  const arm = makeBox(0.26, 0.02, 0.02, makeMetalMaterial(0x9aa2ab, 0.3));
  arm.position.set(0.05, 0.4, 0);
  const electrode = makeCylinder(0.016, 0.016, 0.16, GLASS(), 10, true);
  electrode.position.set(-0.08, 0.32, 0);
  const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.022, 12, 10), GLASS());
  bulb.position.set(-0.08, 0.235, 0);
  g.add(body, disp, armPole, arm, electrode, bulb);
  g.userData = { kind: 'ph_meter', label: '台式 pH 计', isPHMeter: true, electrodeAnchor: new THREE.Vector3(-0.08, 0.22, 0) };
  return g;
}

// 洗瓶
export function makeWashBottle() {
  const g = new THREE.Group();
  const body = makeLathe(lathePoints([[0.01, 0], [0.19, 0.02], [0.19, 0.36], [0.09, 0.42]]), makeStandardMaterial(0xe8f2f8, { transparent: true, opacity: 0.75, roughness: 0.3 }), 32);
  const cap = makeCylinder(0.045, 0.045, 0.04, makeStandardMaterial(0x3a7bd5, { roughness: 0.4 }), 14);
  cap.position.y = 0.44;
  const straw = makeCylinder(0.012, 0.012, 0.3, makeStandardMaterial(0x3a7bd5), 10, true);
  straw.rotation.z = 0.9;
  straw.position.set(0.14, 0.52, 0);
  g.add(body, cap, straw);
  g.userData = { kind: 'wash_bottle', label: '洗瓶（蒸馏水）', capacity: 500, reagent: 'H2O', fillRadius: 0.16, fillBase: 0.03, fillMax: 0.32 };
  return g;
}

// 通用试剂瓶（可带颜色液体）
export function makeReagentBottle(name, color, capacity = 250) {
  const g = new THREE.Group();
  const pts = lathePoints([[0.01, 0], [0.16, 0.015], [0.16, 0.34], [0.09, 0.4], [0.085, 0.46]]);
  const body = makeLathe(pts, GLASS());
  const capMat = makeStandardMaterial(0x8a8f96, { roughness: 0.55 });
  const cap = makeCylinder(0.095, 0.095, 0.05, capMat, 18);
  cap.position.y = 0.485;
  // 标签
  const label = makeCylinder(0.165, 0.165, 0.14, makeStandardMaterial(0xfaf8f0, { roughness: 0.9 }), 20, true, 0);
  label.position.y = 0.18;
  g.add(body, cap, label);
  g.userData = { kind: 'reagent_bottle', label: name, capacity, reagentLabel: name, reagentColor: color, fillRadius: 0.14, fillBase: 0.04, fillMax: 0.3 };
  return g;
}

// 滴瓶（棕色）
export function makeDroppingBottle(name, color) {
  const g = new THREE.Group();
  const body = makeLathe(lathePoints([[0.008, 0], [0.1, 0.012], [0.1, 0.2], [0.06, 0.24]]), GLASS({ color: 0x6b4a2a, opacity: 0.5 }));
  const capBulb = new THREE.Mesh(new THREE.SphereGeometry(0.045, 14, 10), makeStandardMaterial(0x6b4a2a, { roughness: 0.6 }));
  capBulb.scale.y = 1.2;
  capBulb.position.y = 0.3;
  const pipette = makeCylinder(0.008, 0.008, 0.2, GLASS(), 8, true);
  pipette.position.y = 0.16;
  g.add(body, capBulb, pipette);
  g.userData = { kind: 'dropping_bottle', label: name, capacity: 30, reagentLabel: name, reagentColor: color, fillRadius: 0.085, fillBase: 0.02, fillMax: 0.17 };
  return g;
}

export const APPARATUS_FACTORIES = {
  beaker_50: () => makeBeaker(50), beaker_250: () => makeBeaker(250), beaker_500: () => makeBeaker(500),
  flask_conical_250: () => makeConicalFlask(250), flask_round_250: () => makeRoundFlask(250),
  flask_flat_250: () => makeFlatFlask(250), test_tube: makeTestTube,
  cylinder_100: () => makeMeasuringCylinder(100), flask_volumetric_100: () => makeVolumetricFlask(100),
  burette_50: () => makeBurette(50), pipette_10: () => makePipette(10), dropper: makeDropper,
  watch_glass: makeWatchGlass, evaporating_dish: makeEvaporatingDish, crucible: makeCrucible,
  mortar: makeMortar, funnel: makeFunnel, filter_flask: makeFilterFlask, buchner_funnel: makeBuchnerFunnel,
  gas_jar: makeGasJar, desiccator: makeDesiccator, alcohol_lamp: makeAlcoholLamp,
  tripod: makeTripod, wire_gauze: makeWireGauze, stand: makeStand, tube_clamp: makeTestTubeClamp,
  tube_rack: makeTestTubeRack, thermometer: makeThermometer, glass_rod: makeGlassRod,
  spatula: makeSpatula, magnetic_stirrer: makeMagneticStirrer, stir_bar: makeStirBar,
  balance: makeBalance, ph_meter: makePHMeter, wash_bottle: makeWashBottle
};

// 工具：将一个容器加满/部分填充液体（化学引擎用）
export function setContainerLiquid(container, volume, color, opts = {}) {
  const ud = container.userData;
  if (!ud || ud.fillRadius === undefined) return null;
  const ratio = Math.min(1, volume / ud.capacity);
  const h = Math.max(0.001, (ud.fillMax - ud.fillBase) * ratio);
  if (!ud._liquidMesh) {
    const geo = new THREE.CylinderGeometry(ud.fillRadius, ud.fillRadius * 0.92, 1, 24, 1);
    ud._liquidMesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ transparent: true, opacity: 0.95, roughness: 0.2 }));
    ud._liquidMesh.renderOrder = 5;
    container.add(ud._liquidMesh);
  }
  const lm = ud._liquidMesh;
  lm.scale.y = h;
  lm.position.y = ud.fillBase + h / 2;
  lm.material.color.set(color);
  lm.material.emissive = new THREE.Color(color).multiplyScalar(0.25);
  if (ud.coneFill || ud.sphereFill) {
    // 锥形/球形区近似的半径缩放
    const t = ratio;
    lm.scale.x = lm.scale.z = 0.55 + 0.45 * Math.min(1, t * 1.8);
  }
  lm.visible = volume > 0.5;
  return lm;
}
