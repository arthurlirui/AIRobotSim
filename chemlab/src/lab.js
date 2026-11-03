import * as THREE from 'three';
import { makeStandardMaterial, makeGlassMaterial, makeMetalMaterial, makeCylinder, makeBox } from './utils.js';

// ============ 实验室房间与固定设施 ============
// 设计依据调研要点：
//  - 功能分区：中央实验台（双面操作）、通风橱靠墙（面风速 0.5m/s，微负压）、
//    试剂柜（防盗防腐蚀）、洗涤区（水池）、应急设施（洗眼器/灭火器/急救箱/紧急喷淋）
//  - 照明：天花板 LED 面板 + 紫外安全灯；地面：耐酸碱 PVC/环氧地坪
//  - 气流组织：顶部送风、通风橱+万向排风罩局部排风

export function buildRoom(scene) {
  const room = new THREE.Group();
  const W = 16, D = 12, H = 4.2;

  // 地面（环氧树脂地坪，浅灰绿）
  const floorMat = makeStandardMaterial(0x9aa8a0, { roughness: 0.6, metalness: 0.05 });
  const floor = new THREE.Mesh(new THREE.BoxGeometry(W, 0.1, D), floorMat);
  floor.position.y = -0.05;
  floor.receiveShadow = true;
  room.add(floor);

  // 墙壁（浅色乳胶漆）
  const wallMat = makeStandardMaterial(0xe8ecee, { roughness: 0.9 });
  const backWall = new THREE.Mesh(new THREE.BoxGeometry(W, H, 0.12), wallMat);
  backWall.position.set(0, H / 2, -D / 2 - 0.06);
  const leftWall = new THREE.Mesh(new THREE.BoxGeometry(0.12, H, D), wallMat);
  leftWall.position.set(-W / 2 - 0.06, H / 2, 0);
  const rightWall = new THREE.Mesh(new THREE.BoxGeometry(0.12, H, D), wallMat);
  rightWall.position.set(W / 2 + 0.06, H / 2, 0);
  room.add(backWall, leftWall, rightWall);

  // 踢脚线（深色耐腐蚀）
  const skirtMat = makeStandardMaterial(0x4a5560, { roughness: 0.5 });
  const skirt1 = makeBox(W, 0.14, 0.03, skirtMat); skirt1.position.set(0, 0.07, -D / 2 + 0.02);
  const skirt2 = makeBox(0.03, 0.14, D, skirtMat); skirt2.position.set(-W / 2 + 0.02, 0.07, 0);
  const skirt3 = makeBox(0.03, 0.14, D, skirtMat); skirt3.position.set(W / 2 - 0.02, 0.07, 0);
  room.add(skirt1, skirt2, skirt3);

  // 天花板 + LED 面板灯
  const ceilMat = makeStandardMaterial(0xf4f6f7, { roughness: 0.95 });
  const ceil = new THREE.Mesh(new THREE.BoxGeometry(W, 0.1, D), ceilMat);
  ceil.position.y = H + 0.05;
  room.add(ceil);
  const lights = [];
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 3; j++) {
      const panel = makeBox(1.6, 0.05, 0.5, new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xf0f8ff, emissiveIntensity: 1.6 }));
      panel.position.set(-5.2 + i * 3.5, H - 0.03, -3.8 + j * 3.8);
      room.add(panel);
      lights.push(panel);
    }
  }

  // 窗户（左侧墙，自然采光）
  const winMat = makeGlassMaterial({ opacity: 0.18 });
  const win = makeBox(0.06, 1.6, 3.6, winMat);
  win.position.set(-W / 2 + 0.04, 2.4, 0);
  room.add(win);
  const winFrame = makeBox(0.1, 1.7, 3.8, makeStandardMaterial(0xb8bec4, { roughness: 0.4, metalness: 0.5 }));
  winFrame.position.set(-W / 2 + 0.03, 2.4, 0);
  room.add(winFrame);

  // 门（右侧墙）
  const door = makeBox(0.08, 2.3, 1.1, makeStandardMaterial(0x6a7480, { roughness: 0.5, metalness: 0.3 }));
  door.position.set(W / 2 - 0.05, 1.15, 2.8);
  room.add(door);
  const doorSign = makeBox(0.02, 0.25, 0.5, makeStandardMaterial(0x2a7a3a, { emissive: 0x0a3a12, emissiveIntensity: 0.6 }));
  doorSign.position.set(W / 2 - 0.1, 2.1, 2.8);
  room.add(doorSign);

  scene.add(room);
  return { room, lights, dims: { W, D, H } };
}

// 实验台面（耐腐蚀理化板，黑色）
export function makeBenchTop(w, d) {
  const top = makeBox(w, 0.06, d, makeStandardMaterial(0x1e2226, { roughness: 0.35, metalness: 0.1 }));
  top.castShadow = true; top.receiveShadow = true;
  return top;
}

// 中央实验台（岛式，带试剂架层和架子）
export function buildCentralBench() {
  const g = new THREE.Group();
  const top = makeBenchTop(5.2, 1.8);
  top.position.y = 0.92;
  const bodyMat = makeStandardMaterial(0xd8dde2, { roughness: 0.6 });
  const body = makeBox(5.1, 0.86, 1.7, bodyMat);
  body.position.y = 0.46;
  g.add(top, body);
  // 抽屉线条
  for (let i = 0; i < 4; i++) {
    const drawer = makeBox(1.2, 0.18, 0.02, makeStandardMaterial(0xc0c6cc, { roughness: 0.5, metalness: 0.2 }));
    drawer.position.set(-1.9 + i * 1.27, 0.5, 0.87);
    g.add(drawer);
    const knob = makeCylinder(0.02, 0.02, 0.03, makeMetalMaterial(0x8a929a, 0.3), 10);
    knob.rotation.x = Math.PI / 2;
    knob.position.set(-1.9 + i * 1.27, 0.5, 0.89);
    g.add(knob);
  }
  // 试剂架（上层，带防倒杆）
  const shelfMat = makeStandardMaterial(0x2e343a, { roughness: 0.4 });
  const shelf = makeBox(4.6, 0.04, 0.5, shelfMat);
  shelf.position.y = 1.75;
  g.add(shelf);
  for (let i = 0; i < 2; i++) {
    const post = makeCylinder(0.025, 0.025, 0.86, makeMetalMaterial(0x5a626a, 0.35), 10);
    post.position.set(-2.2 + i * 4.4, 1.33, -0.2);
    g.add(post);
  }
  // 台面上吸水纸/操作垫
  const pad = makeBox(1.4, 0.008, 1.0, makeStandardMaterial(0x3a4a5a, { roughness: 0.9 }));
  pad.position.set(0.6, 0.956, 0.2);
  g.add(pad);
  g.userData = { kind: 'bench', label: '中央实验台', benchSurfaceY: 0.95, isBench: true };
  return g;
}

// 靠墙边台（放置大型仪器）
export function buildSideBench() {
  const g = new THREE.Group();
  const top = makeBenchTop(6.4, 0.9);
  top.position.y = 0.92;
  const body = makeBox(6.3, 0.86, 0.8, makeStandardMaterial(0xd8dde2, { roughness: 0.6 }));
  body.position.y = 0.46;
  g.add(top, body);
  // 下方柜门
  for (let i = 0; i < 5; i++) {
    const cab = makeBox(1.16, 0.7, 0.02, makeStandardMaterial(0xc0c6cc, { roughness: 0.5, metalness: 0.2 }));
    cab.position.set(-2.52 + i * 1.26, 0.48, 0.41);
    g.add(cab);
  }
  g.userData = { kind: 'bench', label: '仪器边台', benchSurfaceY: 0.95, isBench: true };
  return g;
}

// 通风橱（fume hood）—— 关键安全设施
export function buildFumeHood() {
  const g = new THREE.Group();
  const frameMat = makeStandardMaterial(0x8a95a0, { roughness: 0.4, metalness: 0.4 });
  const innerMat = makeStandardMaterial(0xdfe6ec, { roughness: 0.5 });
  // 外壳
  const shell = makeBox(2.8, 2.6, 1.5, makeStandardMaterial(0xc8d0d8, { roughness: 0.5 }));
  shell.position.set(0, 1.3, 0.35);
  // 工作腔（挖空视觉：内衬浅色盒体）
  const cavity = makeBox(2.4, 1.7, 1.1, innerMat);
  cavity.position.set(0, 1.15, 0.1);
  // 操作面板（透明滑动门）
  const sashMat = makeGlassMaterial({ opacity: 0.22 });
  const sash = makeBox(2.4, 1.0, 0.04, sashMat);
  sash.position.set(0, 1.25, -0.45);
  sash.userData.isFumeSash = true;
  // 拉手
  const handle = makeCylinder(0.03, 0.03, 1.4, makeMetalMaterial(0x4a525a, 0.3), 12);
  handle.rotation.z = Math.PI / 2;
  handle.position.set(0, 0.72, -0.47);
  // 台面（耐酸碱）
  const top = makeBenchTop(2.6, 1.3);
  top.position.set(0, 0.92, 0.1);
  // 排风管
  const duct = makeCylinder(0.3, 0.3, 1.4, makeMetalMaterial(0x9aa4ae, 0.5), 20, true);
  duct.position.set(0, 3.0, 0.35);
  // 控制面板（面风速显示）
  const panel = makeBox(0.5, 0.28, 0.04, makeStandardMaterial(0x22282e, { roughness: 0.4 }));
  panel.position.set(1.1, 1.9, -0.42);
  const panelDisp = makeBox(0.36, 0.14, 0.01, makeStandardMaterial(0x0a1a2a, { emissive: 0x0d4a7a, emissiveIntensity: 1.2 }));
  panelDisp.position.set(1.1, 1.9, -0.445);
  // 警示标签
  const label = makeBox(0.4, 0.2, 0.01, makeStandardMaterial(0xd8b020, { roughness: 0.6 }));
  label.position.set(-1.0, 1.9, -0.42);
  g.add(shell, cavity, sash, handle, top, duct, panel, panelDisp, label);
  g.userData = { kind: 'fume_hood', label: '通风橱（面风速 0.5 m/s · 微负压）', isFumeHood: true, benchSurfaceY: 0.95, sash, fanOn: true };
  return g;
}

// 通风系统：吊顶排风管道 + 万向排风罩
export function buildVentilation() {
  const g = new THREE.Group();
  const ductMat = makeMetalMaterial(0xa8b2ba, 0.5);
  // 主排风管
  const main = makeBox(0.6, 0.4, 12, ductMat);
  main.position.set(6.8, 3.9, 0);
  g.add(main);
  // 送风管
  const supply = makeBox(0.5, 0.35, 12, ductMat);
  supply.position.set(-6.8, 3.95, 0);
  g.add(supply);
  // 万向排风罩（snorkel）× 2
  const snorkels = [];
  for (let i = 0; i < 2; i++) {
    const s = new THREE.Group();
    const arm1 = makeCylinder(0.06, 0.06, 1.2, ductMat, 12, true);
    arm1.rotation.z = 0.5;
    arm1.position.set(-0.25, 0.55, 0);
    const arm2 = makeCylinder(0.05, 0.05, 1.0, ductMat, 12, true);
    arm2.rotation.z = -0.9;
    arm2.position.set(0.25, 1.15, 0);
    const hood = makeCylinder(0.3, 0.12, 0.35, ductMat, 18, true);
    hood.position.set(0.55, 1.45, 0);
    s.add(arm1, arm2, hood);
    s.position.set(3.0 + i * 3.0, 2.6, -4.4);
    snorkels.push(s);
    g.add(s);
  }
  g.userData = { kind: 'ventilation', label: '通风系统（局部排风+全面通风）' };
  return { group: g, snorkels };
}

// 试剂柜（带玻璃门、防腐、双人双锁危险品柜色标）
export function buildReagentCabinet() {
  const g = new THREE.Group();
  const bodyMat = makeStandardMaterial(0x9aa6b0, { roughness: 0.5, metalness: 0.2 });
  const body = makeBox(2.4, 2.2, 0.6, bodyMat);
  body.position.y = 1.1;
  // 三层隔板
  const shelfMat = makeStandardMaterial(0xe8ecef, { roughness: 0.6 });
  for (let i = 0; i < 3; i++) {
    const sh = makeBox(2.2, 0.03, 0.5, shelfMat);
    sh.position.set(0, 0.5 + i * 0.65, 0.03);
    g.add(sh);
  }
  // 玻璃门
  const doorMat = makeGlassMaterial({ opacity: 0.25 });
  for (let i = 0; i < 2; i++) {
    const d = makeBox(1.1, 2.0, 0.03, doorMat);
    d.position.set(-0.58 + i * 1.16, 1.1, 0.31);
    g.add(d);
  }
  g.add(body);
  // 标签条
  const tag = makeBox(1.0, 0.22, 0.02, makeStandardMaterial(0x2a6a9a, { roughness: 0.6 }));
  tag.position.set(0, 2.28, 0.31);
  g.add(tag);
  g.userData = { kind: 'reagent_cabinet', label: '试剂柜（防腐蚀·分类存放）' };
  return g;
}

// 危险化学品柜（黄色，双锁）
export function buildHazmatCabinet() {
  const g = new THREE.Group();
  const bodyMat = makeStandardMaterial(0xd8b020, { roughness: 0.55 });
  const body = makeBox(1.6, 1.9, 0.65, bodyMat);
  body.position.y = 0.95;
  const doorMat = makeStandardMaterial(0xc8a018, { roughness: 0.5 });
  const d1 = makeBox(0.76, 1.7, 0.03, doorMat); d1.position.set(-0.4, 0.95, 0.33);
  const d2 = makeBox(0.76, 1.7, 0.03, doorMat); d2.position.set(0.4, 0.95, 0.33);
  // 防爆通风口
  const vent = makeBox(1.2, 0.15, 0.02, makeStandardMaterial(0x2a2a2a, { roughness: 0.8 }));
  vent.position.set(0, 1.75, 0.34);
  g.add(body, d1, d2, vent);
  g.userData = { kind: 'hazmat_cabinet', label: '危险化学品柜（双人双锁·防爆通风）' };
  return g;
}

// 洗涤区：水池 + 水龙头 + 沥水架
export function buildWashStation() {
  const g = new THREE.Group();
  const bodyMat = makeStandardMaterial(0xd0d6da, { roughness: 0.5 });
  const body = makeBox(2.2, 0.86, 0.9, bodyMat);
  body.position.y = 0.43;
  const top = makeBenchTop(2.3, 1.0);
  top.position.y = 0.92;
  // 水池
  const sink = makeBox(1.2, 0.02, 0.7, makeStandardMaterial(0xf0f2f4, { roughness: 0.3 }));
  sink.position.y = 0.93;
  const basin = makeBox(1.1, 0.35, 0.6, makeStandardMaterial(0xf0f2f4, { roughness: 0.3 }));
  basin.position.y = 0.75;
  // 龙头
  const tapBase = makeCylinder(0.04, 0.05, 0.12, makeMetalMaterial(0xb8bec4, 0.25), 12);
  tapBase.position.set(0, 1.0, -0.32);
  const tapArm = makeCylinder(0.03, 0.03, 0.4, makeMetalMaterial(0xb8bec4, 0.25), 10, true);
  tapArm.rotation.x = Math.PI / 2;
  tapArm.position.set(0, 1.12, -0.15);
  const tapDown = makeCylinder(0.028, 0.028, 0.14, makeMetalMaterial(0xb8bec4, 0.25), 10, true);
  tapDown.position.set(0, 1.06, 0.05);
  g.add(body, top, sink, basin, tapBase, tapArm, tapDown);
  // 沥水架（旁边）
  const rack = new THREE.Group();
  const pegMat = makeStandardMaterial(0x5a90c8, { roughness: 0.5 });
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 4; j++) {
      const peg = makeCylinder(0.012, 0.012, 0.16, pegMat, 8);
      peg.rotation.x = Math.PI / 2;
      peg.position.set(0, 0, -0.12 + j * 0.08);
      const row = new THREE.Group(); row.add(peg);
      row.position.set(1.35 - 0.0, 0.95 + 0, 0);
      // 直接铺开
      peg.position.set(0, 0, -0.12 + j * 0.08);
      row.position.set(1.35 - 0.12 * i, 0.95 + 0.18 * i, 0.1);
      peg.rotation.x = Math.PI / 2;
      rack.add(row);
    }
  }
  g.add(rack);
  g.userData = { kind: 'wash_station', label: '洗涤区（水池+沥水架）' };
  return g;
}

// 安全设施：洗眼器 + 紧急喷淋
export function buildEmergencyShower() {
  const g = new THREE.Group();
  const pipeMat = makeMetalMaterial(0x4a9a4a, 0.35);
  const pole = makeCylinder(0.04, 0.04, 2.4, pipeMat, 12);
  pole.position.set(0, 1.2, 0);
  const arm = makeCylinder(0.035, 0.035, 0.5, pipeMat, 10, true);
  arm.rotation.z = Math.PI / 2;
  arm.position.set(0.25, 2.35, 0);
  const showerHead = makeCylinder(0.22, 0.12, 0.12, pipeMat, 18, true);
  showerHead.position.set(0.5, 2.3, 0);
  // 洗眼喷头
  const eyeArm = makeCylinder(0.03, 0.03, 0.3, pipeMat, 10, true);
  eyeArm.rotation.z = Math.PI / 2;
  eyeArm.position.set(0.15, 1.15, 0);
  const eyeBowl = makeCylinder(0.14, 0.16, 0.1, makeStandardMaterial(0x3a7a3a, { roughness: 0.4 }), 16);
  eyeBowl.position.set(0.32, 1.12, 0);
  // 拉环
  const ring = new THREE.Mesh(new THREE.TorusGeometry(0.07, 0.012, 8, 20), makeStandardMaterial(0xd84040, { roughness: 0.5 }));
  ring.position.set(-0.1, 2.0, 0.1);
  g.add(pole, arm, showerHead, eyeArm, eyeBowl, ring);
  g.userData = { kind: 'emergency_shower', label: '紧急喷淋与洗眼器' };
  return g;
}

// 灭火器（二氧化碳，红色瓶体）
export function buildFireExtinguisher() {
  const g = new THREE.Group();
  const body = makeCylinder(0.11, 0.11, 0.55, makeStandardMaterial(0xc03028, { roughness: 0.4, metalness: 0.3 }), 20);
  body.position.y = 0.32;
  const top = makeCylinder(0.045, 0.045, 0.1, makeMetalMaterial(0x8a929a, 0.3), 12);
  top.position.y = 0.64;
  const handle = makeBox(0.16, 0.025, 0.03, makeMetalMaterial(0x33383e, 0.4));
  handle.position.y = 0.7;
  const horn = makeCylinder(0.02, 0.05, 0.16, makeMetalMaterial(0x33383e, 0.4), 12);
  horn.rotation.z = 0.6;
  horn.position.set(0.12, 0.55, 0);
  g.add(body, top, handle, horn);
  g.userData = { kind: 'fire_extinguisher', label: 'CO₂ 灭火器' };
  return g;
}

// 急救箱
export function buildFirstAidKit() {
  const g = new THREE.Group();
  const body = makeBox(0.42, 0.3, 0.24, makeStandardMaterial(0xf4f4f4, { roughness: 0.6 }));
  body.position.y = 0.15;
  const cross1 = makeBox(0.14, 0.05, 0.005, makeStandardMaterial(0xc03028));
  cross1.position.set(0, 0.16, 0.125);
  const cross2 = makeBox(0.05, 0.14, 0.005, makeStandardMaterial(0xc03028));
  cross2.position.set(0, 0.16, 0.125);
  g.add(body, cross1, cross2);
  g.userData = { kind: 'first_aid', label: '急救箱' };
  return g;
}

// 气瓶柜（氮气/氧气钢瓶）
export function buildGasCylinder(color, label) {
  const g = new THREE.Group();
  const body = makeCylinder(0.18, 0.18, 1.35, makeStandardMaterial(color, { roughness: 0.4, metalness: 0.4 }), 20);
  body.position.y = 0.7;
  const shoulder = makeCylinder(0.08, 0.17, 0.14, makeStandardMaterial(color, { roughness: 0.4, metalness: 0.4 }), 20);
  shoulder.position.y = 1.44;
  const valve = makeCylinder(0.05, 0.05, 0.1, makeMetalMaterial(0x9aa2ab, 0.3), 12);
  valve.position.y = 1.55;
  const cap = makeCylinder(0.07, 0.07, 0.06, makeMetalMaterial(0x33383e, 0.4), 12);
  cap.position.y = 1.62;
  g.add(body, shoulder, valve, cap);
  g.userData = { kind: 'gas_cylinder', label };
  return g;
}

// 安全信息板 / 实验室规章制度牌
export function buildSafetyBoard() {
  const g = new THREE.Group();
  const board = makeBox(2.0, 1.2, 0.04, makeStandardMaterial(0x1a3a5a, { roughness: 0.6 }));
  board.position.y = 2.0;
  const title = makeBox(1.6, 0.18, 0.01, makeStandardMaterial(0xe8c840, { emissive: 0x6a5a10, emissiveIntensity: 0.4 }));
  title.position.set(0, 2.42, 0.025);
  g.add(board, title);
  g.userData = { kind: 'safety_board', label: '实验室安全守则' };
  return g;
}
