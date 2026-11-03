// 场景管理：房间、地面、灯光、阴影、暗室模式
import * as THREE from 'three';

export const ZONES = {
  mechanics:  { name: '力学区',  color: 0xff9f43, x: -5.0, z: -2.6 },
  optics:     { name: '光学区',  color: 0xb57cff, x:  0.0, z: -4.6 },
  electricity:{ name: '电学区',  color: 0xffd93d, x:  5.0, z: -2.6 },
  waves:      { name: '波学区',  color: 0x4cc3ff, x: -4.6, z:  2.8 },
  thermal:    { name: '热学区',  color: 0xff6b6b, x:  4.6, z:  2.8 },
};

export const BENCH = { H: 0.78, W: 3.4, D: 1.9 }; // 台面高度/宽度/深度（真实实验室规范 0.75–0.85m）

export class LabScene {
  constructor() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0d1420);
    this.scene.fog = new THREE.Fog(0x0d1420, 22, 46);
    this.darkMode = false;
    this._build();
  }

  _build() {
    const s = this.scene;

    // 房间外壳 16m x 12m, 高 3.6m
    const ROOM = { W: 16, D: 12, H: 3.6 };
    this.room = ROOM;
    // 地板
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x8a6f52, roughness: 0.85 });
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(ROOM.W, ROOM.D), floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    s.add(floor);
    // 地板网格线（浅色环氧自流平接缝感）
    const grid = new THREE.GridHelper(ROOM.W, 16, 0x6b5640, 0x6b5640);
    grid.position.y = 0.002;
    grid.material.opacity = 0.25; grid.material.transparent = true;
    s.add(grid);

    // 墙壁（内表面）
    const wallMat = new THREE.MeshStandardMaterial({ color: 0xb8c4d4, roughness: 0.95, side: THREE.BackSide });
    const walls = new THREE.Mesh(new THREE.BoxGeometry(ROOM.W, ROOM.H, ROOM.D), wallMat);
    walls.position.y = ROOM.H / 2;
    s.add(walls);

    // 踢脚线 + 墙裙色带
    const skirtMat = new THREE.MeshStandardMaterial({ color: 0x39516e, roughness: .8 });
    const skirt = new THREE.Mesh(new THREE.BoxGeometry(ROOM.W - 0.02, 0.3, ROOM.D - 0.02), skirtMat);
    skirt.position.y = 0.15;
    s.add(skirt);

    // 窗户（后墙发光板，模拟自然采光）
    const winMat = new THREE.MeshStandardMaterial({ color: 0x9fc8ff, emissive: 0x86b8e8, emissiveIntensity: 0.85 });
    for (const wx of [-4.5, 0, 4.5]) {
      const win = new THREE.Mesh(new THREE.PlaneGeometry(3.0, 1.5), winMat);
      win.position.set(wx, 1.9, -ROOM.D / 2 + 0.02);
      s.add(win);
    }

    // 灯光
    this.ambLight = new THREE.AmbientLight(0xdfe8f5, 0.55);
    s.add(this.ambLight);
    this.hemi = new THREE.HemisphereLight(0xcfe2ff, 0x54442e, 0.5);
    s.add(this.hemi);

    this.ceilLights = [];
    for (const [lx, lz] of [[-4, -3], [0, -3], [4, -3], [-4, 3], [0, 3], [4, 3]]) {
      const pl = new THREE.PointLight(0xfff4e0, 0.5, 14, 1.6);
      pl.position.set(lx, 3.4, lz);
      s.add(pl);
      this.ceilLights.push(pl);
      // 灯管造型
      const tube = new THREE.Mesh(
        new THREE.CylinderGeometry(0.06, 0.06, 1.3, 8),
        new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xfff4e0, emissiveIntensity: 1.2 })
      );
      tube.rotation.z = Math.PI / 2;
      tube.position.set(lx, 3.45, lz);
      s.add(tube);
    }
    this.keyLight = new THREE.DirectionalLight(0xffffff, 0.85);
    this.keyLight.position.set(6, 9, 5);
    this.keyLight.castShadow = true;
    this.keyLight.shadow.mapSize.set(2048, 2048);
    this.keyLight.shadow.camera.left = -10; this.keyLight.shadow.camera.right = 10;
    this.keyLight.shadow.camera.top = 10; this.keyLight.shadow.camera.bottom = -10;
    this.keyLight.shadow.bias = -0.0004;
    s.add(this.keyLight);
    this.opticSpot = new THREE.SpotLight(0xffffff, 0.0, 9, 0.6, 0.5);
    this.opticSpot.position.set(ZONES.optics.x, 3.2, ZONES.optics.z);
    this.opticSpot.target.position.set(ZONES.optics.x, BENCH.H, ZONES.optics.z);
    s.add(this.opticSpot, this.opticSpot.target);

    this._buildBenches();
    this._buildSafetyProps();
  }

  _buildBenches() {
    const s = this.scene;
    this.zoneBenches = {};
    const legMat = new THREE.MeshStandardMaterial({ color: 0x3a4552, metalness: 0.6, roughness: 0.5 });
    const topMat = new THREE.MeshStandardMaterial({ color: 0x5d6d7e, roughness: 0.35 });

    for (const [key, z] of Object.entries(ZONES)) {
      const g = new THREE.Group();
      // 台面
      const top = new THREE.Mesh(new THREE.BoxGeometry(BENCH.W, 0.06, BENCH.D), topMat.clone());
      top.material.color.set(z.color).lerp(new THREE.Color(0x808b96), 0.72);
      top.position.y = BENCH.H - 0.03;
      top.castShadow = true; top.receiveShadow = true;
      g.add(top);
      // 台腿（四角 + 背板）
      for (const [ox, oz] of [[-1, -1], [1, -1], [-1, 1], [1, 1]].map(([a, b]) => [a * (BENCH.W / 2 - 0.12), b * (BENCH.D / 2 - 0.12)])) {
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.07, BENCH.H - 0.06, 0.07), legMat);
        leg.position.set(ox, (BENCH.H - 0.06) / 2, oz);
        leg.castShadow = true;
        g.add(leg);
      }
      // 学科色前沿条
      const strip = new THREE.Mesh(new THREE.BoxGeometry(BENCH.W, 0.07, 0.03),
        new THREE.MeshStandardMaterial({ color: z.color, emissive: z.color, emissiveIntensity: 0.35 }));
      strip.position.set(0, BENCH.H - 0.06, BENCH.D / 2 + 0.015);
      g.add(strip);

      // 悬挂学科标牌
      const sign = new THREE.Mesh(
        new THREE.BoxGeometry(1.5, 0.42, 0.05),
        new THREE.MeshStandardMaterial({ color: 0x18202e, emissive: z.color, emissiveIntensity: 0.28 })
      );
      sign.position.set(0, 2.35, 0);
      g.add(sign);
      const signText = makeTextSprite(z.name, `#${z.color.toString(16).padStart(6, '0')}`);
      signText.position.set(0, 2.35, 0.04);
      signText.scale.set(1.5, 0.42, 1);
      g.add(signText);

      // 地毯色带分区
      const mat = new THREE.Mesh(new THREE.PlaneGeometry(BENCH.W + 2.6, BENCH.D + 2.2),
        new THREE.MeshStandardMaterial({ color: z.color, transparent: true, opacity: 0.1, roughness: 1 }));
      mat.rotation.x = -Math.PI / 2;
      mat.position.y = 0.004;
      g.add(mat);

      g.position.set(z.x, 0, z.z);
      s.add(g);
      this.zoneBenches[key] = g;
    }
  }

  _buildSafetyProps() {
    const s = this.scene;
    const R = this.room;
    // 灭火器（红色圆筒）
    const ext = new THREE.Group();
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.5, 12),
      new THREE.MeshStandardMaterial({ color: 0xcc2222, roughness: 0.4 }));
    ext.add(body);
    const nozzle = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.18, 8),
      new THREE.MeshStandardMaterial({ color: 0x222222 }));
    nozzle.rotation.z = 0.5; nozzle.position.set(0.1, 0.22, 0);
    ext.add(nozzle);
    ext.position.set(-R.W / 2 + 0.45, 0.25, -R.D / 2 + 0.6);
    ext.castShadow = true;
    s.add(ext);

    // 急救箱（白盒红十字）
    const aid = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.3, 0.24),
      new THREE.MeshStandardMaterial({ color: 0xf2f2f2 }));
    aid.position.set(-R.W / 2 + 0.45, 1.5, -R.D / 2 + 0.15);
    const cross1 = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.055, 0.02), new THREE.MeshStandardMaterial({ color: 0xd42020 }));
    const cross2 = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.2, 0.02), new THREE.MeshStandardMaterial({ color: 0xd42020 }));
    cross1.position.z = cross2.position.z = 0.13;
    aid.add(cross1, cross2);
    s.add(aid);

    // 护目镜挂架
    const rack = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.08, 0.1),
      new THREE.MeshStandardMaterial({ color: 0x556677 }));
    rack.position.set(R.W / 2 - 0.5, 1.4, -R.D / 2 + 0.12);
    s.add(rack);
    for (let i = 0; i < 3; i++) {
      const gog = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.07, 0.06),
        new THREE.MeshStandardMaterial({ color: 0x99d6ff, transparent: true, opacity: 0.75 }));
      gog.position.set(R.W / 2 - 0.68 + i * 0.18, 1.32, -R.D / 2 + 0.16);
      s.add(gog);
    }

    // 前墙安全标语
    const sign = makeTextSprite('⚠ 安全操作 · 低压实验 · 护目镜', '#ffd93d', 26);
    sign.position.set(0, 2.5, -R.D / 2 + 0.05);
    sign.scale.set(4.4, 0.55, 1);
    s.add(sign);
    // 黑板（前演示墙）
    const board = new THREE.Mesh(new THREE.PlaneGeometry(5.4, 1.6),
      new THREE.MeshStandardMaterial({ color: 0x24473a, roughness: 1 }));
    board.position.set(-3.6, 1.7, R.D / 2 - 0.02);
    board.rotation.y = Math.PI;
    s.add(board);
    const chalk = makeTextSprite('F = ma    U = IR    T = 2π√(L/g)', '#e8eef7', 24);
    chalk.position.set(-3.6, 1.75, R.D / 2 - 0.06);
    chalk.rotation.y = Math.PI;
    chalk.scale.set(4.6, 0.6, 1);
    s.add(chalk);
  }

  setDarkMode(on) {
    this.darkMode = on;
    this.ambLight.intensity = on ? 0.05 : 0.55;
    this.hemi.intensity = on ? 0.04 : 0.5;
    this.keyLight.intensity = on ? 0.08 : 0.85;
    this.ceilLights.forEach(l => l.intensity = on ? 0.03 : 0.5);
    this.scene.children.forEach(c => {
      if (c.material?.emissive && c !== this.scene && c.userData.win) c.material.emissiveIntensity = on ? 0.15 : 0.85;
    });
    this.scene.fog = on ? new THREE.Fog(0x04060a, 10, 30) : new THREE.Fog(0x0d1420, 22, 46);
    this.scene.background = new THREE.Color(on ? 0x04060a : 0x0d1420);
    this.opticSpot.intensity = on ? 1.6 : 0.0;
  }
}

// 文字 Sprite（Canvas 纹理）
export function makeTextSprite(text, color = '#e8eef7', fontSize = 42) {
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  ctx.font = `bold ${fontSize}px "Microsoft YaHei", sans-serif`;
  const w = Math.ceil(ctx.measureText(text).width) + 24;
  canvas.width = Math.max(w, 64); canvas.height = fontSize + 22;
  const ctx2 = canvas.getContext('2d');
  ctx2.font = `bold ${fontSize}px "Microsoft YaHei", sans-serif`;
  ctx2.fillStyle = color;
  ctx2.textAlign = 'center'; ctx2.textBaseline = 'middle';
  ctx2.fillText(text, canvas.width / 2, canvas.height / 2);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const mat = new THREE.SpriteMaterial({ map: tex, transparent: true });
  const sp = new THREE.Sprite(mat);
  return sp;
}
