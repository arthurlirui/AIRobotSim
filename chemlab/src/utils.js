import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ============ 基础工具 ============
const _color = (c) => new THREE.Color(c);

export function makeStandardMaterial(color, opts = {}) {
  return new THREE.MeshStandardMaterial({ color: _color(color), roughness: 0.7, metalness: 0.05, ...opts });
}

export function makeGlassMaterial(opts = {}) {
  return new THREE.MeshPhysicalMaterial({
    color: 0xf4fbff,
    transparent: true,
    opacity: 0.35,
    roughness: 0.08,
    metalness: 0,
    transmission: 0.55,
    thickness: 0.15,
    ior: 1.5,
    side: THREE.DoubleSide,
    depthWrite: false,
    ...opts
  });
}

// 瓷器材质（白色陶瓷）
export function makePorcelainMaterial(color = 0xfdfdfa) {
  return new THREE.MeshStandardMaterial({ color: _color(color), roughness: 0.25, metalness: 0.02 });
}

// 金属材质
export function makeMetalMaterial(color = 0x9aa4ad, roughness = 0.35) {
  return new THREE.MeshStandardMaterial({ color: _color(color), roughness, metalness: 0.85 });
}

// Lathe（旋转体）快捷方式：points 为 [ [r, y], ... ]
export function lathePoints(arr, segments = 48) {
  return arr.map(([r, y]) => new THREE.Vector2(r, y));
}

export function makeLathe(pts, material, segments = 48) {
  const geo = new THREE.LatheGeometry(pts, segments);
  return new THREE.Mesh(geo, material);
}

// 圆柱快捷
export function makeCylinder(rTop, rBottom, h, material, seg = 32, open = false) {
  const geo = new THREE.CylinderGeometry(rTop, rBottom, h, seg, 1, open);
  const m = new THREE.Mesh(geo, material);
  return m;
}

export function makeBox(w, h, d, material) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), material);
  return m;
}

// 将 mesh 顶点按轮廓函数变形（用于酒瓶/试剂瓶等）
export function taperMesh(mesh, fn) {
  const pos = mesh.geometry.attributes.position;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    const s = fn(y);
    pos.setX(i, x * s); pos.setZ(i, z * s);
  }
  pos.needsUpdate = true;
  mesh.geometry.computeVertexNormals();
}

export { THREE, OrbitControls };
