<script setup>
// Three.js viewport: renders the terrain (splats), robots, lidar scans and
// comm links from the live sim store. OrbitControls-like camera implemented
// inline to keep the vendor surface to `three` only.
import { onMounted, onBeforeUnmount, watch, ref } from 'vue'
import * as THREE from 'three'
import { useSimStore } from '../stores/sim'

const store = useSimStore()
const el = ref(null)

let renderer, scene, camera, raf
let terrainGroup, robotGroup, lidarGroup, commGroup
let drag = null

const ROBOT_COLORS = [0x4d9fff, 0xff8a4d, 0x3fd08a, 0xff4d6d, 0xc04dff]

onMounted(() => {
  scene = new THREE.Scene()
  scene.background = new THREE.Color(0x0b0f16)
  scene.fog = new THREE.Fog(0x0b0f16, 30, 80)

  camera = new THREE.PerspectiveCamera(
    55, el.value.clientWidth / el.value.clientHeight, 0.1, 200)
  camera.position.set(12, 10, 12)
  camera.lookAt(0, 0, 0)

  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setSize(el.value.clientWidth, el.value.clientHeight)
  renderer.setPixelRatio(devicePixelRatio)
  el.value.appendChild(renderer.domElement)

  scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x202833, 1.0))
  const dir = new THREE.DirectionalLight(0xffffff, 0.8)
  dir.position.set(5, 10, 3)
  scene.add(dir)

  // ground grid for spatial reference
  const grid = new THREE.GridHelper(30, 30, 0x2c3a55, 0x1a2130)
  grid.position.y = -0.01
  scene.add(grid)

  terrainGroup = new THREE.Group()
  robotGroup = new THREE.Group()
  lidarGroup = new THREE.Group()
  commGroup = new THREE.Group()
  scene.add(terrainGroup, robotGroup, lidarGroup, commGroup)

  // simple orbit camera
  const dom = renderer.domElement
  dom.addEventListener('mousedown', (e) => {
    drag = { x: e.clientX, y: e.clientY,
             px: camera.position.clone() }
  })
  window.addEventListener('mousemove', onMouseMove)
  window.addEventListener('mouseup', () => (drag = null))
  dom.addEventListener('wheel', onWheel)

  window.addEventListener('resize', onResize)
  loop()
})

function onMouseMove(e) {
  if (!drag) return
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y
  const p = drag.px
  const theta = Math.atan2(p.z, p.x) - dx * 0.01
  const r = Math.hypot(p.x, p.z)
  camera.position.x = Math.cos(theta) * r
  camera.position.z = Math.sin(theta) * r
  camera.position.y = Math.max(1, p.y + dy * 0.05)
  camera.lookAt(0, 0, 0)
}

function onWheel(e) {
  e.preventDefault()
  const dirv = camera.position.clone().normalize()
  camera.position.addScaledVector(
    dirv, e.deltaY > 0 ? 1.5 : -1.5)
  if (camera.position.length() < 3)
    camera.position.setLength(3)
  camera.lookAt(0, 0, 0)
}

function onResize() {
  if (!el.value) return
  camera.aspect = el.value.clientWidth / el.value.clientHeight
  camera.updateProjectionMatrix()
  renderer.setSize(el.value.clientWidth, el.value.clientHeight)
}

function loop() {
  raf = requestAnimationFrame(loop)
  renderer.render(scene, camera)
}

onBeforeUnmount(() => {
  cancelAnimationFrame(raf)
  renderer?.dispose()
})

// ---- reactive scene updates -------------------------------------------

watch(() => store.world, (w) => {
  terrainGroup.clear()
  if (!w) return
  for (const s of w.terrain.splats) {
    const geo = new THREE.BoxGeometry(
      Math.max(0.1, s.scale[0] * 2), Math.max(0.05, s.scale[2] * 2),
      Math.max(0.1, s.scale[1] * 2))
    const mat = new THREE.MeshLambertMaterial({
      color: s.position[2] > 0.2 ? 0x5c6b85 : 0x2c3a55,
      transparent: true, opacity: 0.35 + 0.5 * s.opacity,
    })
    const m = new THREE.Mesh(geo, mat)
    m.position.set(s.position[0], s.position[2] + w.terrain.ground_level,
                   s.position[1])
    terrainGroup.add(m)
  }
})

watch(() => store.frame, (f) => {
  if (!f) return
  // robots
  const ids = Object.keys(f.robots)
  robotGroup.clear()
  for (let i = 0; i < ids.length; i++) {
    const r = f.robots[ids[i]]
    const color = ROBOT_COLORS[i % ROBOT_COLORS.length]
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.5, 0.3, 0.5),
      new THREE.MeshLambertMaterial({ color }))
    const head = new THREE.Mesh(
      new THREE.ConeGeometry(0.14, 0.3, 8),
      new THREE.MeshLambertMaterial({ color: 0xffffff }))
    head.position.set(0.25, 0.02, 0)
    head.rotation.z = -Math.PI / 2
    const g = new THREE.Group()
    g.add(body, head)
    g.position.set(r.position[0], r.position[2] + 0.25, r.position[1])
    g.rotation.y = -r.yaw
    // id label
    const sprite = makeLabel(ids[i], color)
    sprite.position.set(0, 0.6, 0)
    g.add(sprite)
    robotGroup.add(g)
  }
  // lidar rays
  lidarGroup.clear()
  for (const rid of ids) {
    const scan = f.lidar[rid]
    if (!scan) continue
    const r = f.robots[rid]
    const ox = r.position[0], oy = r.position[2] + 0.2, oz = r.position[1]
    const pts = []
    for (const [ang, d] of scan.rays) {
      pts.push(new THREE.Vector3(ox, oy, oz))
      pts.push(new THREE.Vector3(
        ox + Math.cos(ang) * d, oy, oz + Math.sin(ang) * d))
    }
    const geo = new THREE.BufferGeometry().setFromPoints(pts)
    lidarGroup.add(new THREE.LineSegments(
      geo, new THREE.LineBasicMaterial({
        color: 0x3fd08a, transparent: true, opacity: 0.25 })))
  }
  // comm links
  commGroup.clear()
  if (f.comm_links && f.comm_links.length) {
    for (const l of f.comm_links) {
      if (!l.up) continue
      const a = f.robots[l.tx], b = f.robots[l.rx]
      if (!a || !b) continue
      const geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(a.position[0], a.position[2] + 0.4, a.position[1]),
        new THREE.Vector3(b.position[0], b.position[2] + 0.4, b.position[1]),
      ])
      commGroup.add(new THREE.Line(
        geo, new THREE.LineBasicMaterial({ color: 0xffb84d })))
    }
  }
})

function makeLabel(text, color) {
  const c = document.createElement('canvas')
  c.width = c.height = 128
  const ctx = c.getContext('2d')
  ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`
  ctx.font = 'bold 48px sans-serif'
  ctx.textAlign = 'center'
  ctx.fillText(text, 64, 78)
  const tex = new THREE.CanvasTexture(c)
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex }))
  sp.scale.set(0.8, 0.8, 1)
  return sp
}
</script>

<template>
  <div ref="el" class="viewport" />
</template>

<style scoped>
.viewport {
  width: 100%; height: 100%;
  border-radius: 8px;
  overflow: hidden;
}
.viewport :deep(canvas) { display: block; }
</style>
