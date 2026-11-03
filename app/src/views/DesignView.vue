<script setup>
// Robot design tab: pick a preset morphology, run the gait preview via REST,
// replay the FK link frames in a small Three.js viewport.
import { ref, onMounted, onBeforeUnmount, watch } from 'vue'
import * as THREE from 'three'
import { fetchDesignPresets, runDesignSimulate } from '../api'

const presets = ref([])
const selected = ref('')
const frames = ref([])
const busy = ref(false)
const error = ref('')
const playing = ref(false)
const frameIdx = ref(0)
const speed = ref(2)

let renderer, scene, camera, raf, el, group

onMounted(async () => {
  presets.value = (await fetchDesignPresets()).presets
  if (presets.value.length) selected.value = presets.value[0].name
  el = document.querySelector('.design-viewport')
  scene = new THREE.Scene()
  scene.background = new THREE.Color(0x0b0f16)
  camera = new THREE.PerspectiveCamera(
    50, el.clientWidth / el.clientHeight, 0.01, 50)
  camera.position.set(0.5, 0.4, 0.9)
  camera.lookAt(0, 0.1, 0)
  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setSize(el.clientWidth, el.clientHeight)
  renderer.setPixelRatio(devicePixelRatio)
  el.appendChild(renderer.domElement)
  scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x202833, 1.0))
  const grid = new THREE.GridHelper(2, 20, 0x2c3a55, 0x1a2130)
  scene.add(grid)
  group = new THREE.Group()
  scene.add(group)
  loop()
  watch(frames, rebuild)
})

function loop() {
  raf = requestAnimationFrame(loop)
  renderer?.render(scene, camera)
}
onBeforeUnmount(() => { cancelAnimationFrame(raf); renderer?.dispose() })

const links = []     // THREE.Mesh per link name

function rebuild(fs) {
  group.clear()
  links.length = 0
  if (!fs || !fs.length) return
  for (const name of Object.keys(fs[0].links)) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(0.015),
      new THREE.MeshLambertMaterial({ color: 0x4d9fff }))
    group.add(m)
    links.push([name, m])
  }
  applyFrame(0)
}

function applyFrame(i) {
  const f = frames.value[i]
  if (!f) return
  for (const [name, mesh] of links) {
    const L = f.links[name]
    if (!L) continue
    mesh.position.set(L.position[0], L.position[2], L.position[1])
  }
  const b = f.base
  if (group && b) group.position.set(0, 0, 0)
}

let timer = null
watch(playing, (p) => {
  clearInterval(timer)
  if (p) {
    timer = setInterval(() => {
      frameIdx.value = (frameIdx.value + 1) % Math.max(1, frames.value.length)
      applyFrame(frameIdx.value)
    }, 1000 / speed.value)
  }
})

async function run() {
  busy.value = true; error.value = ''
  try {
    const r = await runDesignSimulate({
      preset: selected.value, steps: 120, dt: 0.05 })
    frames.value = r.frames
    frameIdx.value = 0
    rebuild(r.frames)
  } catch (e) { error.value = String(e) }
  busy.value = false
}
</script>

<template>
  <div class="design-view">
    <aside class="panel">
      <h3>构型预设</h3>
      <label v-for="p in presets" :key="p.name" class="pre">
        <input type="radio" :value="p.name" v-model="selected" />
        <div><b>{{ p.name }}</b></div>
      </label>
      <button class="btn" @click="run" :disabled="busy || !selected">
        {{ busy ? '仿真中…' : '▶ 步态预览' }}
      </button>
      <p v-if="error" class="error">{{ error }}</p>
      <template v-if="frames.length">
        <label style="margin-top:14px">回放
          <input type="range" v-model.number="frameIdx" min="0"
                 :max="frames.length - 1"
                 @input="applyFrame(frameIdx)" />
          <span class="muted">{{ frameIdx }} / {{ frames.length - 1 }}</span>
        </label>
        <label>速度
          <input type="range" v-model.number="speed" min="1" max="30" />
        </label>
        <button class="btn ghost" @click="playing = !playing">
          {{ playing ? '⏸ 暂停' : '▶ 播放' }}
        </button>
      </template>
    </aside>
    <section class="panel stage-panel">
      <div class="design-viewport" />
    </section>
  </div>
</template>

<style scoped>
.design-view {
  display: grid; grid-template-columns: 260px 1fr;
  gap: 12px; height: 100%; padding: 12px;
}
aside { overflow-y: auto; display: flex; flex-direction: column; gap: 8px; }
aside label { display: flex; flex-direction: column; gap: 4px; font-size: 13px; }
.pre { flex-direction: row !important; align-items: center; gap: 8px;
       padding: 6px 8px; border-radius: 6px; cursor: pointer; }
.pre:hover { background: var(--panel-2); }
.stage-panel { min-height: 0; padding: 0; overflow: hidden;
               display: flex; }
.design-viewport { flex: 1; }
.error { color: #ff4d6d; font-size: 12px; }
</style>
