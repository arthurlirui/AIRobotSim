<script setup>
// Live simulation tab: scenario config -> /ws/sim live stream -> 3D viewport.
import { ref, reactive, computed, onBeforeUnmount } from 'vue'
import { useSimStore } from '../stores/sim'
import { fetchCatalog } from '../api'
import SimViewport from '../components/SimViewport.vue'

const store = useSimStore()
const catalog = ref(null)
fetchCatalog().then((c) => (catalog.value = c)).catch(() => {})

const cfg = reactive({
  nObstacles: 5,
  size: 10,
  terrain: 'procedural',
  difficulty: 0.5,
  robots: [
    { name: 'r0', type: 'wheeled', position: [-4, 0, 0] },
    { name: 'r1', type: 'wheeled', position: [-4, 2, 0] },
  ],
  useComm: true,
  algorithmKind: 'navigation',
  algorithmName: 'potential_field',
  goal: [4, 0, 0],
  stepsPerRequest: 1,
  speedHz: 10,
})

let timer = null
const running = ref(false)

const algoList = computed(() =>
  catalog.value?.algorithms[cfg.algorithmKind] ?? [])

function buildConfig() {
  const env = cfg.terrain === 'procedural'
    ? { procedural: { n_obstacles: cfg.nObstacles, size: cfg.size } }
    : { terrain: cfg.terrain,
        terrain_kwargs: { difficulty: cfg.difficulty } }
  const algos = []
  if (cfg.algorithmKind && cfg.algorithmName) {
    const ac = { kind: cfg.algorithmKind, name: cfg.algorithmName,
                 config: { robot_id: 'r0', goal: cfg.goal } }
    algos.push(ac)
  }
  return {
    env,
    robots: cfg.robots.map((r) => ({
      name: r.name, type: r.type,
      position: r.position, max_speed: 2.0 })),
    algorithms: algos,
    comm: cfg.useComm ? { channel: 'free_space' } : null,
    world: { dt: 0.1, seed: 0, lidar_rays: 36, lidar_range: 6.0 },
  }
}

function start() {
  store.connect(buildConfig())
  running.value = false
}
function stop() { store.disconnect() }
function playPause() {
  if (running.value) {
    clearInterval(timer); timer = null; running.value = false
  } else {
    timer = setInterval(
      () => store.stepOnce(cfg.stepsPerRequest), 1000 / cfg.speedHz)
    running.value = true
  }
}
function reset() { store.reset() }
onBeforeUnmount(() => { if (timer) clearInterval(timer) })

const metricRows = computed(() => {
  const f = store.frame
  if (!f || !f.algorithms) return []
  return Object.entries(f.algorithms).flatMap(([alg, m]) =>
    Object.entries(m).map(([k, v]) => ({
      name: `${alg.split(' ')[0]}.${k}`, value: v })))
})

function fmt(v) {
  return typeof v === 'number' ? v.toFixed(3) : String(v)
}
</script>

<template>
  <div class="sim-view">
    <aside class="config panel">
      <h3>场景配置</h3>
      <label>环境
        <select v-model="cfg.terrain">
          <option value="procedural">程序化障碍场景</option>
          <option v-for="t in catalog?.terrains ?? []" :key="t" :value="t">
            {{ t }}
          </option>
        </select>
      </label>
      <label v-if="cfg.terrain === 'procedural'">障碍数量
        <input type="range" v-model.number="cfg.nObstacles" min="0" max="20" />
        <span class="muted">{{ cfg.nObstacles }}</span>
      </label>
      <label v-else>难度
        <input type="range" v-model.number="cfg.difficulty" min="0" max="1" step="0.1" />
        <span class="muted">{{ cfg.difficulty }}</span>
      </label>
      <label>场地尺寸
        <input type="number" v-model.number="cfg.size" min="4" max="30" />
      </label>
      <label>启用通信网络
        <input type="checkbox" v-model="cfg.useComm" />
      </label>

      <h3 style="margin-top:14px">算法</h3>
      <label>类型
        <select v-model="cfg.algorithmKind">
          <option v-for="k in Object.keys(catalog?.algorithms ?? {})"
                  :key="k" :value="k">{{ k }}</option>
        </select>
      </label>
      <label>算法
        <select v-model="cfg.algorithmName">
          <option v-for="a in algoList" :key="a.name" :value="a.name">
            {{ a.name }}
          </option>
        </select>
      </label>
      <label v-if="cfg.algorithmKind === 'navigation'">目标点
        <div class="row">
          <input type="number" v-model.number="cfg.goal[0]" />
          <input type="number" v-model.number="cfg.goal[1]" />
          <input type="number" v-model.number="cfg.goal[2]" />
        </div>
      </label>

      <div class="actions">
        <button class="btn" @click="start"
                :disabled="store.status === 'connecting'">
          {{ store.world ? '重建场景' : '连接仿真' }}
        </button>
        <button class="btn ghost" @click="stop" v-if="store.ws">断开</button>
      </div>
      <p v-if="store.error" class="error">{{ store.error }}</p>
    </aside>

    <section class="stage">
      <SimViewport />
      <div class="playbar panel">
        <button class="btn" @click="playPause" :disabled="!store.world">
          {{ running ? '⏸ 暂停' : '▶ 播放' }}
        </button>
        <button class="btn ghost" @click="store.stepOnce(1)"
                :disabled="!store.world">单步</button>
        <button class="btn ghost" @click="reset" :disabled="!store.world">
          重置
        </button>
        <label>倍速
          <input type="range" v-model.number="cfg.speedHz" min="1" max="60" />
          <span class="muted">{{ cfg.speedHz }} Hz</span>
        </label>
        <span class="status" :data-s="store.status">{{ store.status }}</span>
        <span class="muted">t = {{ store.simTime.toFixed(2) }} s ·
          step {{ store.step }}</span>
      </div>
    </section>

    <aside class="metrics panel">
      <h3>实时指标</h3>
      <div v-if="store.world" class="world-info">
        <p v-for="r in store.world.robots" :key="r.id">
          <b>{{ r.id }}</b> <span class="muted">{{ r.kind }}</span>
        </p>
        <p class="muted">算法: {{ store.world.algorithms.join(', ') }}</p>
        <p class="muted">通信: {{ store.world.has_comm ? '已启用' : '无' }}</p>
      </div>
      <table v-if="metricRows.length">
        <tbody>
          <tr v-for="m in metricRows" :key="m.name">
            <td>{{ m.name }}</td>
            <td class="num">{{ fmt(m.value) }}</td>
          </tr>
        </tbody>
      </table>
      <p v-else class="muted">等待仿真帧…</p>
    </aside>
  </div>
</template>

<style scoped>
.sim-view {
  display: grid;
  grid-template-columns: 260px 1fr 240px;
  gap: 12px;
  height: 100%;
  padding: 12px;
}
.config { display: flex; flex-direction: column; gap: 8px; overflow-y: auto; }
.config label, .playbar label {
  display: flex; flex-direction: column; gap: 4px; font-size: 13px;
}
.config .row { display: flex; gap: 6px; }
.stage { display: flex; flex-direction: column; gap: 10px; min-width: 0; }
.stage > :first-child { flex: 1; min-height: 0; }
.playbar {
  display: flex; align-items: center; gap: 14px;
  flex-wrap: wrap; padding: 10px 14px;
}
.playbar label { flex-direction: row !important; align-items: center; gap: 8px; }
.status { font-weight: 600; }
.status[data-s="live"] { color: var(--ok); }
.status[data-s="error"] { color: #ff4d6d; }
.status[data-s="connecting"] { color: var(--warn); }
.metrics { overflow-y: auto; }
.metrics table { width: 100%; border-collapse: collapse; font-size: 12px; }
.metrics td { padding: 3px 4px; border-bottom: 1px solid var(--border); }
.metrics td.num { text-align: right; font-family: Consolas, monospace; }
.error { color: #ff4d6d; font-size: 12px; }
.actions { display: flex; gap: 8px; margin-top: 10px; }
.world-info p { margin: 4px 0; font-size: 13px; }
</style>
