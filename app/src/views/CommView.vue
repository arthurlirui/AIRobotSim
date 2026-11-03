<script setup>
// Comm experiment tab: pick a preset experiment, run it via REST, show the
// resulting delivery / link records.
import { ref, onMounted } from 'vue'
import { fetchCommScenarios, runCommExperiment } from '../api'

const experiments = ref([])
const selected = ref('')
const result = ref(null)
const busy = ref(false)
const error = ref('')

onMounted(async () => {
  try {
    const r = await fetchCommScenarios()
    experiments.value = r.scenarios
    if (r.scenarios.length) selected.value = r.scenarios[0].name
  } catch (e) { error.value = String(e) }
})

async function run() {
  busy.value = true; error.value = ''; result.value = null
  try {
    result.value = await runCommExperiment(selected.value)
  } catch (e) { error.value = String(e) }
  busy.value = false
}

function entries(obj) {
  return Object.entries(obj ?? {}).filter(
    ([k, v]) => typeof v !== 'object' || v === null)
}
</script>

<template>
  <div class="comm-view">
    <aside class="panel">
      <h3>实验列表</h3>
      <label v-for="e in experiments" :key="e.name" class="exp">
        <input type="radio" :value="e.name" v-model="selected" />
        <div>
          <b>{{ e.name }}</b>
          <p class="muted">{{ e.description }}</p>
        </div>
      </label>
      <button class="btn" @click="run" :disabled="busy || !selected">
        {{ busy ? '运行中…' : '▶ 运行实验' }}
      </button>
      <p v-if="error" class="error">{{ error }}</p>
    </aside>

    <section class="panel result">
      <h3>实验结果</h3>
      <template v-if="result">
        <div class="kv-grid">
          <div v-for="[k, v] in entries(result)" :key="k" class="kv">
            <span class="muted">{{ k }}</span>
            <b>{{ v === null ? '—' : v }}</b>
          </div>
        </div>
        <details v-for="(val, key) in result" :key="key"
                 v-show="Array.isArray(val)">
          <summary>{{ key }} ({{ Array.isArray(val) ? val.length : 0 }})</summary>
          <pre>{{ JSON.stringify(val, null, 2) }}</pre>
        </details>
      </template>
      <p v-else class="muted">选择左侧实验并运行，结果将显示在这里。</p>
    </section>
  </div>
</template>

<style scoped>
.comm-view {
  display: grid; grid-template-columns: 320px 1fr;
  gap: 12px; height: 100%; padding: 12px;
}
.comm-view aside { overflow-y: auto; }
.exp { display: flex; gap: 10px; padding: 8px; border-radius: 6px;
       cursor: pointer; align-items: flex-start; }
.exp:hover { background: var(--panel-2); }
.exp p { margin: 2px 0 0; font-size: 12px; }
.result { overflow-y: auto; }
.kv-grid { display: grid; grid-template-columns: repeat(auto-fill,
           minmax(180px, 1fr)); gap: 10px; margin-bottom: 14px; }
.kv { background: var(--panel-2); border-radius: 6px; padding: 10px;
      display: flex; flex-direction: column; gap: 4px; font-size: 13px; }
details { margin: 8px 0; }
summary { cursor: pointer; color: var(--accent); font-size: 13px; }
pre { background: var(--bg); padding: 10px; border-radius: 6px;
      font-size: 12px; overflow-x: auto; max-height: 300px; }
.error { color: #ff4d6d; font-size: 12px; margin-top: 10px; }
</style>
