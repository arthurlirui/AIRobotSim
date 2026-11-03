<script setup>
import { ref, computed } from 'vue'
import SimView from './views/SimView.vue'
import CommView from './views/CommView.vue'
import DesignView from './views/DesignView.vue'

const tabs = [
  { id: 'sim', label: '实时仿真' },
  { id: 'comm', label: '通信实验' },
  { id: 'design', label: '构型设计' },
]
const active = ref('sim')
const current = computed(() =>
  ({ sim: SimView, comm: CommView, design: DesignView }[active.value]))
</script>

<template>
  <div class="console">
    <header class="console-header">
      <h1>AIRobotSim <span>仿真控制台</span></h1>
      <nav>
        <button v-for="t in tabs" :key="t.id"
                :class="{ active: active === t.id }"
                @click="active = t.id">{{ t.label }}</button>
      </nav>
    </header>
    <main class="console-body">
      <component :is="current" />
    </main>
  </div>
</template>

<style>
:root {
  --bg: #10141c;
  --panel: #1a2130;
  --panel-2: #222b3d;
  --border: #323d54;
  --text: #dce4f2;
  --muted: #8b98b0;
  --accent: #4d9fff;
  --ok: #3fd08a;
  --warn: #ffb84d;
}
* { box-sizing: border-box; }
html, body, #app { height: 100%; margin: 0; }
body {
  background: var(--bg);
  color: var(--text);
  font-family: 'Segoe UI', 'Microsoft YaHei', system-ui, sans-serif;
  font-size: 14px;
}
.console { display: flex; flex-direction: column; height: 100%; }
.console-header {
  display: flex; align-items: center; gap: 24px;
  padding: 10px 20px; background: var(--panel);
  border-bottom: 1px solid var(--border);
}
.console-header h1 { font-size: 18px; margin: 0; font-weight: 600; }
.console-header h1 span { color: var(--muted); font-weight: 400; font-size: 14px; }
.console-header nav { display: flex; gap: 4px; }
.console-header button {
  background: transparent; color: var(--muted); border: none;
  padding: 8px 16px; border-radius: 6px; cursor: pointer; font-size: 14px;
}
.console-header button:hover { color: var(--text); background: var(--panel-2); }
.console-header button.active { color: #fff; background: var(--accent); }
.console-body { flex: 1; min-height: 0; }

button.btn {
  background: var(--accent); color: #fff; border: none;
  padding: 7px 16px; border-radius: 6px; cursor: pointer; font-size: 13px;
}
button.btn:disabled { opacity: 0.45; cursor: not-allowed; }
button.btn.ghost {
  background: var(--panel-2); color: var(--text);
  border: 1px solid var(--border);
}
input, select {
  background: var(--panel-2); color: var(--text);
  border: 1px solid var(--border); border-radius: 5px;
  padding: 6px 8px; font-size: 13px;
}
.panel {
  background: var(--panel); border: 1px solid var(--border);
  border-radius: 8px; padding: 14px;
}
.panel h3 {
  margin: 0 0 10px; font-size: 13px; color: var(--muted);
  text-transform: uppercase; letter-spacing: 0.5px;
}
.muted { color: var(--muted); }
code, pre { font-family: Consolas, monospace; }
</style>
