// UI 总装：设备仓库 / 属性面板 / 数据面板 / 指导面板 / 工具栏
import { getEquipmentDefs, getCategories, getEquipmentDef } from '../equipment/registry.js';
import { EventBus, EV } from '../core/EventBus.js';
import { GUIDES } from '../experiments/guides.js';

export class UI {
  constructor(app) {
    this.app = app;
    this.selected = null;
    this.currentCat = 'mechanics';
    this.dataRows = [];      // {source: name, row: {col: val}}
    this.currentGuide = null;
    this.doneSteps = new Set();
    this._bind();
    this._buildEquipList();
    this._buildGuideList();
    this.selectGuide(GUIDES[0].id, false);
  }

  _bind() {
    // 设备分类 tab
    document.querySelectorAll('.etab').forEach(b => b.addEventListener('click', () => {
      document.querySelectorAll('.etab').forEach(x => x.classList.remove('active'));
      b.classList.add('active');
      this.currentCat = b.dataset.cat;
      this._buildEquipList();
    }));
    // 右侧 tab
    document.querySelectorAll('.rtab').forEach(b => b.addEventListener('click', () => {
      document.querySelectorAll('.rtab').forEach(x => x.classList.remove('active'));
      b.classList.add('active');
      document.querySelectorAll('.rtab-body').forEach(x => x.style.display = 'none');
      document.getElementById('tab-' + b.dataset.tab).style.display = 'flex';
      if (b.dataset.tab === 'data') this._refreshDataUI();
      if (b.dataset.tab === 'props') this._refreshPropsLive();
    }));
    // 数据面板
    document.getElementById('btn-record').addEventListener('click', () => this._recordRow());
    document.getElementById('btn-clear-data').addEventListener('click', () => {
      this.dataRows = []; this._refreshDataUI();
    });
    document.getElementById('btn-export-csv').addEventListener('click', () => this._exportCSV());
    document.getElementById('plot-x').addEventListener('change', () => this._drawPlot());
    document.getElementById('plot-y').addEventListener('change', () => this._drawPlot());
    document.getElementById('plot-fit').addEventListener('change', () => this._drawPlot());
    document.getElementById('btn-help').addEventListener('click', () => {
      document.getElementById('help-modal').style.display = 'flex';
    });
    // 指导步骤完成状态持久化在内存
    EventBus.on(EV.TOAST, ({ msg, kind }) => this.toast(msg, kind));
  }

  toast(msg, kind = '') {
    const el = document.createElement('div');
    el.className = 'toast ' + kind;
    el.textContent = msg;
    document.getElementById('toast-container').appendChild(el);
    setTimeout(() => { el.style.opacity = '0'; el.style.transition = 'opacity .4s'; }, 2200);
    setTimeout(() => el.remove(), 2700);
  }

  /* ---------- 设备仓库 ---------- */
  _buildEquipList() {
    const list = document.getElementById('equip-list');
    list.innerHTML = '';
    for (const def of getEquipmentDefs(this.currentCat)) {
      const item = document.createElement('div');
      item.className = 'equip-item';
      item.innerHTML = `<div class="icon">${def.icon}</div><div><div class="name">${def.name}</div><div class="desc">${def.desc}</div></div>`;
      item.addEventListener('click', () => this.app.spawnEquipment(def.id));
      list.appendChild(item);
    }
  }

  /* ---------- 属性面板 ---------- */
  showProps(eq) {
    this.selected = eq;
    document.getElementById('props-empty').style.display = 'none';
    const box = document.getElementById('props-content');
    box.style.display = '';
    box.innerHTML = '';

    const title = document.createElement('h3');
    title.innerHTML = `${eq.def.icon} ${eq.def.name}`;
    const zoneName = { mechanics: '力学区', optics: '光学区', electricity: '电学区', waves: '波学区', thermal: '热学区', measurement: '测量' }[eq.zoneKey] || '';
    const zoneEl = document.createElement('div');
    zoneEl.className = 'props-zone';
    zoneEl.textContent = `位于${zoneName} · 拖拽本体可在台面内移动`;
    box.append(title, zoneEl);

    // 参数
    for (const p of eq.paramSchema()) {
      const row = document.createElement('div');
      row.className = 'prop-row';
      if (p.type === 'select') {
        const lab = document.createElement('label');
        lab.textContent = p.label;
        const sel = document.createElement('select');
        sel.className = 'tb-select';
        sel.style.width = '100%';
        for (const [v, name] of p.options) {
          const o = document.createElement('option');
          o.value = v; o.textContent = name;
          if (eq.params[p.key] === v) o.selected = true;
          sel.appendChild(o);
        }
        sel.addEventListener('change', () => { eq.params[p.key] = sel.value; eq.onParamsChanged(); this._refreshReadouts(); });
        row.append(lab, sel);
      } else if (p.type === 'color') {
        const lab = document.createElement('label');
        lab.textContent = p.label;
        const c = document.createElement('input');
        c.type = 'color'; c.value = eq.params[p.key];
        c.style.width = '100%'; c.style.height = '26px';
        c.addEventListener('input', () => { eq.params[p.key] = c.value; eq.onParamsChanged(); });
        row.append(lab, c);
      } else {
        const lab = document.createElement('label');
        const valSpan = document.createElement('span');
        valSpan.className = 'val';
        lab.append(document.createTextNode(p.label), valSpan);
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.min = p.min; slider.max = p.max; slider.step = p.step;
        slider.value = eq.params[p.key];
        valSpan.textContent = (+eq.params[p.key]).toFixed(p.step >= 1 ? 0 : 2);
        slider.addEventListener('input', () => {
          eq.params[p.key] = +slider.value;
          valSpan.textContent = (+slider.value).toFixed(p.step >= 1 ? 0 : 2);
          eq.onParamsChanged();
          this._refreshReadouts();
        });
        row.append(lab, slider);
      }
      box.appendChild(row);
    }

    // 动作按钮
    const actions = eq.actions?.() || [];
    if (actions.length) {
      const sep = document.createElement('div');
      sep.className = 'props-sep';
      const row = document.createElement('div');
      row.className = 'props-actions';
      for (const a of actions) {
        const btn = document.createElement('button');
        btn.className = 'tb-btn small';
        btn.textContent = a.label;
        btn.addEventListener('click', () => { eq.doAction?.(a.key); this._refreshReadouts(); });
        row.appendChild(btn);
      }
      box.append(sep, row);
    }

    // 读数卡
    this._buildReadouts();
    // 删除按钮
    const sep2 = document.createElement('div');
    sep2.className = 'props-sep';
    const del = document.createElement('button');
    del.className = 'tb-btn small danger';
    del.textContent = '🗑 删除本设备';
    del.addEventListener('click', () => this.app.removeEquipment(eq));
    box.append(sep2, del);
  }

  clearProps() {
    this.selected = null;
    document.getElementById('props-empty').style.display = '';
    document.getElementById('props-content').style.display = 'none';
  }

  _buildReadouts() {
    const eq = this.selected;
    if (!eq) return;
    let box = document.getElementById('readouts-box');
    if (!box) {
      box = document.createElement('div');
      box.id = 'readouts-box';
      document.getElementById('props-content').appendChild(box);
    }
    box.innerHTML = '';
    const ro = eq.getReadouts();
    const grid = document.createElement('div');
    grid.className = 'readouts';
    for (const [k, v] of Object.entries(ro)) {
      const cell = document.createElement('div');
      cell.className = 'readout';
      cell.innerHTML = `<div class="k">${k}</div><div class="v">${v}</div>`;
      grid.appendChild(cell);
    }
    box.appendChild(grid);
  }

  _refreshReadouts() { if (this.selected) this._buildReadouts(); }
  _refreshPropsLive() { this._refreshReadouts(); }

  /* ---------- 数据面板 ---------- */
  _recordRow() {
    const eq = this.selected;
    if (!eq) { this.toast('请先选中一个设备作为数据来源', 'warn'); return; }
    const ro = eq.getReadouts();
    this.dataRows.push({ source: eq.def.name, row: { ...ro } });
    EventBus.emit(EV.DATA_RECORDED, {});
    this._refreshDataUI();
    this.toast('已记录 1 行数据');
  }

  _refreshDataUI() {
    const sel = document.getElementById('data-source');
    const sources = [...new Set(this.dataRows.map(r => r.source))];
    sel.innerHTML = sources.map(s => `<option>${s}</option>`).join('') || '<option>（暂无数据）</option>';
    this._renderTable();
    this._fillPlotCols();
    this._drawPlot();
  }

  _renderTable() {
    const table = document.getElementById('data-table');
    const source = document.getElementById('data-source').value;
    const rows = this.dataRows.filter(r => !source || r.source === source);
    if (rows.length === 0) { table.innerHTML = '<tr><td style="padding:12px;color:#93a4bd">暂无数据，选中设备后点「记录一行」</td></tr>'; return; }
    const cols = [...new Set(rows.flatMap(r => Object.keys(r.row)))];
    let html = '<tr><th>#</th>' + cols.map(c => `<th>${c}</th>`).join('') + '</tr>';
    rows.forEach((r, i) => {
      html += `<tr><td>${i + 1}</td>` + cols.map(c => `<td>${r.row[c] ?? '—'}</td>`).join('') + '</tr>';
    });
    table.innerHTML = html;
  }

  _fillPlotCols() {
    const source = document.getElementById('data-source').value;
    const rows = this.dataRows.filter(r => !source || r.source === source);
    const cols = [...new Set(rows.flatMap(r => Object.keys(r.row)))].filter(c => typeof rows[0]?.row[c] === 'number');
    for (const id of ['plot-x', 'plot-y']) {
      const sel = document.getElementById(id);
      const cur = sel.value;
      sel.innerHTML = cols.map(c => `<option>${c}</option>`).join('');
      if (cols.includes(cur)) sel.value = cur;
    }
  }

  _drawPlot() {
    const canvas = document.getElementById('plot-canvas');
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);
    const source = document.getElementById('data-source').value;
    const rows = this.dataRows.filter(r => !source || r.source === source);
    const xk = document.getElementById('plot-x').value;
    const yk = document.getElementById('plot-y').value;
    const pts = rows.map(r => [r.row[xk], r.row[yk]]).filter(p => typeof p[0] === 'number' && typeof p[1] === 'number');
    const fitEl = document.getElementById('fit-result');
    if (pts.length < 1) { fitEl.textContent = ''; return; }
    const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]);
    const xmin = Math.min(...xs), xmax = Math.max(...xs);
    const ymin = Math.min(...ys), ymax = Math.max(...ys);
    const pad = 34;
    const sx = v => pad + (v - xmin) / (xmax - xmin || 1) * (W - pad - 10);
    const sy = v => H - pad - (v - ymin) / (ymax - ymin || 1) * (H - pad - 12);
    // 坐标轴
    ctx.strokeStyle = '#5a6a80'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(pad, 10); ctx.lineTo(pad, H - pad); ctx.lineTo(W - 8, H - pad); ctx.stroke();
    ctx.fillStyle = '#93a4bd'; ctx.font = '10px Consolas';
    ctx.fillText(xmin.toFixed(2), pad - 4, H - pad + 12);
    ctx.fillText(xmax.toFixed(2), W - 30, H - pad + 12);
    ctx.fillText(ymin.toFixed(2), 4, H - pad);
    ctx.fillText(ymax.toFixed(2), 4, 14);
    // 数据点
    ctx.fillStyle = '#4cc3ff';
    for (const [px, py] of pts) { ctx.beginPath(); ctx.arc(sx(px), sy(py), 3, 0, 7); ctx.fill(); }
    // 线性拟合
    if (document.getElementById('plot-fit').checked && pts.length >= 2) {
      const n = pts.length;
      const mx = xs.reduce((a, b) => a + b) / n, my = ys.reduce((a, b) => a + b) / n;
      let num = 0, den = 0;
      for (let i = 0; i < n; i++) { num += (xs[i] - mx) * (ys[i] - my); den += (xs[i] - mx) ** 2; }
      if (den > 0) {
        const kFit = num / den, bFit = my - kFit * mx;
        ctx.strokeStyle = '#ffb454'; ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(sx(xmin), sy(kFit * xmin + bFit));
        ctx.lineTo(sx(xmax), sy(kFit * xmax + bFit));
        ctx.stroke();
        fitEl.textContent = `拟合: y = ${kFit.toFixed(4)}·x + ${bFit.toFixed(4)}`;
      }
    }
  }

  _exportCSV() {
    if (this.dataRows.length === 0) { this.toast('暂无数据可导出', 'warn'); return; }
    const source = document.getElementById('data-source').value;
    const rows = this.dataRows.filter(r => !source || r.source === source);
    const cols = [...new Set(rows.flatMap(r => Object.keys(r.row)))];
    let csv = '\uFEFF' + ['来源', ...cols].join(',') + '\n';
    for (const r of rows) csv += [r.source, ...cols.map(c => r.row[c] ?? '')].join(',') + '\n';
    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `物理实验数据_${new Date().toISOString().slice(0, 10)}.csv`;
    a.click();
    this.toast('CSV 已导出');
  }

  /* ---------- 指导面板 ---------- */
  _buildGuideList() {
    const list = document.getElementById('guide-list');
    list.innerHTML = '';
    for (const g of GUIDES) {
      const chip = document.createElement('button');
      chip.className = 'guide-chip';
      chip.textContent = g.name;
      chip.dataset.gid = g.id;
      chip.addEventListener('click', () => this.selectGuide(g.id, true));
      list.appendChild(chip);
    }
  }

  selectGuide(id, doSetup) {
    this.currentGuide = GUIDES.find(g => g.id === id);
    this.doneSteps.clear();
    document.querySelectorAll('.guide-chip').forEach(c => c.classList.toggle('active', c.dataset.gid === id));
    const g = this.currentGuide;
    const box = document.getElementById('guide-content');
    box.innerHTML = `
      <h4>🎯 实验目的</h4><div>${g.goal}</div>
      <h4>📖 原理</h4><div>${g.theory}</div>
      <button class="tb-btn primary setup-btn" id="btn-setup">📦 一键布置器材</button>
      <h4>📝 步骤</h4>
      <ol>${g.steps.map((s, i) => `<li class="step" data-step="${i}">${s}</li>`).join('')}</ol>
      <div class="tip">💡 ${g.tip}</div>
      <h4>🤔 思考题</h4>
      <ol>${g.think.map(t => `<li>${t}</li>`).join('')}</ol>
    `;
    box.querySelectorAll('.step').forEach(li => li.addEventListener('click', () => {
      li.classList.toggle('done');
    }));
    document.getElementById('btn-setup').addEventListener('click', () => {
      this.app.setupGuide(g);
      this.toast('器材已布置：' + g.setup.map(s => getEquipmentDef(s.type)?.name).join('、'));
    });
    if (doSetup) this.app.cameraToZone(g.zone);
  }

  setPlaying(on) {
    const btn = document.getElementById('btn-play');
    btn.textContent = on ? '⏸ 暂停' : '▶ 运行';
  }
  setDark(on) {
    document.getElementById('btn-dark').classList.toggle('active', on);
  }
  setStatus(msg) { document.getElementById('status-info').textContent = msg; }
  setFPS(fps) { document.getElementById('fps').textContent = fps + ' FPS'; }
  setClock(t) { document.getElementById('sim-clock').textContent = `t = ${t.toFixed(2)} s`; }
}
