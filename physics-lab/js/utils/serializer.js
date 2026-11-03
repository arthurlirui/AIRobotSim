// 序列化：保存 / 加载场景（localStorage + 文件）
export function serializeScene(equipments, extra = {}) {
  return JSON.stringify({
    version: 1,
    savedAt: new Date().toISOString(),
    extra,
    equipments: equipments.map(eq => {
      const d = eq.serialize();
      if (eq.isCircuitHost && d.circuitKeep) {
        d.circuit = {
          components: eq.components.map(c => ({ kind: c.kind, pos: c.group.position.toArray(), on: c.isOn(), R: c.R, v: c.v })),
          wires: eq.wires.map(w => ({
            a: [eq.components.indexOf(w.a.comp), w.a.comp.terminals.indexOf(w.a)],
            b: [eq.components.indexOf(w.b.comp), w.b.comp.terminals.indexOf(w.b)],
          })),
        };
      }
      return d;
    }),
  }, null, 2);
}

export function deserializeScene(json, equipments, spawn) {
  const data = JSON.parse(json);
  // 清空现有
  for (const eq of [...equipments]) eq.dispose?.();
  const result = [];
  for (const d of data.equipments) {
    const eq = spawn(d.type, d.zone);
    if (!eq) continue;
    if (d.pos) eq.group.position.fromArray(d.pos);
    if (d.params) { Object.assign(eq.params, d.params); eq.onParamsChanged(); }
    if (eq.isCircuitHost && d.circuit) {
      for (const cd of d.circuit.components) {
        const comp = eq.addComponent(cd.kind);
        if (cd.pos) comp.group.position.fromArray(cd.pos);
        if (cd.on === false && comp.toggle) comp.toggle();
        if (cd.R !== undefined && comp.R !== undefined) { comp.R = cd.R; if (comp.setFraction) comp.setFraction(1 - cd.R / 20); }
        if (cd.v !== undefined && comp.v !== undefined) comp.v = cd.v;
      }
      for (const wd of d.circuit.wires) {
        const ca = eq.components[wd.a[0]], cb = eq.components[wd.b[0]];
        if (ca && cb) eq.connect(ca.terminals[wd.a[1]], cb.terminals[wd.b[1]]);
      }
    }
    result.push(eq);
  }
  return { equipments: result, extra: data.extra };
}
