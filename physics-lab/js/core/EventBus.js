// 全局事件总线：设备增删、选择变化、仿真事件、数据记录
class EventBusImpl {
  constructor() { this.map = new Map(); }
  on(type, fn) {
    if (!this.map.has(type)) this.map.set(type, new Set());
    this.map.get(type).add(fn);
    return () => this.off(type, fn);
  }
  off(type, fn) { this.map.get(type)?.delete(fn); }
  emit(type, payload) { this.map.get(type)?.forEach(fn => { try { fn(payload); } catch (e) { console.error(`[EventBus] ${type}`, e); } }); }
}
export const EventBus = new EventBusImpl();
export const EV = {
  EQUIPMENT_ADDED: 'eq:added',
  EQUIPMENT_REMOVED: 'eq:removed',
  SELECTION_CHANGED: 'sel:changed',
  SIM_TICK: 'sim:tick',
  DATA_RECORDED: 'data:recorded',
  DATA_CLEARED: 'data:cleared',
  TOAST: 'ui:toast',
};
