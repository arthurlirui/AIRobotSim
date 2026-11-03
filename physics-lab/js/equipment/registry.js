// 设备注册表：id -> 定义。各学科模块在 import 时自注册。
const REGISTRY = new Map();
const CATS = []; // 分类顺序

export function registerCategory(key, name) { CATS.push({ key, name }); }

export function registerEquipment(cat, def) {
  def.cat = cat;
  REGISTRY.set(def.id, def);
}

export function getEquipmentDefs(cat) {
  return [...REGISTRY.values()].filter(d => d.cat === cat);
}
export function getEquipmentDef(id) { return REGISTRY.get(id); }
export function getCategories() { return CATS; }
export function getAllDefs() { return [...REGISTRY.values()]; }
