// Thin fetch wrappers over the FastAPI backend (dev: proxied by Vite).

async function j(r) {
  if (!r.ok) throw new Error(`${r.status} ${r.statusText}`)
  return r.json()
}

export async function fetchCatalog() {
  return j(await fetch('/api/catalog'))
}

export async function runScenario(name, config) {
  return j(await fetch('/api/scenarios/run', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name, config }),
  }))
}

export async function fetchCommScenarios() {
  return j(await fetch('/api/comm/scenarios'))
}

export async function runCommExperiment(experiment) {
  return j(await fetch('/api/comm/simulate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ experiment }),
  }))
}

export async function fetchDesignPresets() {
  return j(await fetch('/api/design/presets'))
}

export async function runDesignSimulate(req) {
  return j(await fetch('/api/design/simulate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(req),
  }))
}
