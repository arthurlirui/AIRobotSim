// Live simulation store: owns the /ws/sim WebSocket connection and the
// latest world state (terrain, robots, lidar, comm links, algorithm metrics).
import { defineStore } from 'pinia'

export const useSimStore = defineStore('sim', {
  state: () => ({
    ws: null,
    status: 'idle',          // idle | connecting | live | paused | error
    error: '',
    world: null,             // {robots, algorithms, dt, has_comm, terrain}
    frame: null,             // latest frame from the server
    step: 0,
    simTime: 0,
  }),
  actions: {
    connect(config) {
      this.disconnect()
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      this.status = 'connecting'
      this.error = ''
      const ws = new WebSocket(`${proto}://${location.host}/ws/sim`)
      this.ws = ws
      ws.onopen = () => ws.send(JSON.stringify({ type: 'create', config }))
      ws.onmessage = (ev) => this._onMessage(JSON.parse(ev.data))
      ws.onerror = () => {
        this.status = 'error'
        this.error = 'WebSocket 连接失败（后端未启动？）'
      }
      ws.onclose = () => {
        if (this.status !== 'error') this.status = 'idle'
      }
    },
    disconnect() {
      if (this.ws) { this.ws.close(); this.ws = null }
      this.status = 'idle'
    },
    stepOnce(n = 1) {
      if (this.ws && this.ws.readyState === 1)
        this.ws.send(JSON.stringify({ type: 'step', n }))
    },
    reset() {
      if (this.ws && this.ws.readyState === 1)
        this.ws.send(JSON.stringify({ type: 'reset' }))
    },
    _onMessage(msg) {
      switch (msg.type) {
        case 'created':
          this.world = msg
          this.frame = null
          this.step = 0
          this.simTime = 0
          this.status = 'live'
          break
        case 'frame':
          this.frame = msg
          this.step = msg.step
          this.simTime = msg.t
          break
        case 'paused':
          this.status = 'paused'
          break
        case 'error':
          this.error = msg.message
          break
      }
    },
  },
})
