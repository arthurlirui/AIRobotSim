import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// Dev server proxies API/WS to the FastAPI backend (default :8000).
export default defineConfig({
  plugins: [vue()],
  server: {
    port: 5173,
    proxy: {
      '/api': { target: 'http://127.0.0.1:8000', changeOrigin: true },
      '/ws': { target: 'ws://127.0.0.1:8000', ws: true },
    },
  },
  build: { outDir: 'dist' },
})
