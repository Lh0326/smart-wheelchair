// Service Worker - 缓存前端文件实现离线启动
// 注意：rosbridge WebSocket 和 ArcGIS 瓦片不缓存（动态数据/在线资源）

const CACHE_NAME = 'wheelchair-monitor-v1';
const STATIC_ASSETS = [
  './',
  './index.html',
  './app.js',
  './styles.css',
  './manifest.json',
  './poi.geojson',
  './vendor/leaflet.css',
  './vendor/leaflet.js',
  './vendor/roslibjs.js',
  './icon-192.png',
  './icon-512.png',
];

// 安装：预缓存静态资源
self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => {
      // 用 addAll 但忽略单个文件失败（如图标不存在）
      return Promise.allSettled(
        STATIC_ASSETS.map((url) => cache.add(url))
      );
    }).then(() => {
      return self.skipWaiting();
    })
  );
});

// 激活：清理旧缓存
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) => {
      return Promise.all(
        keys.filter((k) => k !== CACHE_NAME).map((k) => caches.delete(k))
      );
    }).then(() => {
      return self.clients.claim();
    })
  );
});

// 请求拦截：缓存优先，网络兜底
self.addEventListener('fetch', (event) => {
  const req = event.request;

  // 跳过非 GET 请求（如 WebSocket 升级、service call）
  if (req.method !== 'GET') return;

  const url = new URL(req.url);

  // 同源请求：缓存优先
  if (url.origin === self.location.origin) {
    event.respondWith(
      caches.match(req).then((cached) => {
        return cached || fetch(req).then((resp) => {
          // 缓存新资源
          if (resp.ok) {
            const clone = resp.clone();
            caches.open(CACHE_NAME).then((c) => c.put(req, clone));
          }
          return resp;
        }).catch(() => cached);
      })
    );
    return;
  }

  // 跨域请求（ArcGIS 瓦片、公共 OSRM）：直接走网络
  // 离线时这些资源不可用是预期行为（位置/航向仍可显示）
});
