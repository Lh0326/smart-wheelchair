// 三雷达点云 + 相机视频流 + ASR 文本流
const COLORS = {
  n10p:   '#D4A574',  // 暖金 (N10P)
  ld14p:  '#6A9B9B',  // 青 (LD14P)
  camera: '#B571A5',  // 紫 (Gemini 335L)
};

// ========== Page Visibility：Tab 切走时暂停重渲染/相机拉流 ==========
// QTabWidget 中三个 Tab 同时活跃，Tab2 在后台仍跑 radarTick + fetchLoop，
// 长时间累积会拖垮 WebEngine GPU 进程。document.hidden 准确反映 Tab 可见性。
let pageVisible = !document.hidden;
document.addEventListener('visibilitychange', () => {
  pageVisible = !document.hidden;
  console.log('[companion] visibility:', pageVisible ? 'visible' : 'hidden');
});

// ========== 三色点云 Canvas ==========
const radarCanvas = document.getElementById('radar-canvas');
const radarCtx = radarCanvas.getContext('2d');
const W = 400, H = 400, CX = W/2, CY = H/2;
const MAX_RANGE = 8;
const PX_PER_M = (Math.min(W, H) / 2 - 30) / MAX_RANGE;

const scansBuffer = { n10p: null, ld14p: null, camera: null };
const SENSOR_POSES = {
  // base_link: x 前, y 左, 单位 m。yaw 暂按三个外设均朝轮椅正前方。
  n10p:   { x: -0.255, y:  0.000, yaw: 0 },
  ld14p:  { x:  0.320, y:  0.000, yaw: 0 },
  camera: { x: -0.255, y: -0.255, yaw: 0 },
};

function scanPointToBase(sensorKey, range, angle) {
  const pose = SENSOR_POSES[sensorKey] || { x: 0, y: 0, yaw: 0 };
  const localX = range * Math.cos(angle);
  const localY = range * Math.sin(angle);
  const c = Math.cos(pose.yaw);
  const s = Math.sin(pose.yaw);
  return {
    x: pose.x + localX * c - localY * s,
    y: pose.y + localX * s + localY * c,
  };
}

function drawBackground() {
  radarCtx.clearRect(0, 0, W, H);
  radarCtx.fillStyle = '#050608';
  radarCtx.fillRect(0, 0, W, H);

  // 同心圆
  radarCtx.strokeStyle = 'rgba(212,165,116,0.15)';
  radarCtx.lineWidth = 1;
  for (let r = 1; r <= MAX_RANGE; r++) {
    radarCtx.beginPath();
    radarCtx.arc(CX, CY, r * PX_PER_M, 0, Math.PI * 2);
    radarCtx.stroke();
  }

  // 距离标签(右侧)
  radarCtx.fillStyle = 'rgba(184,169,142,0.5)';
  radarCtx.font = '10px monospace';
  for (let r = 2; r <= MAX_RANGE; r += 2) {
    radarCtx.fillText(r + 'm', CX + r * PX_PER_M + 2, CY - 2);
  }

  // 十字辅助线
  radarCtx.strokeStyle = 'rgba(212,165,116,0.1)';
  radarCtx.beginPath();
  radarCtx.moveTo(0, CY); radarCtx.lineTo(W, CY);
  radarCtx.moveTo(CX, 0); radarCtx.lineTo(CX, H);
  radarCtx.stroke();

  // 方位标签(前=上, 后=下, 左=左, 右=右)
  radarCtx.fillStyle = '#D4A574';
  radarCtx.font = 'bold 13px sans-serif';
  radarCtx.textAlign = 'center';
  radarCtx.fillText('前', CX, 14);
  radarCtx.fillText('后', CX, H - 4);
  radarCtx.textAlign = 'left';
  radarCtx.fillText('右', W - 16, CY + 4);
  radarCtx.textAlign = 'right';
  radarCtx.fillText('左', 12, CY + 4);
  radarCtx.textAlign = 'left';  // 恢复默认

  // 中心轮椅图标(三角形朝上=前方)
  radarCtx.fillStyle = '#D4A574';
  radarCtx.beginPath();
  radarCtx.moveTo(CX, CY - 7);
  radarCtx.lineTo(CX - 5, CY + 5);
  radarCtx.lineTo(CX + 5, CY + 5);
  radarCtx.closePath();
  radarCtx.fill();
}

function drawScan(scan, color, sensorKey) {
  const ranges = scan?.ranges;
  const frame = scan;
  if (!ranges || !frame) return;
  radarCtx.fillStyle = color;
  const angleMin = frame.angle_min;
  const angleInc = frame.angle_increment;
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i];
    if (!isFinite(r) || r < frame.range_min || r > Math.min(frame.range_max, MAX_RANGE)) continue;
    const angle = angleMin + i * angleInc;
    const p = scanPointToBase(sensorKey, r, angle);
    const dist = Math.hypot(p.x, p.y);
    if (dist > MAX_RANGE) continue;
    // base_link 坐标: x 前 -> 画布上方, y 左 -> 画布左侧
    const canvasX = CX - p.y * PX_PER_M;
    const canvasY = CY - p.x * PX_PER_M;
    radarCtx.beginPath();
    radarCtx.arc(canvasX, canvasY, 1.5, 0, Math.PI * 2);
    radarCtx.fill();
  }
}

// ========== 方位最近障碍距离(前/后/左/右 ±30°)==========
function computeDirectionDistances() {
  const dirs = { front: Infinity, back: Infinity, left: Infinity, right: Infinity };
  const ranges_def = [
    { name: 'front', center: 0,         min: -Math.PI/6, max: Math.PI/6 },  // ±30°
    { name: 'back',  center: Math.PI,   min: Math.PI - Math.PI/6, max: Math.PI + Math.PI/6 },
    { name: 'left',  center: Math.PI/2, min: Math.PI/3, max: 2*Math.PI/3 },
    { name: 'right', center: -Math.PI/2,min: -2*Math.PI/3, max: -Math.PI/3 },
  ];

  for (const key of ['n10p', 'ld14p', 'camera']) {
    const scan = scansBuffer[key];
    if (!scan) continue;
    const angleMin = scan.angle_min;
    const angleInc = scan.angle_increment;
    const ranges = scan.ranges;
    const rangeMax = Math.min(scan.range_max, MAX_RANGE);

    for (const dir of ranges_def) {
      for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i];
        if (!isFinite(r) || r < scan.range_min || r > rangeMax) continue;
        const rawAngle = angleMin + i * angleInc;
        const p = scanPointToBase(key, r, rawAngle);
        const dist = Math.hypot(p.x, p.y);
        if (!isFinite(dist) || dist <= 0 || dist > MAX_RANGE) continue;
        const baseAngle = Math.atan2(p.y, p.x);
        let inSector = false;
        if (dir.name === 'back') {
          const a = baseAngle < 0 ? baseAngle + 2 * Math.PI : baseAngle;
          inSector = a >= dir.min && a <= dir.max;
        } else {
          inSector = baseAngle >= dir.min && baseAngle <= dir.max;
        }
        if (inSector && dist < dirs[dir.name]) {
          dirs[dir.name] = dist;
        }
      }
    }
  }
  return dirs;
}

function updateDirectionDisplay() {
  const dirs = computeDirectionDistances();
  const fmt = (v) => isFinite(v) ? v.toFixed(2) + 'm' : '--';
  document.getElementById('dir-front').textContent = fmt(dirs.front);
  document.getElementById('dir-back').textContent = fmt(dirs.back);
  document.getElementById('dir-left').textContent = fmt(dirs.left);
  document.getElementById('dir-right').textContent = fmt(dirs.right);
}

function radarTick() {
  if (!pageVisible) return;  // Tab 后台时跳过 Canvas 重绘（避免抢 GPU）
  drawBackground();
  drawScan(scansBuffer.n10p, COLORS.n10p, 'n10p');
  drawScan(scansBuffer.ld14p, COLORS.ld14p, 'ld14p');
  drawScan(scansBuffer.camera, COLORS.camera, 'camera');
  updateDirectionDisplay();
}
setInterval(radarTick, 100);

// ========== 相机视频流(web_video_server MJPEG,浏览器原生 <img> 解码)==========
// 注:不再用 rosbridge WebSocket + Canvas base64,改用 web_video_server HTTP MJPEG,
// 性能提升 10x(浏览器原生 img 解码 vs JS Canvas)
const cameraPlaceholder = document.getElementById('camera-placeholder');

function initCamera(ros) {
  // 用 camera_http_streamer(端口 8086) 拉 JPEG,目标 30 FPS。
  // 该节点订阅 /camera/color/image_raw,按源频率编码一次并缓存最新 JPEG,
  // HTTP GET / 立即返回缓存字节 → 前端 33ms fetch 即可达 30 FPS 显示。
  // 实际内容刷新率 = 相机源帧率(USB 2.1 下约 15Hz),CPU 成本与 fetch 频率解耦。
  const imgEl = document.getElementById('camera-img');
  const placeholder = document.getElementById('camera-placeholder');
  const fpsEl = document.getElementById('camera-fps');
  const baseHost = window.location.hostname;
  const snapshotUrl = 'http://' + baseHost + ':8086/?t=';
  const FETCH_INTERVAL_MS = 33;  // 30 FPS 上限

  // FPS 计算：滑动 1 秒窗口统计 onload 事件数
  const fpsWindow = [];
  let connected = false;
  let stopped = false;

  function recordFrame() {
    const now = performance.now();
    fpsWindow.push(now);
    while (fpsWindow.length > 0 && now - fpsWindow[0] > 1000) {
      fpsWindow.shift();
    }
    if (fpsEl) {
      fpsEl.textContent = fpsWindow.length + ' FPS';
    }
    if (!connected) {
      connected = true;
      placeholder.style.display = 'none';
      imgEl.style.display = 'block';
      addAsrLine('📷', '#8BA571', '相机已连接(snapshot)');
    }
  }

  async function fetchLoop() {
    while (!stopped) {
      // Tab 后台时降速到 2 FPS，避免抢 GPU/CPU
      const interval = pageVisible ? FETCH_INTERVAL_MS : 500;
      if (!pageVisible) {
        await new Promise(r => setTimeout(r, interval));
        continue;
      }
      try {
        // 加 cache-buster 避免 HTTP 缓存
        const url = snapshotUrl + Date.now();
        const resp = await fetch(url, { cache: 'no-store' });
        if (!resp.ok) {
          throw new Error('HTTP ' + resp.status);
        }
        const blob = await resp.blob();
        if (blob.size < 1000) {
          throw new Error('frame too small: ' + blob.size);
        }
        // createObjectURL 比 base64 快 5-10x
        const objUrl = URL.createObjectURL(blob);
        // 用 Image 对象预解码，onload 后再赋给 imgEl
        const tmpImg = new Image();
        tmpImg.onload = () => {
          if (imgEl.src && imgEl.src.startsWith('blob:')) {
            URL.revokeObjectURL(imgEl.src);  // 释放上一帧 blob
          }
          imgEl.src = objUrl;
          recordFrame();
        };
        tmpImg.onerror = () => {
          URL.revokeObjectURL(objUrl);
        };
        tmpImg.src = objUrl;
      } catch (e) {
        console.warn('[companion] snapshot fetch error:', e.message);
      }
      await new Promise(r => setTimeout(r, interval));
    }
  }

  // 启动 fetch 循环（async，不阻塞主线程）
  fetchLoop();
}

// ========== ASR 文本流 ==========
const asrLog = document.getElementById('asr-log');
const asrCount = document.getElementById('asr-count');
let asrTotal = 0;

function addAsrLine(icon, color, text) {
  const placeholder = asrLog.querySelector('.placeholder');
  if (placeholder) placeholder.remove();
  const time = new Date().toLocaleTimeString('zh-CN', {hour12: false});
  const line = document.createElement('div');
  line.className = 'asr-line';
  line.innerHTML = '<span class="ts">' + time + '</span> <span style="color:' + color + '">' + icon + '</span> ' + text;
  asrLog.appendChild(line);
  asrLog.scrollTop = asrLog.scrollHeight;
  asrTotal++;
  asrCount.textContent = asrTotal + ' 条';
  while (asrLog.children.length > 100) asrLog.removeChild(asrLog.firstChild);
}

// ========== 语音状态卡片(倒计时 + 三态视觉)==========
const VOICE_STATE_TEXT = {
  idle: '等待唤醒',
  listening: '聆听中',
  processing: '处理中',
};
const VOICE_STATE_HINT = {
  idle: '说"小智你好"唤醒我',
  listening: '请说出指令...',
  processing: '正在处理...',
};
let voiceState = { state: 'idle', remaining: 0 };
let voiceCountdownTimer = null;

function updateVoiceStatusUI(state, remaining) {
  const statusEl = document.getElementById('voice-status');
  const stateText = document.getElementById('voice-state-text');
  const hintEl = document.getElementById('voice-hint');
  const countdownEl = document.getElementById('voice-countdown');
  if (!statusEl) return;

  statusEl.dataset.state = state;
  stateText.textContent = VOICE_STATE_TEXT[state] || state;
  hintEl.textContent = VOICE_STATE_HINT[state] || '';

  if (state === 'listening') {
    countdownEl.textContent = remaining.toFixed(1) + 's';
  } else {
    countdownEl.textContent = '';
  }
}

// 倒计时定时器(10Hz,仅在 listening 时实际更新)
function startCountdownTimer() {
  if (voiceCountdownTimer) return;
  voiceCountdownTimer = setInterval(() => {
    if (voiceState.state === 'listening' && voiceState.remaining > 0) {
      voiceState.remaining = Math.max(0, voiceState.remaining - 0.1);
      updateVoiceStatusUI('listening', voiceState.remaining);
    }
  }, 100);
}

// 初始 UI + 启动后台倒计时
updateVoiceStatusUI('idle', 0);
startCountdownTimer();

// ========== YOLO bbox 叠加(Phase C 启用)==========
const detCanvas = document.getElementById('detection-canvas');
const detCtx = detCanvas.getContext('2d');
const CLASS_COLORS = {
  0: '#D97557',  // person 红
  1: '#D4A574',  // bicycle 暖金
  2: '#9B6A9B',  // car 紫
  9: '#8BA571',  // traffic_light 绿
};

function initDetections(ros) {
  new ROSLIB.Topic({
    ros, name: '/detections', messageType: 'rtk_msgs/msg/DetectionArray'
  }).subscribe((msg) => {
    detCtx.clearRect(0, 0, 640, 480);
    for (const d of msg.detections) {
      const x1 = d.bbox_px[0], y1 = d.bbox_px[1], x2 = d.bbox_px[2], y2 = d.bbox_px[3];
      const color = CLASS_COLORS[d.class_id] || '#FFFFFF';
      detCtx.strokeStyle = color;
      detCtx.lineWidth = 2;
      detCtx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      detCtx.fillStyle = color;
      detCtx.font = 'bold 12px monospace';
      detCtx.fillText(d.class_name + ' ' + (d.confidence*100).toFixed(0) + '% ' + d.distance_m.toFixed(1) + 'm',
                      x1, Math.max(15, y1 - 4));
    }
  });
}

// ========== roslibjs 连接 + 订阅 ==========
const ROSBRIDGE_URL = 'ws://' + window.location.hostname + ':9091';
const ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
let _rosTopics = [];  // 跟踪订阅的 topic，重连前 unsubscribe 防泄漏

function _subscribeTopic(name, messageType, cb) {
  const t = new ROSLIB.Topic({ros, name, messageType});
  t.subscribe(cb);
  _rosTopics.push(t);
  return t;
}

function _unsubscribeAll() {
  for (const t of _rosTopics) {
    try { t.unsubscribe(); } catch (e) {}
  }
  _rosTopics = [];
}

function _onRosConnected() {
  console.log('[companion] rosbridge connected');
  _subscribeTopic('/scan', 'sensor_msgs/msg/LaserScan', (m) => { scansBuffer.n10p = m; });
  _subscribeTopic('/scan_ld14p', 'sensor_msgs/msg/LaserScan', (m) => { scansBuffer.ld14p = m; });
  _subscribeTopic('/scan_gemini', 'sensor_msgs/msg/LaserScan', (m) => { scansBuffer.camera = m; });
  try { initDetections(ros); } catch (e) { console.log('[companion] detections not ready'); }

  // === 硬件监控条 ===
  _subscribeTopic('/hw_status', 'std_msgs/msg/String', (msg) => {
    try {
      const hw = JSON.parse(msg.data);
      document.getElementById('hw-cpu-pct').textContent = hw.cpu_percent.toFixed(0);
      document.getElementById('hw-cpu').style.width = hw.cpu_percent + '%';
      document.getElementById('hw-gpu-pct').textContent = hw.gpu_percent.toFixed(0);
      document.getElementById('hw-gpu').style.width = hw.gpu_percent + '%';
      document.getElementById('hw-npu-pct').textContent = hw.npu_percent.toFixed(0);
      document.getElementById('hw-npu').style.width = hw.npu_percent + '%';
      document.getElementById('hw-mem-pct').textContent = hw.mem_percent.toFixed(0);
      document.getElementById('hw-mem-gb').textContent = hw.mem_used_gb.toFixed(1);
      document.getElementById('hw-mem').style.width = hw.mem_percent + '%';
      document.getElementById('hw-load').textContent = hw.load_1m.toFixed(2);
    } catch (e) {
      console.warn('[companion] hw_status 解析失败:', e);
    }
  });
  // /voice_state:JSON,更新状态卡片 + 倒计时
  _subscribeTopic('/voice_state', 'std_msgs/msg/String', (msg) => {
    try {
      const st = JSON.parse(msg.data);
      const newState = st.state || 'idle';
      const newRemaining = st.remaining || 0;

      // 状态变化时记日志
      if (newState !== voiceState.state) {
        if (newState === 'listening') {
          addAsrLine('🔔', '#D4A574', '唤醒成功,开始聆听');
        } else if (newState === 'processing') {
          addAsrLine('⚙️', '#8BA571', '处理指令中...');
        } else if (voiceState.state === 'processing' && newState === 'idle') {
          addAsrLine('✅', '#8BA571', '指令处理完成');
        }
      }

      voiceState = { state: newState, remaining: newRemaining };
      updateVoiceStatusUI(newState, newRemaining);
    } catch (e) {
      // 非 JSON,fallback
      console.warn('[companion] voice_state 非 JSON:', msg.data);
    }
  });

  // /voice_command:JSON 格式 {action, text},友好显示
  const ACTION_TEXT = {
    wakeup: '唤醒',
    query: '询问',
    stop: '停止',
    emergency_stop: '紧急停止',
    speed_up: '加速',
    speed_down: '减速',
    unknown: '未识别',
  };
  _subscribeTopic('/voice_command', 'std_msgs/msg/String', (msg) => {
    try {
      const cmd = JSON.parse(msg.data);
      const action = cmd.action || 'unknown';
      const text = cmd.text || '';
      const actionCN = ACTION_TEXT[action] || action;
      if (action === 'wakeup') {
        addAsrLine('🔔', '#D4A574', `唤醒成功${text ? ':"' + text + '"' : ''}`);
      } else if (text) {
        addAsrLine('🎤', '#D4A574', `你说:"${text}"`);
        addAsrLine('⚙️', '#8BA571', `→ 指令:${actionCN}`);
      } else {
        addAsrLine('⚙️', '#8BA571', `指令:${actionCN}`);
      }
    } catch (e) {
      // 非 JSON,fallback
      addAsrLine('🎤', '#D4A574', msg.data);
    }
  });
  addAsrLine('🟢', '#8BA571', '系统已连接');
}

ros.on('connection', _onRosConnected);
ros.on('error', (e) => console.error('[companion] rosbridge error:', e));

// 关键修复：close 改为 reconnect，不再 location.reload()
// reload 会让 Chromium 反复重启渲染进程，长时间运行后触发 GPU 内存崩溃 → 黑屏
ros.on('close', () => {
  console.log('[companion] rosbridge closed, unsubscribe + retry connect in 3s');
  _unsubscribeAll();
  setTimeout(() => {
    try { ros.connect(ROSBRIDGE_URL); }
    catch (e) { console.error('[companion] reconnect failed:', e); }
  }, 3000);
});

// ========== 相机 fetchLoop 独立启动（不依赖 rosbridge 连接）==========
// 原来放在 ros.on('connection') 里 → 重连会启动第二次 fetchLoop，造成内存泄漏
try { initCamera(ros); } catch (e) { console.log('[companion] camera init deferred'); }
