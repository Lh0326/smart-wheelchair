// 智慧轮椅室外导航 - 前端逻辑（M1.5 增强版）
// 任务 9-10：地图基础 + 终点选择
// M1.5 增强：
//   - 高德卫星图作为底图（GCJ-02 坐标系，本地 mbtiles_server）
//   - 自身位置显示（订阅 /fix，WGS-84 → GCJ-02 转换后显示）
//   - 方向箭头（订阅 /heading_imu）
//   - 路径规划（订阅 /global_plan，WGS-84 → GCJ-02 转换后显示）

// ============ WGS-84 ↔ GCJ-02 坐标转换（标准公开算法）============
const _GCJ_PI = 3.1415926535897932384626;
const _GCJ_A = 6378245.0;
const _GCJ_EE = 0.00669342162296594323;

function _transformLat(x, y) {
  let ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x));
  ret += (20.0 * Math.sin(6.0 * x * _GCJ_PI) + 20.0 * Math.sin(2.0 * x * _GCJ_PI)) * 2.0 / 3.0;
  ret += (20.0 * Math.sin(y * _GCJ_PI) + 40.0 * Math.sin(y / 3.0 * _GCJ_PI)) * 2.0 / 3.0;
  ret += (160.0 * Math.sin(y / 12.0 * _GCJ_PI) + 320 * Math.sin(y * _GCJ_PI / 30.0)) * 2.0 / 3.0;
  return ret;
}

function _transformLng(x, y) {
  let ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x));
  ret += (20.0 * Math.sin(6.0 * x * _GCJ_PI) + 20.0 * Math.sin(2.0 * x * _GCJ_PI)) * 2.0 / 3.0;
  ret += (20.0 * Math.sin(x * _GCJ_PI) + 40.0 * Math.sin(x / 3.0 * _GCJ_PI)) * 2.0 / 3.0;
  ret += (150.0 * Math.sin(x / 12.0 * _GCJ_PI) + 300.0 * Math.sin(x / 30.0 * _GCJ_PI)) * 2.0 / 3.0;
  return ret;
}

function _outOfChina(lng, lat) {
  return !(lng > 73.66 && lng < 135.05 && lat > 3.86 && lat < 53.55);
}

// WGS-84 → GCJ-02：返回 [lat, lng]
function wgsToGcj(wgsLat, wgsLng) {
  if (_outOfChina(wgsLng, wgsLat)) return [wgsLat, wgsLng];
  let dLat = _transformLat(wgsLng - 105.0, wgsLat - 35.0);
  let dLng = _transformLng(wgsLng - 105.0, wgsLat - 35.0);
  const radLat = wgsLat / 180.0 * _GCJ_PI;
  let magic = Math.sin(radLat);
  magic = 1 - _GCJ_EE * magic * magic;
  const sqrtMagic = Math.sqrt(magic);
  dLat = (dLat * 180.0) / ((_GCJ_A * (1 - _GCJ_EE)) / (magic * sqrtMagic) * _GCJ_PI);
  dLng = (dLng * 180.0) / (_GCJ_A / sqrtMagic * Math.cos(radLat) * _GCJ_PI);
  return [wgsLat + dLat, wgsLng + dLng];
}

// GCJ-02 → WGS-84：返回 [lat, lng]（近似逆变换）
function gcjToWgs(gcjLat, gcjLng) {
  if (_outOfChina(gcjLng, gcjLat)) return [gcjLat, gcjLng];
  const [gLat, gLng] = wgsToGcj(gcjLat, gcjLng);
  return [gcjLat * 2 - gLat, gcjLng * 2 - gLng];
}

// 瓦片源：本地 mbtiles_server（全离线，不需要网络）
// 数据：高德卫星图 z=15-18（1903 张，覆盖昆明理工 5km 区域）
// 注意：高德瓦片用 GCJ-02 坐标系，GPS（WGS-84）位置会有约 50-500m 偏移
const TILES_URL = 'http://localhost:8080/tiles/{z}/{x}/{y}.png';
const POI_URL = 'poi.geojson';
// 动态获取当前页面 host，支持手机/远程访问（不写死 localhost）
const ROSBRIDGE_URL = `ws://${window.location.hostname}:9091`;

// 初始视图：昆工呈贡校区中心
const INITIAL_VIEW = [24.8551, 102.8553];
const INITIAL_ZOOM = 17;

let map = null;
let tilesLayer = null;
let goalMarker = null;
let selfMarker = null;
let selfCircle = null;
let routeLayer = null;
let ros = null;
let goalTopic = null;
let fixListener = null;
let headingListener = null;
let poiLayer = null;
let currentHeading = 0;

let lastFix = null;
let waypointMarkers = [];      // 所有规划点 markers 数组
let currentWaypointMarker = null; // 当前目标 marker（脉冲）
let allWaypointsListener = null;
let nextWaypointListener = null;

// ========== GPS 三态状态管理 ==========
// cold: 冷启动（5 秒收不到 /fix）  weak: 信号弱（收到 /fix 但 status=-1）  fix: 成功定位
let lastFixMsgTime = 0;
let gpsState = 'cold';
const GPS_COLD_TIMEOUT_MS = 5000;

function updateGpsState(state, detail) {
  if (gpsState === state) {
    // 同状态下只更新副文字（如卫星数变化）
    const detailEl = document.querySelector('#gps-status-card .gps-detail');
    if (detailEl && detail) detailEl.textContent = detail;
    return;
  }
  gpsState = state;
  const card = document.getElementById('gps-status-card');
  if (!card) return;
  card.className = 'gps-card gps-state-' + state;
  const labelEl = card.querySelector('.gps-state-label');
  const detailEl = card.querySelector('.gps-detail');
  const labels = { cold: '冷启动', weak: '信号弱', fix: '定位成功' };
  if (labelEl) labelEl.textContent = labels[state] || state;
  if (detailEl) detailEl.textContent = detail || '';
}

function startGpsTimeoutWatchdog() {
  setInterval(() => {
    if (Date.now() - lastFixMsgTime > GPS_COLD_TIMEOUT_MS && gpsState !== 'cold') {
      updateGpsState('cold', '等待 GNSS 引擎启动');
    }
  }, 1000);
}

function log(message) {
  const list = document.getElementById('log-list');
  const li = document.createElement('li');
  const time = new Date().toLocaleTimeString('zh-CN');
  li.textContent = `[${time}] ${message}`;
  list.insertBefore(li, list.firstChild);
  while (list.children.length > 20) list.removeChild(list.lastChild);
}

function initMap() {
  map = L.map('map', {
    center: INITIAL_VIEW,
    zoom: INITIAL_ZOOM,
    zoomControl: true,
    attributionControl: false,
  });

  // ArcGIS World Imagery（WGS-84 卫星图，与 GNSS 坐标系对齐）
  tilesLayer = L.tileLayer(TILES_URL, {
    maxZoom: 18,
    minZoom: 12,
  });
  tilesLayer.addTo(map);
  tilesLayer.on('tileerror', (e) => {
    log(`WARN: 瓦片加载失败 z=${e.coords.z} x=${e.coords.x} y=${e.coords.y}`);
  });

  log(`地图已加载（ArcGIS World Imagery，WGS-84）`);

  map.on('move', () => {
    const c = map.getCenter();
    document.getElementById('map-center').textContent =
      `${c.lat.toFixed(5)}, ${c.lng.toFixed(5)}`;
  });

  map.on('click', (e) => {
    // 地图点击得到的是 GCJ-02 坐标（高德底图），发 /goal_gps 前需转 WGS-84
    const [wLat, wLng] = gcjToWgs(e.latlng.lat, e.latlng.lng);
    setGoal(wLat, wLng, 'map_click', '地图点击');
  });
}

// 定位到当前位置（一键）
function locateSelf() {
  if (!selfMarker) {
    log('⚠️ 当前位置未知（EC20 未定位），无法定位');
    return;
  }
  const ll = selfMarker.getLatLng();
  map.setView(ll, 18, { animate: true });
  log(`📍 已定位到当前位置：${ll.lat.toFixed(6)}, ${ll.lng.toFixed(6)}`);
}

function setGoal(lat, lng, source, label) {
  // lat/lng 是 WGS-84（来自 POI 或地图点击转换）
  // 新终点：清除 clear_goal grace period，否则新算出的路径会被前端忽略
  clearGoalGraceUntil = 0;
  document.getElementById('goal-lat').textContent = lat.toFixed(6);
  document.getElementById('goal-lng').textContent = lng.toFixed(6);
  document.getElementById('goal-source').textContent = label;
  document.getElementById('clear-goal-btn').disabled = false;

  // 画 marker 用 GCJ-02（高德地图对齐）
  const [gLat, gLng] = wgsToGcj(lat, lng);
  if (goalMarker) map.removeLayer(goalMarker);
  goalMarker = L.marker([gLat, gLng], {
    icon: L.divIcon({
      className: 'goal-marker',
      html: '<div style="background:#e07b6e;width:24px;height:24px;border-radius:50% 50% 50% 0;transform:rotate(-45deg);border:2px solid white;box-shadow:0 2px 4px rgba(0,0,0,0.3);"></div>',
      iconSize: [24, 24],
      iconAnchor: [12, 24],
    }),
  }).addTo(map);

  log(`终点已设：${lat.toFixed(6)}, ${lng.toFixed(6)}（${label}）`);

  publishGoal(lat, lng, source, label);
}

function publishGoal(lat, lng, source, label) {
  if (!goalTopic) {
    log('WARN: rosbridge 未连接，终点未发布');
    return;
  }
  const msg = new ROSLIB.Message({
    header: { stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 }, frame_id: 'wgs84' },
    source: source,
    poi_name: source === 'poi' ? label : '',
    latitude: lat,
    longitude: lng,
    altitude: 0.0,
  });
  goalTopic.publish(msg);
  log(`已发布 /goal_gps`);
}

function clearGoal() {
  if (goalMarker) {
    map.removeLayer(goalMarker);
    goalMarker = null;
  }
  if (routeLayer) {
    map.removeLayer(routeLayer);
    routeLayer = null;
  }
  // 清除所有规划点 markers（起点/拐角/终点）
  waypointMarkers.forEach(m => map.removeLayer(m));
  waypointMarkers = [];
  // 清除当前目标脉冲 marker
  if (currentWaypointMarker) {
    map.removeLayer(currentWaypointMarker);
    currentWaypointMarker = null;
  }
  // 设置 grace period：2 秒内忽略后端迟到的 /global_plan（race condition 防护）
  clearGoalGraceUntil = Date.now() + 2000;
  // 通知后端 NetworkX planner 停止定时刷新（防止 3 秒后路径重新出现）
  if (ros && ros.isConnected) {
    const clearTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/clear_goal',
      messageType: 'std_msgs/msg/Empty',
    });
    clearTopic.publish({});
  }
  document.getElementById('goal-lat').textContent = '--';
  document.getElementById('goal-lng').textContent = '--';
  document.getElementById('goal-source').textContent = '--';
  document.getElementById('clear-goal-btn').disabled = true;
  log('终点已清除（含路径、规划点、后端定时刷新，2s grace period 生效）');
}

// 更新自身位置标记
function updateSelfPosition(lat, lng, heading) {
  const [gLat, gLng] = wgsToGcj(lat, lng);
  const latlng = [gLat, gLng];

  // 自身位置标记（带方向箭头的圆）
  if (selfMarker) {
    selfMarker.setLatLng(latlng);
  } else {
    selfMarker = L.marker(latlng, {
      icon: L.divIcon({
        className: 'self-marker',
        html: `
          <div style="position:relative;width:32px;height:32px;">
            <div style="position:absolute;inset:0;background:#2980b9;border:2px solid white;border-radius:50%;box-shadow:0 2px 6px rgba(0,0,0,0.4);"></div>
            <div id="self-arrow" style="position:absolute;left:50%;top:50%;width:0;height:0;border-left:6px solid transparent;border-right:6px solid transparent;border-bottom:14px solid #fff;transform-origin:50% 100%;transform:translate(-50%,-100%) rotate(0deg);"></div>
          </div>`,
        iconSize: [32, 32],
        iconAnchor: [16, 16],
      }),
    }).addTo(map);
    // marker 刚创建，立即用当前已知 heading 应用（处理 /heading_imu 先到的情况）
    if (currentHeading !== null) {
      const arrowEl = document.querySelector('#self-arrow');
      if (arrowEl) {
        arrowEl.style.transform = `translate(-50%,-100%) rotate(${currentHeading}deg)`;
      }
    }
  }

  // 精度圆（10m 半径示意）
  if (selfCircle) {
    selfCircle.setLatLng(latlng);
  } else {
    selfCircle = L.circle(latlng, {
      radius: 10,
      color: '#2980b9',
      fillColor: '#2980b9',
      fillOpacity: 0.15,
      weight: 1,
    }).addTo(map);
  }

  // 更新箭头方向（如果传了 heading 参数）
  if (heading !== undefined && heading !== null) {
    currentHeading = heading;
    const arrowEl = document.querySelector('#self-arrow');
    if (arrowEl) {
      arrowEl.style.transform = `translate(-50%,-100%) rotate(${heading}deg)`;
    }
  }
}

// ============ DGPS 双 GPS 显示与评估 ============
function haversine(a, b) {
  // a, b: {lat, lng} 度；返回米
  const R = 6371000;
  const dLat = (b.lat - a.lat) * Math.PI / 180;
  const dLng = (b.lng - a.lng) * Math.PI / 180;
  const la1 = a.lat * Math.PI / 180;
  const la2 = b.lat * Math.PI / 180;
  const h = Math.sin(dLat/2) ** 2 + Math.cos(la1) * Math.cos(la2) * Math.sin(dLng/2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}

function mean(arr) {
  if (!arr.length) return null;
  return arr.reduce((s, x) => s + x, 0) / arr.length;
}

function fmtMeters(m) {
  if (m === null || m === undefined) return '--';
  if (m < 1000) return `${m.toFixed(1)} m`;
  return `${(m/1000).toFixed(3)} km`;
}

// ============ 规划点 markers（D6）============
function updateWaypointMarkers(poses) {
  // 清除旧 markers
  waypointMarkers.forEach(m => map.removeLayer(m));
  waypointMarkers = [];

  if (!poses || !Array.isArray(poses)) return;

  poses.forEach((pose) => {
    const [lat, lng] = wgsToGcj(pose.position.y, pose.position.x);
    // orientation.z 编码 type: 0=start, 0.5=corner, 1=goal
    const typeCode = pose.orientation.z;
    let className = 'waypoint-corner';
    let label = '拐角';
    if (Math.abs(typeCode) < 0.1) {
      className = 'waypoint-start';
      label = '起点';
    } else if (Math.abs(typeCode - 1.0) < 0.1) {
      className = 'waypoint-goal';
      label = '终点';
    }
    const icon = L.divIcon({
      className: '',
      html: `<div class="${className}" style="width:14px;height:14px;"></div>`,
      iconSize: [14, 14]
    });
    const m = L.marker([lat, lng], { icon: icon, title: label });
    m.addTo(map);
    waypointMarkers.push(m);
  });
}

function updateCurrentWaypoint(lat, lng) {
  const [gLat, gLng] = wgsToGcj(lat, lng);
  if (currentWaypointMarker) {
    map.removeLayer(currentWaypointMarker);
  }
  const icon = L.divIcon({
    className: '',
    html: `<div class="waypoint-current" style="width:18px;height:18px;"></div>`,
    iconSize: [18, 18]
  });
  currentWaypointMarker = L.marker([gLat, gLng], { icon: icon, title: '当前目标' });
  currentWaypointMarker.addTo(map);
}

// clear_goal grace period：清除终点后短时间内忽略迟到的 /global_plan，
// 防止后端 race condition（clear_goal 与已发出的 /global_plan 同时在路上）
let clearGoalGraceUntil = 0;

// 处理 /global_plan 消息：渲染路径线 + 调整视野 + 记录日志
function handleGlobalPlan(msg) {
  // grace period 内忽略后端迟到的路径（前端已清，不应被重新画上）
  if (Date.now() < clearGoalGraceUntil) {
    console.log('[nav] 忽略 clear_goal grace period 内的 /global_plan');
    return;
  }

  if (routeLayer) {
    map.removeLayer(routeLayer);
    routeLayer = null;
  }

  if (msg.status !== 'OK') {
    log(`⚠️ 路径规划失败：${msg.error_message || msg.status}`);
    return;
  }

  const latlngs = msg.path_wgs84.map(p => { const [la, lo] = wgsToGcj(p.y, p.x); return [la, lo]; });

  routeLayer = L.polyline(latlngs, {
    color: '#22c55e',
    weight: 5,
    dashArray: '8,8',
  });
  routeLayer.addTo(map);

  const srcLabel = msg.source === 'public_osrm' ? '公共'
                 : (msg.source === 'noop' ? '原点' : '本地');
  log(`✅ 路径已规划 [${srcLabel}]：${msg.distance_meters.toFixed(0)} 米，约 ${msg.duration_seconds.toFixed(0)} 秒`);

  const bounds = L.latLngBounds(latlngs);
  map.fitBounds(bounds, { padding: [50, 50] });
}

async function loadPoi() {
  try {
    const resp = await fetch(POI_URL);
    const geojson = await resp.json();
    poiLayer = L.geoJSON(geojson, {
      pointToLayer: (feature, latlng) => L.circleMarker(latlng, {
        radius: 6,
        color: '#2980b9',
        fillColor: '#2980b9',
        fillOpacity: 0.8,
      }).bindTooltip(feature.properties.name),
    }).addTo(map);
    const list = document.getElementById('poi-list');
    list.innerHTML = '';
    geojson.features.forEach((feature) => {
      const [lng, lat] = feature.geometry.coordinates;
      const li = document.createElement('li');
      li.textContent = feature.properties.name;
      li.onclick = () => {
        setGoal(lat, lng, 'poi', feature.properties.name);
        map.setView([lat, lng], 18);
      };
      list.appendChild(li);
    });
    log(`加载了 ${geojson.features.length} 个 POI`);
  } catch (err) {
    document.getElementById('poi-list').innerHTML = '<li class="loading">POI 加载失败</li>';
    log(`ERROR: POI 加载失败 ${err.message}`);
  }
}

function initRosbridge() {
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on('connection', () => {
    document.getElementById('connection-status').textContent = '已连接 ROS2';
    document.getElementById('connection-status').className = 'value connected';

    // 发布 /goal_gps
    goalTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/goal_gps',
      messageType: 'rtk_msgs/msg/GoalGPS',
    });

    // 订阅 /fix（自身位置：DGPS 模式下是修正后位置，非 DGPS 模式下是 EC20 直发）
    fixListener = new ROSLIB.Topic({
      ros: ros,
      name: '/fix',
      messageType: 'sensor_msgs/NavSatFix',
    });
    let hasRealFix = false;
    fixListener.subscribe((msg) => {
      lastFixMsgTime = Date.now();
      const sats = (msg.position_covariance && msg.position_covariance[0]) || 0;
      const hdop = (msg.position_covariance && msg.position_covariance[4]) || 0;
      // ROS2 NavSatFix.status.status: -1=NO_FIX, 0=STATUS_FIX
      if (msg.status && msg.status.status >= 0) {
        // 成功定位
        if (!hasRealFix) {
          hasRealFix = true;
          log('✅ 收到 /fix 定位');
        }
        const satsInt = Math.round(sats);
        updateGpsState('fix', `${satsInt} 颗卫星 · HDOP ${hdop.toFixed(1)}`);
        updateSelfPosition(msg.latitude, msg.longitude, currentHeading);
        lastFix = { lat: msg.latitude, lng: msg.longitude, t: Date.now() };
      } else {
        // 信号弱：节点在跑但没锁定卫星（fix_quality=0 时节点仍发布 /fix(status=-1)）
        updateGpsState('weak', `正在搜索卫星… (可见 ${Math.round(sats)} 颗)`);
      }
    });

    // 启动冷启动超时检测（5 秒没收到 /fix → 切回冷启动）
    startGpsTimeoutWatchdog();

    // 订阅所有规划点（PoseArray，frame_id="wgs84"）
    allWaypointsListener = new ROSLIB.Topic({
      ros: ros,
      name: '/all_waypoints_wgs84',
      messageType: 'geometry_msgs/PoseArray'
    });
    allWaypointsListener.subscribe((msg) => {
      updateWaypointMarkers(msg.poses);
    });

    // 订阅当前目标点（NavSatFix）
    nextWaypointListener = new ROSLIB.Topic({
      ros: ros,
      name: '/next_waypoint_wgs84',
      messageType: 'sensor_msgs/NavSatFix'
    });
    nextWaypointListener.subscribe((msg) => {
      updateCurrentWaypoint(msg.latitude, msg.longitude);
    });

    // 订阅 /heading_imu（IMU 积分航向，throttle 到 30Hz 避免 rosbridge 拥塞）
    const imuHeadingListener = new ROSLIB.Topic({
      ros: ros,
      name: '/heading_imu',
      messageType: 'std_msgs/Float64',
      throttle_rate: 33,  // 33ms = ~30Hz
    });
    imuHeadingListener.subscribe((msg) => {
      currentHeading = msg.data;
      // 更新地图上箭头（统一用 #self-arrow ID 选择器，避免 selfMarker 未创建时找不到）
      const arrowEl = document.querySelector('#self-arrow');
      if (arrowEl) {
        arrowEl.style.transform = `translate(-50%,-100%) rotate(${msg.data}deg)`;
      }
      // 更新侧边栏航向显示
      const headingEl = document.getElementById('heading-value');
      if (headingEl) {
        headingEl.textContent = `${msg.data.toFixed(1)}°`;
      }
    });

    // 订阅 HWT IMU /imu/data（含 gyro/accel/orientation）—— throttle 到 20Hz
    const imuListener = new ROSLIB.Topic({
      ros: ros,
      name: '/imu/data',
      messageType: 'sensor_msgs/Imu',
      throttle_rate: 50,  // 50ms = 20Hz
    });
    imuListener.subscribe((msg) => {
      // 四元数 → 欧拉角（ZYX 顺序）
      const q = msg.orientation;
      if (!q || (q.x === 0 && q.y === 0 && q.z === 0 && q.w === 0)) return;
      const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
      const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
      const roll = Math.atan2(sinr_cosp, cosr_cosp);
      const sinp = 2 * (q.w * q.y - q.z * q.x);
      const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
      const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      const yaw = Math.atan2(siny_cosp, cosy_cosp);

      const rollDeg = roll * 57.2958;
      const pitchDeg = pitch * 57.2958;
      const yawDeg = yaw * 57.2958;

      // 角速度（rad/s → °/s）
      const gx = (msg.angular_velocity.x || 0) * 57.2958;
      const gy = (msg.angular_velocity.y || 0) * 57.2958;
      const gz = (msg.angular_velocity.z || 0) * 57.2958;
      // 加速度（m/s²，含重力）
      const ax = msg.linear_acceleration.x || 0;
      const ay = msg.linear_acceleration.y || 0;
      const az = msg.linear_acceleration.z || 0;

      // 姿态指示器：roll 旋转、pitch 平移
      const pitchClamped = Math.max(-30, Math.min(30, pitchDeg));
      const horizon = document.getElementById('attitude-horizon');
      if (horizon) {
        horizon.style.transform =
          `rotate(${-rollDeg.toFixed(2)}deg) translateY(${(pitchClamped * 1.5).toFixed(1)}px)`;
      }
      const rollArrow = document.getElementById('attitude-roll-arrow');
      if (rollArrow) {
        rollArrow.style.transform = `translateX(-50%) rotate(${rollDeg.toFixed(2)}deg)`;
      }

      const set = (id, val) => {
        const el = document.getElementById(id);
        if (el) el.textContent = val;
      };
      set('imu-roll-pitch', `${rollDeg.toFixed(1)}° / ${pitchDeg.toFixed(1)}°`);
      set('imu-yaw', yawDeg.toFixed(1) + '°');
      set('imu-gz', gz.toFixed(2));
    });

    // 如果 1 秒内 EC20 还没定位，先用 INITIAL_VIEW 显示位置标记
    setTimeout(() => {
      if (!hasRealFix && !selfMarker) {
        updateSelfPosition(INITIAL_VIEW[0], INITIAL_VIEW[1], 0);
        log('⚠️ EC20 未定位，先用默认位置（昆工中心广场）显示，等真实定位后切换');
      }
    }, 1000);

    // 订阅 /global_plan（由 rtk_planner/osrm_planner_node 发布）
    const planListener = new ROSLIB.Topic({
      ros: ros,
      name: '/global_plan',
      messageType: 'rtk_msgs/msg/GlobalPlan',
    });
    planListener.subscribe(handleGlobalPlan);

    log('rosbridge 已连接，订阅 /fix + /heading_imu + /global_plan + /all_waypoints_wgs84 + /next_waypoint_wgs84');
  });

  ros.on('error', (error) => {
    document.getElementById('connection-status').textContent = 'ROS2 连接错误';
    log(`ERROR: rosbridge ${error}`);
  });

  ros.on('close', () => {
    document.getElementById('connection-status').textContent = 'ROS2 已断开';
    document.getElementById('connection-status').className = 'value disconnected';
    updateGpsState('cold', '等待 ROS2 连接');
    log('rosbridge 已断开，5 秒后重连');
    setTimeout(initRosbridge, 5000);
  });
}

window.addEventListener('DOMContentLoaded', () => {
  initMap();
  loadPoi();
  initRosbridge();
  document.getElementById('clear-goal-btn').onclick = clearGoal;
  // 侧边栏"定位到当前位置"按钮
  const locateBtn = document.getElementById('locate-self-btn');
  if (locateBtn) locateBtn.onclick = locateSelf;

  // PWA Service Worker 已禁用（开发期间总是缓存旧 app.js 导致看不到 markers 更新）
  // 如果将来需要 PWA 离线支持，取消下面的注释
  // if ('serviceWorker' in navigator) {
  //   navigator.serviceWorker.register('./sw.js').then((reg) => {
  //     console.log('[PWA] Service Worker 注册成功:', reg.scope);
  //   }).catch((err) => {
  //     console.warn('[PWA] Service Worker 注册失败:', err);
  //   });
  // }
});
