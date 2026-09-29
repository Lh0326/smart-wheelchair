# 部署与运行指南

[返回项目首页](../README.md)

本指南面向 Ubuntu 22.04 / ROS 2 Humble 的源码部署。以下命令均在 **Bash** 中执行；项目尚无锁定全部依赖版本的安装器，完整启动需要匹配的驱动、模型和设备。文档中的入口依据当前源码核对，不代表已在全新机器上完成复现。

## 1. 准备环境

| 类别 | 需要准备的内容 |
| --- | --- |
| 系统 | Ubuntu 22.04、ROS 2 Humble、Python 3.10、colcon、rosdep；图形界面需要桌面会话与有效 `DISPLAY` |
| ROS 组件 | Nav2、robot_localization、rosbridge_server、RViz、cv_bridge、depthimage_to_laserscan、web_video_server、xacro |
| Python 组件 | PyQt5 / QtWebEngine、NumPy、SciPy、scikit-learn、joblib、pyserial、psutil、OSMnx、NetworkX、Shapely、Flask、音频与推理依赖 |
| 独立驱动 | 与 Gemini 335L 匹配的 `orbbec_camera` 包及设备权限；主启动文件要求存在 `gemini_330_series.launch.py` |
| 仓库内第三方包 | `third_party/teb_ws_src`、`third_party/ldlidar_src`、`third_party/ladar_ai`；按下文分层构建 |
| 数据 | 已附带示例区域 GraphML 路网和 MBTiles；其他地区需另行准备，不能直接复用示例位置 |

先按 [ROS 2 Humble 安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)准备 ROS 环境，按 [Orbbec 官方说明](https://github.com/orbbec/OrbbecSDK_ROS2)安装相机驱动并加载其工作空间。相机驱动不包含在本仓库内；不同版本的参数和 launch 文件需要对照核实。

```bash
git clone https://github.com/Lh0326/smart-wheelchair.git
cd smart-wheelchair
source /opt/ros/humble/setup.bash
```

`package.xml` 可以帮助安装 ROS 依赖，但还未完整声明所有图形、推理与外设运行依赖。下面的 rosdep 命令以 **rosdep 已初始化并更新索引** 为前提；`rtk_map` 使用的 `Flask` 名称需单独处理：

```bash
sudo apt install python3-flask
rosdep install --from-paths src third_party/teb_ws_src \
  third_party/ldlidar_src third_party/ladar_ai \
  --ignore-src --rosdistro humble --skip-keys Flask -r -y
```

语音 / 视觉的部分 Python 依赖列在 [requirements.txt](../third_party/ladar_ai/requirements.txt)，OpenVINO、QtWebEngine 和脑控 / 地图相关依赖需另行补齐。安装到与 ROS 2 节点相同的 Python 3.10 环境，并保留实际使用的版本记录；该 requirements 不是整个项目的完整依赖锁文件。

## 2. 模型与资源

大体积推理模型托管在 [Hugging Face](https://huggingface.co/lh527/smart-wheelchair/tree/main)，Git 仓库仅附带脑控模块中的少量模型文件。模型仓库自身包含 `models/` 顶层目录，因此从**本项目根目录**下载：

```bash
python3 -m pip install -U huggingface_hub
hf download lh527/smart-wheelchair --include 'models/*' --local-dir .
```

预期目录：

```text
models/
├── eeg/
├── voice/
│   ├── kws/
│   ├── asr/
│   └── tts/
└── yolo/
    └── yolo11s_int8_openvino/
```

不要把上述命令的目标设为 `models/`，否则会形成 `models/models/`。支持该变量的节点可通过 `MODELS_ROOT` 指定模型目录；旧语音组件的配置还需单独核对，见下文。

当前 [camera_detect_node.py](../src/rtk_perception/rtk_perception/camera_detect_node.py)显式指定 `models/yolo/yolo11s_openvino/yolo11s.xml`。如果下载的是 `yolo11s_int8_openvino` 包，且 `yolo11s_openvino` 路径尚不存在，可建立兼容别名：

```bash
# 已有同名目录或符号链接时先核对内容，不覆盖现有模型
if [ ! -e models/yolo/yolo11s_openvino ] && [ ! -L models/yolo/yolo11s_openvino ]; then
  ln -s yolo11s_int8_openvino models/yolo/yolo11s_openvino
fi
test -f models/yolo/yolo11s_openvino/yolo11s.xml
```

OpenVINO 设备回退由 [yolo_engine.py](../third_party/ladar_ai/src/ladar_ai/yolo_engine.py)处理。回退机制不能替代缺失的模型或设备驱动；最终执行设备应以运行日志为准。

### 语音模型的版本差异

当前 [voice_node.py](../src/wheelchair_app/wheelchair_app/nodes/voice_node.py)调用的是 [VoicePipeline](../third_party/ai_model_src/voice_pipeline.py) 中的 `OfflineRecognizer.from_zipformer_ctc`。其 [config.py](../third_party/ai_model_src/config.py) 在 Linux 上默认把 `third_party/models` 当作模型根目录，并要求：

- `voice/asr-offline/manyeyes/k2transducer-zipformer-ctc-zh-onnx-offline-20250703/model.int8.onnx` 与 `tokens.txt`；
- `voice/config/keywords.txt`；
- 配套 KWS 与 TTS 资源。

2026-09-29 核对公开模型文件清单时，下载包中的 ASR 为 `voice/asr/sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30`，未包含上述离线 ASR 和关键词配置。两种识别器的文件结构不同，不能仅改目录名互换；需要补齐与现有 `VoicePipeline` 匹配的资源，并将配置中的根目录对齐。只设置 `MODELS_ROOT` 或执行下载命令不会自动解决这一差异。

## 3. 构建工作空间

[source_env.sh](../source_env.sh) 会加载以下构建产物：

| 源码目录 | 安装目录 | 用途 |
| --- | --- | --- |
| `third_party/teb_ws_src` | `third_party/teb_ws_install` | TEB 与 costmap_converter |
| `third_party/ldlidar_src` | `third_party/ldlidar_install` | LD14P 驱动 |
| `third_party/ladar_ai` | `third_party/ladar_ai_install` | 本地语音与视觉支持、消息类型 |
| `src` | `install` | 本项目九个 ROS 包 |

**构建前需处理的资源差异：** [ladar_ai/CMakeLists.txt](../third_party/ladar_ai/CMakeLists.txt) 的资源安装列表还包含 `frontend`、`models`、`worlds`、`rviz`，公开快照没有这些目录。应按所用模块恢复对应资源，或在本地将安装列表调整为实际分发的目录；否则该包可能在安装阶段失败。根目录下载的 `models/` 不会自动补齐这份旧资源安装列表。

完成依赖与资源准备后，可按以下顺序构建；每一步成功后再继续：

```bash
colcon --log-base third_party/teb_ws_log build \
  --base-paths third_party/teb_ws_src \
  --build-base third_party/teb_ws_build \
  --install-base third_party/teb_ws_install
source third_party/teb_ws_install/setup.bash

colcon --log-base third_party/ldlidar_log build \
  --base-paths third_party/ldlidar_src \
  --build-base third_party/ldlidar_build \
  --install-base third_party/ldlidar_install
source third_party/ldlidar_install/setup.bash

colcon --log-base third_party/ladar_ai_log build \
  --base-paths third_party/ladar_ai \
  --build-base third_party/ladar_ai_build \
  --install-base third_party/ladar_ai_install
source third_party/ladar_ai_install/setup.bash

colcon build --base-paths src --symlink-install
source source_env.sh
```

当前主启动入口使用 [scripts/n10p_python_driver.py](../scripts/n10p_python_driver.py) 驱动 N10P；`third_party/lidar_n10p_src` 保留 C++ 驱动源码，仅切换回该实现时才需要构建到 `third_party/lidar_n10p_install`。

## 4. 核对硬件与参数

| 设备 / 配置 | 当前默认值或入口 | 调整方式 |
| --- | --- | --- |
| 底盘串口 | `/dev/wheelchair_chassis`，脚本也会探测原设备的 by-path | `CHASSIS_SERIAL_PORT` 环境变量 |
| 车体 IMU | `/dev/ttyIMU`，脚本也会探测原设备的 by-path | `IMU_SERIAL_PORT` 环境变量 |
| N10P | `/dev/lidar_n10p`，脚本尝试 by-id 回退 | `N10P_SERIAL_PORT` 环境变量 |
| LD14P | `/dev/LD14P` | 当前写在主 launch 内，需按实际端口调整或设置相应 udev 别名 |
| EC20 GNSS | `/dev/ttyUSB_AT`、`/dev/ttyUSB_NMEA` | 直接启动 launch 时传 `gnss_at_port`、`gnss_nmea_port` |
| EEG 与头部 IMU | 脑控页面及 `braincontrol` 配置 | 核对采集设备、连接参数、姿态零点与个体模型 |
| 坐标与机身尺寸 | URDF、静态 TF、扫描屏蔽矩形、costmap 参数 | 更换车架或传感器位置后重新标定 |

车体 IMU 与头部 IMU 是两套用途不同的设备。仓库中的 USB 路径、姿态符号、扫描屏蔽范围与底盘协议针对原型硬件，不应直接当作任意轮椅的通用配置。

## 5. 启动、观察与停止

以下命令从仓库根目录执行。仓库中的启停脚本没有可执行位，使用 `bash` 调用即可。

```bash
source source_env.sh
USE_REAL_CHASSIS=0 USE_REAL_IMU=0 bash start/start_all.sh
```

上述模式只模拟底盘及车体 IMU / GNSS，雷达、相机、ASR 与 YOLO 仍会启动。项目没有通过这两个开关提供全传感器模拟环境。

完成实物设备配置后，启用真实底盘：

```bash
source source_env.sh
USE_REAL_CHASSIS=1 USE_REAL_IMU=1 bash start/start_all.sh
```

`start_all.sh` 默认就是这组实物开关；`USE_REAL_CHASSIS=1` 会强制 `USE_REAL_IMU=1`。脚本启动前还会清理匹配的已有项目 / ROS 进程，应在专用调试环境使用，避免与其他 ROS 任务共用同一会话。

| 观察入口 | 位置 |
| --- | --- |
| 主窗口 | PyQt5 三页面及 RViz |
| 导航网页 | `http://localhost:8000/nav/index.html` |
| 陪伴网页 | `http://localhost:8000/companion/index.html` |
| 地图服务健康检查 | `http://localhost:8080/health` |
| 相机 JPEG 服务 | `http://localhost:8086` |
| ROS WebSocket | `ws://localhost:9091` |
| 日志 | `/tmp/rtk_logs/sim_nav.log`、`wheelchair_main.log`、`voice_node.log` |

启动脚本的完成提示不能代替运行状态检查。在另一个已加载 `source_env.sh` 的终端中检查：

```bash
ros2 lifecycle get /controller_server
ros2 action info /follow_path
ros2 topic hz /scan
ros2 topic echo /cmd_vel_safe --once
curl -fsS http://localhost:8080/health
```

检查目标为：控制器处于 `active`，`/follow_path` 有服务端，雷达扫描持续更新，地图服务可访问。仅有非空话题或速度消息不能证明端到端导航和实物执行正确。

停止：

```bash
bash start/stop_all.sh
```

停止脚本会按名称匹配进程，作用范围不限于某一个启动进程；实物断连时是否停车仍需由下位机看门狗和硬件验证。

## 6. 验证与排查

源码中的纯算法测试可先在不启动 ROS 节点的情况下运行，例如已安装 `pytest` 和 `numpy` 时：

```bash
PYTHONPATH="$PWD/src/rtk_perception${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m pytest src/rtk_perception/tests/test_safety_chain.py -q
```

完整 ROS 包测试应在依赖构建完成后执行：

```bash
colcon test --base-paths src --packages-select rtk_perception
colcon test-result --verbose
```

不同测试涉及不同依赖；不要把某个纯算法测试通过等同于所有 ROS、语音、脑控或硬件链路验证通过。历史自动化与硬件联调结果见 [tests/results](../tests/results)；[L 形路径检查](../tests/sim_teb_l_shape_test.md)属于人工验收流程。

| 现象 | 优先检查 |
| --- | --- |
| ROS 找不到包或 TEB 插件 | 对应安装目录是否构建成功；当前终端是否加载 `source_env.sh` 及 Orbbec 工作空间 |
| `ladar_ai` 安装阶段找不到资源目录 | 第 3 节所列 CMake 安装资源与公开快照差异 |
| 语音初始化失败 | 语音组件的模型根目录、离线 ASR 权重和 `keywords.txt`；参见第 2 节的版本差异 |
| 视觉模型加载失败 | `MODELS_ROOT`、YOLO XML / BIN 文件、路径别名与 OpenVINO 设备日志 |
| 地图或视频空白 | 本地 HTTP 服务与端口、MBTiles 路径、QtWebEngine / 桌面会话、相机话题 |
| 有路径但底盘无响应 | `/follow_path` 服务、控制器生命周期、`/scan` 有效性、定位与指令时效、导航门控；再检查底盘串口与协议 |
| 仅切换模拟底盘仍提示外设失败 | 雷达与相机不会被底盘开关关闭，按实际硬件准备或另行搭建模拟输入 |

本次文档维护核对了入口、路径、参数和历史记录，并修正了多个模块中被误放入文档字符串的工作空间路径初始化。尚未在目标 Ubuntu / ROS 2 环境进行完整构建和硬件端到端复测。
