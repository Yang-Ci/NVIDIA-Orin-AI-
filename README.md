# 基于 NVIDIA Orin 的智能机器人 AI 功能模块

基于 ROS2 Humble 开发的智能机器人 AI 功能模块，运行于 Jetson AGX Orin 平台，实现语音交互、视觉感知、运动控制的完整 AI 闭环。

## 项目架构

```
src_ros2/
├── app/                    # 应用层
│   ├── lidar_controller    # 激光雷达控制
│   ├── line_following      # 线跟随
│   └── object_tracking     # 目标追踪（PID控制）
│
├── bringup/                # 系统启动
│   └── startup_check       # 启动检查与服务
│
├── calibration/            # 标定工具
│   ├── calibrate_angular   # 角速度标定
│   └── calibrate_linear    # 线性标定
│
├── driver/                 # 驱动层
│   ├── controller/         # 底盘控制器
│   │   ├── ackermann       # 阿克曼底盘
│   │   └── mecanum         # 麦克纳姆底盘
│   ├── kinematics/         # 运动学模块
│   │   ├── forward_kinematics   # 正运动学
│   │   └── inverse_kinematics   # 逆运动学
│   ├── servo_controller/   # 舵机控制
│   └── sdk/                # SDK工具（PID/LED/FPS）
│
├── example/                # 示例应用
│   ├── ar_detect/          # AR 增强现实检测
│   ├── color_detect/       # 颜色检测
│   ├── color_sorting/      # 颜色分拣
│   ├── garbage_classification/  # 垃圾分类
│   ├── hand_gesture_control/    # 手势控制
│   ├── hand_track/         # 手部追踪
│   ├── mediapipe_example/  # MediaPipe 应用
│   │   ├── face_detect     # 人脸检测
│   │   ├── hand_gesture    # 手势识别
│   │   └── pose            # 姿态估计
│   ├── navigation_transport/    # 导航运输
│   ├── rgbd_function/      # RGB-D 深度相机功能
│   │   ├── track_and_grab  # 追踪抓取
│   │   └── prevent_falling # 防跌落
│   ├── self_driving/       # 自动驾驶演示
│   └── yolo_detect/        # YOLO 目标检测
│
└── large_models/           # 大模型/AI语音交互
    ├── agent_process       # Agent工具调用
    ├── vocal_detect        # 语音唤醒/识别
    └── tts_node            # TTS语音合成
```

## 功能模块

### 大模型与语音交互
- **多模型适配**：OpenAI GPT-4、阿里云通义千问、Ollama 本地模型
- **语音交互闭环**：语音唤醒 → ASR识别 → LLM推理 → TTS合成
- **Agent工具调用**：LLM 动态调用机器人控制接口（导航、抓取、追踪等）

### 视觉感知
- **YOLO目标检测**：TensorRT 加速，单帧推理 ≤16ms，支持 OBB 旋转目标检测
- **MediaPipe应用**：手势识别（12种手势）、人脸检测、姿态估计
- **颜色追踪**：LAB颜色空间 + PID控制器，追踪误差 ≤20像素

### 机器人控制
- **底盘控制**：阿克曼、麦克纳姆、差速底盘适配
- **运动学**：机械臂正/逆运动学求解，轨迹规划
- **舵机控制**：动作组编排，关节位置控制

### 智能应用
- 垃圾分类、自动驾驶演示、导航运输
- RGB-D 抓取避障、过桥、防跌落
- AR 增强现实检测

## 环境要求

- **硬件**：Jetson AGX Orin / Xavier / NX
- **系统**：Ubuntu 22.04
- **ROS**：ROS2 Humble
- **Python**：3.10+
- **依赖**：TensorRT、MediaPipe、OpenCV、PyTorch

## 快速开始

### 编译

```bash
cd src_ros2
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

### 启动

```bash
source install/setup.bash

# 启动系统
ros2 launch bringup bringup.launch.py

# 启动大模型语音交互
ros2 launch large_models start.launch.py

# 启动 YOLO 检测
ros2 launch example yolo_detect.launch.py

# 启动手势控制
ros2 launch example hand_gesture_control_node.launch.py

# 启动自动驾驶演示
ros2 launch example self_driving.launch.py
```

## 核心节点

| 功能包 | 节点 | 说明 |
|--------|------|------|
| large_models | agent_process | Agent工具调用处理 |
| large_models | vocal_detect | 语音唤醒与识别 |
| large_models | tts_node | TTS语音合成 |
| example | yolo_node | YOLO目标检测 |
| example | hand_gesture | 手势识别 |
| example | color_detect | 颜色检测 |
| example | self_driving | 自动驾驶演示 |
| driver | controller | 底盘运动控制 |
| driver | kinematics_control | 运动学控制 |

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      语音交互层                              │
│   语音唤醒 → ASR识别 → LLM推理 → TTS合成                    │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                      视觉感知层                              │
│   YOLO检测 │ MediaPipe │ 颜色追踪 │ RGB-D深度               │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                      决策规划层                              │
│   Agent工具调用 │ 状态机 │ 行为树                           │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                      运动控制层                              │
│   底盘控制 │ 运动学 │ 舵机控制                              │
└─────────────────────────────────────────────────────────────┘
```

## 服务接口

功能模块通过 `~/enter` 和 `~/exit` 服务实现热切换：

```python
# 进入功能模块
ros2 service call /yolo_detect/enter std_srvs/srv/Trigger

# 退出功能模块
ros2 service call /yolo_detect/exit std_srvs/srv/Trigger
```

## 许可证

MIT License
