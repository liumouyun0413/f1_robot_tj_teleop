# f1_tianji_teleop —— F1 轮式人形机器人 PICO 遥操作

按 XRoboToolkit-Teleop-Sample-Python 的 **Galaxea R1 Lite 真机示例**范式重写的 F1 遥操作程序，
功能对齐 `lxl_0918_add_head` 版本：双臂 IK 遥操、夹爪、头部摇杆、腰部升降、底盘摇杆、
质心/碰撞安全门、ros2 bag 数采、复位、外部位姿回放。

## 目录结构

```
f1_tianji_teleop/
├── start_teleop.sh / stop_teleop.sh      一键启停（腰部节点 + RoboticsService + 遥操主程序）
├── scripts/teleop_f1_hardware.py         入口（≈ 官方 teleop_r1lite_hardware.py）
├── f1_teleop/                            Python 包
│   ├── config/f1_teleop.yaml             ★ 全部参数（话题、关节映射、位姿、阈值、安全、数采）
│   ├── common/
│   │   ├── xr_client.py                  PICO SDK 封装（官方同款）
│   │   ├── base_teleop_controller.py     Placo IK 核心：三模式、正则化、构型同步、输入抽象
│   │   └── hardware_teleop_controller.py 多线程骨架：ik / control / aux 线程 + run()
│   ├── hardware/
│   │   ├── f1_teleop_controller.py       ★ F1 控制器（≈ 官方 galaxea_r1_lite_teleop_controller.py）
│   │   └── interface/                    硬件接口（≈ 官方 hardware/interface/galaxea.py）
│   │       ├── ros_context.py            进程内唯一 rclpy 节点 + spin 线程
│   │       ├── f1_arm.py                 双臂指令/反馈/夹爪/插值运动
│   │       ├── f1_head.py                头部三关节增量控制
│   │       ├── f1_waist.py               腰部升降（→ EC_Line_Control 节点）
│   │       ├── f1_chassis.py             底盘速度 + SetControlMode
│   │       ├── safety_guard.py           质心/碰撞安全门（主进程客户端）
│   │       ├── safety_worker.py          安全检测子进程（持有 cyb SafetyProtector）
│   │       ├── recording.py              ros2 bag 录制 + 外部相机录制器信号
│   │       ├── lead_publisher.py         主端数据镜像 /lead/*
│   │       └── external_pose_input.py    外部位姿回放输入
│   └── utils/                            geometry / control_utils(JointMap, 边沿, 限频) / path_utils
├── assets/f1_wheel/                      URDF + meshes
├── EC_Line_Control/                      （已有）腰部升降 ROS2 节点
└── interface_20260623/                   （已有）ROS2 接口定义（move/srv/SetControlMode 等）
```

## 运行

```bash
# 机器人上（conda pico4ultra_py312 + ROS2 jazzy + interface 工作区已编译）
./start_teleop.sh                       # TELEOP_ARGS="--visualize-placo" ./start_teleop.sh
tail -f log/teleop_*.log
./stop_teleop.sh

# 手动运行
source /opt/ros/jazzy/setup.bash && source ~/workspace/f1/install/setup.bash
conda activate pico4ultra_py312
python scripts/teleop_f1_hardware.py --help
```

启动流程：等待 `/hal/joint_states` → 头部目标取真实角度 → 双臂插值回 `home_pose`（3s）→
Placo 同步到该位姿 → 自检反馈偏差 → 启动安全门子进程 → 启动 ik / control / aux 三线程。

## 手柄操作

| 输入 | 功能 |
|---|---|
| 握把 > 0.9 | 激活该臂，手柄位姿增量驱动末端 |
| 扳机 | 夹爪（0→张开指令 100，1→闭合指令 0） |
| 左摇杆按压 / 右摇杆按压 / 同时 | 末端模式 position / orientation / pose |
| X + Y | 双臂复位到 `reset_pose`（2s 插值，Placo 同步） |
| Y 长按 0.3s / X 长按 0.3s | 腰部上升 / 下降一步 |
| A / B | 开始 / 停止数采（ros2 bag + 外部录制器） |
| 右扳机按住且双握把松开 | 头部模式：左摇杆 X/Y → roll/pitch，右摇杆 X → yaw |
| 右摇杆上拨 / 下拨 | 底盘使能 / 停止；使能后左摇杆平移、右摇杆 X 旋转 |

## 关键设计

* **单进程**：lxl 版的 UDP→bridge 双进程结构在同一台机器上没有必要，改为直接发布 ROS2 话题；
  复位/回零的插值运动由本进程完成，`/lead/joint_states` 自动与之同步。
* **开环虚拟机器人**：Placo 只在启动/复位时与真机同步一次，之后仅由手柄驱动，
  避免与滞后的真机位置控制互相拉扯（与 lxl 版一致，但去掉了三套偏差补偿变量）。
* **统一关节映射** `JointMap`：`deg = sign·rad2deg(rad) + offset`，正反两向共用一组参数，
  参数由 lxl 版 `angle_map` + `sync_placo_to_real` + 启动偏差补偿合并推导（见 yaml 注释）。
* **安全门**：cyb `SafetyProtector` 在子进程中运行；冻结时回退到**最近一次通过检查**的位姿；
  配置/URDF 加载失败直接报错退出而不是静默直通；下发指令另有每关节速度限幅。
* **所有魔法数字进 yaml**：话题名、关节名、位姿、阈值、权重、限位。

## 依赖

ROS2 jazzy（rclpy, sensor_msgs, control_msgs, geometry_msgs, std_msgs），`move` 接口包（interface_20260623），
`xrobotoolkit_sdk`，`placo==0.9.4`，`meshcat`，`numpy`，`pyyaml`，`tyro`；
安全门需 `cyb_safe_control_real`（路径见 yaml `safety.module_dir`）。
