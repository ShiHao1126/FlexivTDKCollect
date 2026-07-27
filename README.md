# FlexivTDKCollect

基于 Flexiv TDK LAN 透明遥操作的数据采集工具，用于同步采集 follower 机械臂状态、RealSense 原始图像以及 Xense 夹爪宽度和触觉数据。

## 环境安装

现有 `haptic_exo_env` 已包含 Flexiv TDK、Flexiv RDK、RealSense 和 Xense SDK，可以直接复用，无需安装本仓库：

```bash
conda activate haptic_exo_env
cd ~/haptic_exo_teleop_ws/shihao/FlexivTDKCollect
```

首次复用环境时可以检查依赖：

```bash
python -m pip check
python -c "import flexivtdk, flexivrdk, pyrealsense2, xensesdk, xensegripper"
```

## 数据采集

采集入口是 `scripts/collect.py`：

```bash
conda activate haptic_exo_env
cd ~/haptic_exo_teleop_ws/shihao/FlexivTDKCollect
python scripts/collect.py
```

设备序列号、Tool、采集开关、频率和保存路径统一在 `scripts/collect.py` 的 `CollectArgs` 中修改，不需要在终端传入参数。参数按 `Robot`、`Gripper`、`Camera` 分组。

- `enable_robot`：决定是否配置双臂 Tool、使能机械臂并启动 TDK。启用后固定采集关节位置/速度/力矩、TCP 位姿/速度，以及 TCP 和 world 坐标系下的末端 wrench。
- `enable_gripper`：决定是否连接 `gripper_macs` 中的夹爪并采集宽度。
- `enable_tactile`：决定是否连接 `tactile_sns` 中的触觉传感器，并固定采集 marker、force/torque、rectify、difference 和 depth。
- `enable_camera`：决定是否连接 `serials` 中的 RealSense；RGB 始终采集，`collect_depth` 决定是否同时采集原始 Depth。

RealSense 按序列号连接，不硬编码相机型号，因此可配置 D405、D415 等不同型号。所有相机使用同一组 `width`、`height`、`fps`；该 profile 必须被对应型号支持，否则 SDK 会在启动时直接报告错误。采集阶段不做 RGB-Depth 对齐，也不保存或处理相机外参。

启动 Robot 前确认双臂无接触且工作区安全。`leader_tool` 和 `follower_tool` 必须是对应机械臂中已经存在的 Tool。运行后使用：

- `c`：开始一条记录。
- `v`：完成并保存当前记录。
- `q`：退出；正在采集的记录会标记为无效记录。

正常记录保存为 `record_*`；设备异常、中断或退出中的记录保存为 `invalid_record_*`。原因记录在 `metadata.json` 和 `validation.json`。

## 机械臂 Homing

Homing 入口是 `scripts/home_robot.py`，用于将单台 Flexiv 机械臂移动到指定的七关节初始姿态：

```bash
conda activate haptic_exo_env
cd ~/haptic_exo_teleop_ws/shihao/FlexivTDKCollect
python scripts/home_robot.py
```

机械臂序列号、目标关节角、最大关节速度和最大关节加速度统一在 `HomeArgs` 中修改。角度相关配置使用度、度每秒和度每二次方秒。执行前确认同一台机械臂没有被 TDK 或其他程序控制。

## 软件测试

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. \
  python -m unittest discover -s scripts/tests -v
```