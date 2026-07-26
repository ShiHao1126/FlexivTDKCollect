# FlexivTDKCollect

基于 Flexiv TDK LAN 透明遥操作的数据采集工具。TDK 负责臂对臂遥操作，采集线程只读取 follower `RobotStates`、RealSense 图像以及 Xense 夹爪和触觉数据，不向设备发送额外控制命令。

## 运行

```bash
./run_collect.sh --leader-sn <LEADER_SERIAL> --output records
```

`--leader-sn` 必填。`--lan-ip` 是允许 TDK 使用的本机网卡 IPv4 地址，可重复传入；它不是机器人 IP。未传入时由 TDK 自动选择可用网卡。

启动会调用 TDK `Init()` 和 `Start()`。`Init()` 可能使能机械臂并执行 FT 传感器置零，因此必须确认机械臂无接触且工作区安全。进入操作界面后：

- `c`：创建 `.inprogress/record_*` 并开始一条记录。
- `v`：停止 worker、写完队列并将记录原子完成为 `record_*`。
- `q`：退出；若正在采集，该记录会保存为 `invalid_record_*`。

## Enable 参数

所有 enable 参数都显式接受 `true` 或 `false`。例如：

```bash
./run_collect.sh \
  --leader-sn <LEADER_SERIAL> \
  --enable-camera-color true \
  --enable-camera-depth false \
  --enable-gripper-width true \
  --enable-tactile-marker false \
  --enable-tactile-force true \
  --enable-tactile-rectify false \
  --enable-tactile-difference false \
  --enable-tactile-depth false
```

机器人数据按任务用途分组：

- `--enable-robot-joint-state`：`q`、`dq`
- `--enable-robot-torque`：`tau`、`tau_ext`、`tau_interact`
- `--enable-robot-tcp`：`tcp_pose`、`tcp_vel`
- `--enable-robot-wrench-tcp`：TCP 坐标系外力 `ext_wrench_in_tcp`
- `--enable-robot-wrench-world`：world 坐标系外力 `ext_wrench_in_world`

raw wrench、关节温度、flange pose 及重复的 motor-side 状态不采集。

## RealSense

通过 `--camera-serial` 选择相机，而不是按 D415、D435、L515 等型号硬编码。参数可重复传入以同时采集多个相机，每个相机由独立 worker 持有并保存到 `camera/<serial>/`。color 和 depth 可分别设置 enable、宽、高和 FPS：

```bash
./run_collect.sh \
  --leader-sn <LEADER_SERIAL> \
  --camera-serial <SERIAL_1> --camera-serial <SERIAL_2> \
  --camera-color-width 1280 --camera-color-height 720 --camera-color-fps 30 \
  --camera-depth-width 640 --camera-depth-height 480 --camera-depth-fps 30 \
  --align-depth-to-color true
```

所选分辨率和 FPS 必须是该相机实际支持的 profile。只有 color 和 depth 同时启用时，`--align-depth-to-color true` 才生效。

## 输出与状态

正常记录保存为 `record_*`；设备异常、中断或退出中的记录保存为 `invalid_record_*`。原因写入 `metadata.json` 和 `validation.json`。所有数据源保存相对单调时钟 `timestamp_s`，TDK、RealSense 和 Xense 同时保留各自 SDK 时间戳。

图像通过有界后台队列写盘。磁盘跟不上时丢弃整组图像并增加 `dropped_frames`，不会阻塞设备读取；写入失败会增加 `write_errors` 并使记录无效。
