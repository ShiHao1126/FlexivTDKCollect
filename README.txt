双臂 TDK + 双侧力记录：无相机本地重建版
日期：2026-11-26

目标与状态
以恢复的 10 月 26 日 project_20261026 文件为基础，恢复最小可用软件闭环：
TDK 两臂控制 -> 100 Hz 双侧状态采样 -> HDF5 -> 独立离线校验。
本项目是“基于恢复原代码的重建版”，不是已证明逐字恢复的完整原项目。
原始证据位于 ../TDK_recovered_20261126，未覆盖修改。
没有访问或修改 SSH 设备，也没有运行真机。假 SDK 测试不等于真机验证。

本地入口（PowerShell，工作目录为本项目）
  & .\.venv\Scripts\python.exe scripts/collect.py
默认 interface=terminal、execute=False：终端离线预览，不连接、不使能、不运动。
终端命令需回车：r 请求开始，再输入 RUN 确认；s 停止并保存；q 保存后退出。
SSH 挂断信号、EOF、Ctrl+C 请求停止并将中断片段标记失败，保留 partial。
网络断开到系统发出信号可能有延迟，软件清理不替代物理急停。
图形窗口使用 --interface gui；以下窗口按钮说明适用于 GUI 模式。
只检查配置并退出：python scripts/collect.py --check_only
  & .\.venv\Scripts\python.exe -m unittest discover -s scripts/tests -p 'test_*.py' -v
  & .\.venv\Scripts\python.exe scripts/validate_record.py --path <HDF5文件路径>
测试产物保留于 offline_validation/<运行标识>/，全部是假设备合成数据。
测试不会批量删除任何文件；正式保存只移除已验证且已创建最终硬链接的那个 partial 文件。

入口参数
scripts/collect.py 开头 Args 集中设置 config、output、interface、execute、check_only、reset_home。
窗口点击“开始采集”并确认后初始化并启用遥操作，无采集时长上限。
点击“停止并保存”或按 Esc：停止遥操作、校验数据、自动保存本段 HDF5。
关闭窗口同样请求停止并保存，清理完成后窗口才退出。可连续采集多段。
异常只保留 partial，不自动发布；窗口锁定故障并显示错误，详情在 session_errors.log。
execute=True 仅允许 Linux，并且每次开始都要求窗口确认；本轮仅测试假 SDK。
窗口使用 Tkinter（Python 的 Tk 组件）；Linux 运行需要图形桌面及对应环境的 Tk。
停止请求不能强行打断阻塞的原生 SDK 调用，不能代替物理急停。
reset_home=False 为新入口的安全默认；开启后才调用继承的双臂关节 Home。
底层 recorder 使用 stop_event 手动停止；原快捷键菜单未复原，由新窗口提供开始/停止操作。

配置与数据协议
config.tdk_force_only.yaml 原样复制。相机、腕相机、触觉与夹爪均禁止启用。
里面保留了原外围设备的关闭配置和注释，不表示重建版实现了这些设备。
leader=Rizon4R-062116；follower=Rizon4R-062115；接口地址192.168.97.10。
这些是继承的历史配置，不是本轮核实的当前设备身份/网络/标定。
反馈比例0.75、原接触力限制、关节Home、状态超时和速度限制保留。
本重建入口仅开放关节 Home，不开放恢复控制器中保留的 Cartesian Home 分支。

HDF5 format=flexiv_l515_raw（沿用旧协议名字，实际不包含图像）。
根目录只含 leader 和 robot；robot 始终是 follower，不能随意改为左右臂。
两侧各记录：host_time_ns、robot_time_ns、tcp_pose_wxyz、tcp_vel、q、dq、
ext_wrench_in_tcp、ext_wrench_in_tcp_raw、ft_sensor_raw。
位姿平移m、关节rad、速度对应每秒；wrench前3力N、后3力矩Nm；四元数wxyz。
ext_wrench_in_tcp* 保留各自 TCP 坐标，不自动做跨臂坐标变换、符号翻转或重力补偿。
ft_sensor_raw 原样保存 SDK 传感器输出；其坐标定义必须以原 SDK/工具标定为准。
host_time_ns 是本机单调时钟，不是Unix时间；robot_time_ns 按 SDK 秒/纳秒拼接。
保留两台机器人各自时钟，不声称机器人时钟同步。
同轮读取组成一个原子队列项；max_age_ms 在此检查两侧主机读取时间差，
并不代表端到端通信延迟测量。机器人重复/倒退/停更由恢复控制器检查。
禁止空记录、非有限力值、非递增时间、两侧数量不等、队列溢出、异常统计。

依赖与限制
已建 .venv，复用本机基础 Python 科学计算包；requirements.txt 是独立重建依赖列表。
离线实际版本和测试结果见 OFFLINE_TEST_REPORT.txt。
原 Linux Python3.10 + flexivrdk1.9.2 + flexivtdk1.6.2 未在本机安装/验证。
假设备覆盖 API 调用顺序和文件契约，无法证明真实 SDK 参数、线程阻塞行为、
传感器零偏/标定、控制稳定性、实际100Hz时序、急停或力反馈方向。
未重建相机/触觉/夹爪、原完整快捷键菜单、数据集训练格式转换和原环境安装脚本。
若以后部署，应先单独确认设备序列号、工具负载与归零条件；本轮未部署。

溯源
见 PROVENANCE.json：原文件/当前文件哈希、复用与修改分类。
旧 GitHub 仓库仅是历史参考；本轮未用旧仓库的单臂实现替换已恢复 TDK 控制器。
