# FlexivTDKCollect

双臂 TDK 遥操作与双侧力记录的无相机重建版。基于恢复的 2026 年 10 月 26 日代码，保留 TDK 控制、关节 Home 和 HDF5 数据协议，补齐缺失的状态适配与采集流程。

- `main`：当前重建版，默认打开 SSH 终端交互界面（离线预览）。
- [legacy-before-rebuild-20261126](https://github.com/ShiHao1126/FlexivTDKCollect/tree/legacy-before-rebuild-20261126)：原 main，保留在提交 `216a602b0758dae5832a286adb4ccc2a12af3c6c`。

## 离线使用

在仓库根目录创建 Python 虚拟环境并安装 `requirements.txt`，然后运行：

```text
python scripts/collect.py
python scripts/collect.py --interface gui
python scripts/collect.py --check_only
python -m unittest discover -s scripts/tests -p "test_*.py" -v
python scripts/validate_record.py --path <episode.hdf5>
```

`scripts/collect.py` 的 `Args` 集中设置参数，默认 `execute=False`，打开终端离线预览、不连接设备。`--check_only` 仅检查配置并退出。

终端模式不需要 Tk 或 DISPLAY，命令均需回车，无二次确认输入：

| 命令 | 行为 |
| --- | --- |
| `h` | 主臂和从臂都回到各自配置的关节 Home；完成后保持停止 |
| `r` | 只开启双臂遥操作，不创建采集文件 |
| `c` | 在遥操作已开启时开始采集，无时长上限 |
| `s` | 只停止采集、校验并保存本段数据，遥操作继续开启 |
| `q` | 停止采集及遥操作，正常完成的片段保存后退出 |

推荐顺序：`h → 等待 ready → r → 等待 teleop → c → s`；下一段直接 `c`。采集中拒绝 `h`，先 `s` 并等待保存完成。再次 `h` 会先停止遥操作，再让双臂回 Home；不会自动重新开启遥操作。

状态持续显示时长与双侧样本数。未采集时仍检查遥操作故障和状态停更。输入关闭、SSH 挂断信号或 Ctrl+C 会请求停止全部，未完成片段标记为失败并保留，不自动发布。真实网络断线到操作系统发出挂断信号可能有延迟；软件清理不能代替急停。

使用 `--interface gui` 可打开图形窗口，分别提供“双臂 Home”“开启遥操作”“开始采集”“停止采集并保存”按钮；Esc 只停止采集，关闭窗口才停止全部。需要 Tkinter 和图形桌面。测试仅使用假 SDK；虚拟环境和 HDF5 测试产物不随仓库上传。

## 数据与验证范围

HDF5 的 `leader` 与 `robot`（follower）分别保存三类六维力数据、位姿、关节状态与独立时间戳。相机、触觉和夹爪均禁用。

本地原有 28 项离线测试通过，新增手动生命周期与窗口按钮测试，涵盖双侧回读、连续采集重连、状态异常、Home 异常清理、队列溢出与写入失败。真实 SDK、硬件、力标定和控制稳定性尚未验证；历史设备配置不代表当前已确认设置。

详见 [完整使用说明及缺失部分](README.txt)、[代码来源及哈希](PROVENANCE.json)、[离线测试报告](OFFLINE_TEST_REPORT.txt)。其中本机绝对路径仅用于追溯，测试生成的数据保留在原本地目录，GitHub 不包含这些数据。

## Pro6000_FRP

代码目录 `/home/kbe6000/shihao/FlexivTDKCollect`，离线检查使用已有 Python 环境：

```bash
cd /home/kbe6000/shihao/FlexivTDKCollect
/home/kbe6000/anaconda3/envs/flexiv_collect/bin/python scripts/collect.py
```

默认 execute=False，不连接真机；修改脚本 Args 后才允许真机启动。本次只同步代码及运行离线测试，未验证硬件 SDK。普通交互 SSH 即可使用；真机模式拒绝无 PTY 的重定向输入。
