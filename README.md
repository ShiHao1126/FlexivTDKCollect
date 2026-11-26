# FlexivTDKCollect

双臂 TDK 遥操作与双侧力记录的无相机重建版。基于恢复的 2026 年 10 月 26 日代码，保留 TDK 控制、关节 Home 和 HDF5 数据协议，补齐缺失的状态适配与采集流程。

- `main`：当前重建版，默认打开离线预览界面。
- [legacy-before-rebuild-20261126](https://github.com/ShiHao1126/FlexivTDKCollect/tree/legacy-before-rebuild-20261126)：原 main，保留在提交 `216a602b0758dae5832a286adb4ccc2a12af3c6c`。

## 离线使用

在仓库根目录创建 Python 虚拟环境并安装 `requirements.txt`，然后运行：

```text
python scripts/collect.py
python scripts/collect.py --check_only
python -m unittest discover -s scripts/tests -p "test_*.py" -v
python scripts/validate_record.py --path <episode.hdf5>
```

`scripts/collect.py` 的 `Args` 集中设置参数，默认 `execute=False`，打开离线预览窗口、不连接设备。`--check_only` 仅检查配置并退出。

真机模式（Linux、`execute=True`）下，点击“开始采集”并确认后持续记录，直到点击“停止并保存”或按 Esc，没有定时结束。停止后校验并自动保存，可继续下一段；关闭窗口也会请求停止并保存。故障保护仍可自动中止，且禁止故障后直接重启。窗口显示时长、双侧样本数、错误或文件路径。需要 Tkinter 和图形桌面。测试仅使用假 SDK；虚拟环境和 HDF5 测试产物不随仓库上传。

## 数据与验证范围

HDF5 的 `leader` 与 `robot`（follower）分别保存三类六维力数据、位姿、关节状态与独立时间戳。相机、触觉和夹爪均禁用。

本地原有 28 项离线测试通过，新增手动生命周期与窗口按钮测试，涵盖双侧回读、连续采集重连、状态异常、Home 异常清理、队列溢出与写入失败。真实 SDK、硬件、力标定和控制稳定性尚未验证；历史设备配置不代表当前已确认设置。

详见 [完整使用说明及缺失部分](README.txt)、[代码来源及哈希](PROVENANCE.json)、[离线测试报告](OFFLINE_TEST_REPORT.txt)。其中本机绝对路径仅用于追溯，测试生成的数据保留在原本地目录，GitHub 不包含这些数据。
