# FlexivTDKCollect

双臂 TDK 遥操作与双侧力记录的无相机重建版。基于恢复的 2026 年 10 月 26 日代码，保留 TDK 控制、关节 Home 和 HDF5 数据协议，补齐缺失的状态适配与采集流程。

- `main`：当前重建版，默认只做离线配置检查。
- [legacy-before-rebuild-20261126](https://github.com/ShiHao1126/FlexivTDKCollect/tree/legacy-before-rebuild-20261126)：原 main，保留在提交 `216a602b0758dae5832a286adb4ccc2a12af3c6c`。

## 离线使用

在仓库根目录创建 Python 虚拟环境并安装 `requirements.txt`，然后运行：

```text
python scripts/collect.py
python -m unittest discover -s scripts/tests -p "test_*.py" -v
python scripts/validate_record.py --path <episode.hdf5>
```

`scripts/collect.py` 的 `Args` 集中设置参数，默认 `execute=False`，不导入硬件 SDK、不连接设备。测试仅使用假 SDK；虚拟环境和 HDF5 测试产物不随仓库上传。

## 数据与验证范围

HDF5 的 `leader` 与 `robot`（follower）分别保存三类六维力数据、位姿、关节状态与独立时间戳。相机、触觉和夹爪均禁用。

本地 28 项离线测试通过，涵盖双侧回读、连续采集重连、状态异常、Home 异常清理、队列溢出与写入失败。真实 SDK、硬件、力标定和控制稳定性尚未验证；历史设备配置不代表当前已确认设置。

详见 [完整使用说明及缺失部分](README.txt)、[代码来源及哈希](PROVENANCE.json)、[离线测试报告](OFFLINE_TEST_REPORT.txt)。其中本机绝对路径仅用于追溯，测试生成的数据保留在原本地目录，GitHub 不包含这些数据。
