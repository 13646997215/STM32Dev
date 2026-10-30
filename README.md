# F401_COURSE — 课程软件准备版

本工程把双电机、GY953、PC/蓝牙通信、轮速 PI、里程估计、上板、找倾覆、平衡、配重扰动、返 A 和矩形路径分支放在**同一个可构建固件**中。默认 `DIAGNOSTIC`、PWM=0、`cal_flags=0`；不会自动开始运动。

**这是软件预备交付，不是课程实物验收结论。** 本次没有打开 COM、连接探针、烧录、复位或驱动电机。任务 1–6 现场如何重新摆放、任务 5 返回后任务 6 又从平衡开始的矛盾，以及任务 7 的初始化许可，需要在正式验收前确认。当前提供各任务分支，尚未宣称完成一个无需场地复位的 1→6 自动串联赛程。

## 从哪里开始

2026-10-19任务1按用户最新目标改为Pitch正前进、负后退、近零停车，Yaw不控轮；与原PDF第1项“以失衡为准”的口径差异见[任务1流程](docs/task1-practical-run.md)。当前未装机的手动IMU联动使用[任务1台架](docs/task1-bench.md)：新版固件与GUI需同步更新，旧3°触发结束逻辑已移除，台架60s会话内可反复俯仰。

1. 从包根双击 **02-Open-VSCode.cmd** 打开已配置好的独立窗口（或文件 → 打开文件夹选包根下 `projects/F401_COURSE`），避免多根工作区选错目标。
2. 打开 [VS Code 操作指引](docs/vscode-guide.md)，用 CMake 侧栏的生成按钮或状态栏“生成”构建。运行与调试选择 `STM32Cube: Course ST-Link`。后续硬件阶段再点绿色三角形。
3. 双击 [离线演示](PC/离线演示.cmd) 可先认识上位机；双击 [启动上位机](PC/启动上位机.cmd) 进入真实串口界面。只有点击“连接”才打开端口。
4. 硬件准备时按 [接线契约](docs/hardware-contract.md)、[标定与验收](docs/bringup.md) 逐项进行；暂定轮距/外廓/PPR 必须实测，不能直接把标志设成 255 来跳过。

## 软件目录

| 路径 | 职责 |
|---|---|
| `App/Inc/course.h` | 模块 API、状态、单位契约的入口 |
| `App/Src/imu.c`, `wire.c` | GY953 流式解析、PC CRC16 协议 |
| `config.c`, `control.c`, `motion.c` | 参数、FG、PI、换向、差速、位置估计 |
| `balance.c`, `mission.c`, `app.c` | 平衡、任务分支、模式与故障仲裁 |
| `BSP/Src/board.c` | GPIO、TIM、双串口 RX DMA、TX 队列、FG 中断 |
| `BSP/Src/runtime.c` | 单一所有者的收包/控制/LED/遥测编排 |
| `PC` | Tk 上位机、协议模型、日志/回放/CSV |
| `tests`, `test-results` | 实际 C 行为测试、Python 测试、红/绿日志、构建记录 |
| `docs` | 规则覆盖、接口、验证边界、现场清单 |

## 可复现验证

日常不必输入命令：VS Code **终端 → 运行任务**，选择 `Course: 软件测试（不连接硬件）` 或 `Course: 离线演示`。任务配置已在 `.vscode/tasks.json`。

维护时可运行：

```powershell
python tools/verify.py           # C/Python 测试、GUI离线启动、Debug/Release、ELF、基线检查
python tools/verify.py --no-build # 不交叉构建，只做本地软件验证
```

项目专用宿主编译器为随包 `environment/tcc/win32/tcc.exe`（`COURSE_HOST_CC` 指向，随包平移）；ARM 编译器不用于执行 Windows 测试。工具没有修改全局 PATH。版本与已知兼容问题见 [工具说明](docs/toolchain.md)。

## 固件与约束

构建输出在 `build/Debug`、`build/Release`，均含 `F401_COURSE.elf/.bin/.hex/.map`。源码对应校验见 `test-results/verification.json` 与发布包 manifest。

本工程从已验证的 `F401_MOTOR_BASIC` 复制创建，原工程保持不变。Core 改动位于 USER CODE；厂商 Drivers/Middlewares 不变。没有编造 `.ioc`：当前 BSP 为手工维护，**不能对本目录直接 CubeMX 重新生成**；若要迁入 CubeMX，应另建工程并按接线契约迁移。

单路 FG 不提供独立方向；里程采用受控换向下的方向假设。IMU 不提供绝对平面位置，无法凭软件保证未知横向初始偏移、打滑或 40 mm 圈精度。详见 [能力与规则边界](docs/limitations.md)。
