# VS Code 图形操作

## 打开与编译

1. 从包根双击 `02-Open-VSCode.cmd` 直接进入已配置好的独立窗口，或在 VS Code 用文件 → 打开文件夹选包根下的 `projects/F401_COURSE`。建议单独一个窗口，防止生成或调试仍指向旧 MOTOR_BASIC。
2. 左侧 CMake 视图选择此文件夹，配置选 Debug；将鼠标移到“生成”行，点生成图标，或点底部状态栏“生成”。首次需完成配置后再生成；`ninja: no work to do` 说明当前产物已经最新。
3. 输出下拉选择“CMake/生成”，确认退出0，以及输出目录确实是 `F401_COURSE/build/Debug`。Release同理切换预设，不修改其他工程。
4. 不需要关闭 Codex/Claude 扩展。未做全局禁用，clangd使用本工程真实编译数据库。

## 不接硬件先认识界面

菜单“终端 → 运行任务”可选 `Course: 离线演示`、`Course: 上位机`、`Course: 软件测试（不连接硬件）`。也可资源管理器双击 PC 目录里的两个 `.cmd`。演示画面为生成数据，按键不会连接硬件。

## 后续硬件阶段才执行

左侧“运行和调试”，下拉选 `STM32Cube: Course ST-Link`；绿色三角形会触发构建、连接ST-LINK、下载及在main停住，属于硬件动作。确认电机电源断开/安全缺省有效后再做。配置固定 Debug ELF，因此切Release后若要调试Release须明确改对应路径；不默默下载另一种构建。

在“STM32Cube 实时监视”添加：

```text
g_course.mode
g_course.armed
g_course.fault
g_course.imu.euler_deg[0]
g_course.imu.euler_deg[1]
g_course.imu.euler_deg[2]
g_course.speed[0].hz
g_course.speed[1].hz
g_course.pwm[0]
g_course.pwm[1]
g_course.pose.x
g_course.pose.y
g_course.pose.valid
g_course.mission.state
g_course.mission.balance.stable
g_runtime_max_us
g_control_stack_free_words
g_free_heap_bytes
```

Live Watch采样频率与控制环不同，不显示每个10ms周期。查看变量不必暂停；移动时暂停 CPU 会让控制任务失去调度，且PWM可能继续。电机实测先使用独立断电通路，再使用断点。

`tasks.c` 是 FreeRTOS 调度器。业务断点优先放 `App/Src/app.c` 的 `App_Step` 或 `BSP/Src/runtime.c` 的 `Runtime_Tick`；关电机电源后再进行单步。当前工程没有可再生成的IOC，别用“重新生成代码”覆盖手工BSP。
