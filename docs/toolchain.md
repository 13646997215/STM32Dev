# 构建与测试工具

ARM GCC `14.3.1+st.2`、CMake `4.4.0+st.1`、Ninja `1.13.2+st.1`，来自本机 STM32Cube bundles。`tools/build.py` 核对这些文件，只给构建子进程增加 PATH；不会修改注册表、全局PATH或工具版本。

Python：随包 `environment/python/python.exe`（3.13，不依赖系统 Python 或注册表），已核对 Tkinter 与 pyserial 存在，没有为本项目额外安装 Python 包。迁移入口 `migration/kit.py` 会自动设置该解释器与 PATH 前缀。

用户批准项目专用 TinyCC。来源为 [TinyCC 官方站](https://bellard.org/tcc/) 指向的 Savannah 发布包：

| 包 | SHA256 |
|---|---|
| tcc-0.9.27-win64-bin.zip | 34a721949a2583fdff725312da092fa0f5f1f284b702e6f811c6954714faabb2 |
| tcc-0.9.27-win32-bin.zip | 02e2bfe8c272a549b15e4bfa4507bd7e05304692af1761db6c1e8e88af675651 |

64bit旧发行版在本机的 float 转换/传参测试出现错误，故正式行为测试固定使用随包 `environment/tcc/win32/tcc.exe`（由 `COURSE_HOST_CC` 指向，随包平移，不再是过去某个 D 盘绝对路径）。没有为了测试通过而修改固件算法。`test_toolchain.c` 检查浮点转换、函数传参与角度wrap基础行为，防止误用错误编译器。

宿主测试用 `tests/host/math.h` 把 float 数学接口接到 Windows CRT 的 double 运算再转float，绕开旧TinyCC头文件的x87内联汇编不兼容。ARM固件使用自己的newlib/libm。测试执行同一业务C源码，但这不等于两个平台具有逐bit相同的数学库和运行时行为；容差与实机数值仍需验证。

红色日志是缺行为 stub 或修改前的实际回归失败，绿色日志链接 App 当前源码；stub 均位于tests，不进入固件。`test_runtime.c` 用fake板接口测运行编排；硬件寄存器适配器只完成ARM编译/静态检查，DMA时序、IRQ负载、电平与实际输出没有经过硬件验证。
