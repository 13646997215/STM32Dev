# 硬件、单位与并发契约

## 板卡与供电

目标为 STM32F401CCU6，`STM32F401xC`、`startup_stm32f401ccux.s`、256 KB Flash / 64 KB RAM linker 一致。HSE=25 MHz，PLL M25/N160/P2，SYSCLK=80 MHz；APB1=40 MHz、APB2=80 MHz，TIM2/4=80 MHz。HAL 时间基准 TIM1，FreeRTOS tick=1 kHz SysTick。

电机红/黑线由独立 12 V 供电，控制板独立供电，12 V 负极与控制板 GND 共地。ST-LINK 不承担电机供电。**12 V 不接 STM32 信号针脚**。蓝牙模块 VCC=5 V 是用户已确认的该板供电条件，不代表其 RX 信号允许 5 V。GY953 手册供电范围 3–5 V；逻辑电平须按实际模块确认。

## 固定引脚

用户已明确指定：电机1为左轮（index0），电机2为右轮（index1）。这确定通道对应，不等于已完成装车前进电平方向标定。

| 信号 | STM32 | 软件设置 |
|---|---|---|
| M1 蓝：速度 | PB6 / TIM4_CH1 | AF2、PWM1、PSC39 ARR999，2 kHz |
| M2 蓝：速度 | PB7 / TIM4_CH2 | 同上 |
| M1 黄：FG | PB8 / EXTI8 | 上升沿、上拉；100 us 去毛刺 |
| M2 黄：FG | PB9 / EXTI9 | 同上 |
| M1 白：方向 | PB4 | Open-drain，正向电平由参数 102 定义 |
| M2 白：方向 | PB5 | Open-drain，正向电平由参数 103 定义 |
| GY953 TX → MCU RX | PA10 / USART1_RX | AF7，115200 8N1 |
| MCU TX → GY953 RX | PA9 / USART1_TX | AF7，同上 |
| CH340/HC05 TX → MCU RX | PA3 / USART2_RX | AF7，115200 8N1 |
| MCU TX → CH340/HC05 RX | PA2 / USART2_TX | AF7，同上 |
| LED | PC13 | 低有效 |
| KEY | PA0 | 上拉，低有效，连续三个10ms周期按下停车 |
| ST-LINK | PA13 SWDIO / PA14 SWCLK / GND | 保留 SWD，不占用 |

USART2 一次选择 CH340 或 HC05，不能将两个 TX 输出直接并联到 PA3。HC05 的工作态串口波特率须与固件一致，AT 模式波特率不等于工作态波特率。第二块控制板/蓝牙/TTL 保留备用，不虚构第二个 IMU 或分布式控制需求。

## DMA / IRQ / 时间

USART1 RX：DMA2 Stream2 Channel4；USART2 RX：DMA1 Stream5 Channel4。各1024字节 circular buffer，TC 计数，任务以 NDTR+wrap 计算生产位置，超容量增错误计数。DMA 优先级 IRQ7；USART IRQ7（仅错误与 TXE 出队），EXTI IRQ6；均不调用 RTOS API。串口 TX 各512字节队列，整帧入队，满则拒绝，不阻塞控制。

TIM2 为1MHz/32bit自由计数器，FG 周期/时间差使用无符号回绕运算。FG快照与串口队列临界区保存/恢复 PRIMASK；DMA 写内存与CPU交接含 DMB；本 MCU 无 D-cache。

`CourseTask` 每10ms `vTaskDelayUntil`，独占 `g_course`、参数、解析器与PWM输出。每周期每串口最多256字节，正常持续115200流量约115字节/10ms；必须实测 CPU 预算及干扰下的DMA溢出行为。遥测20Hz，IMU原始观测10Hz；ACK可因TX拥塞丢失，PC超时报告，不重发 ARM/START。

控制 task 栈768 words=3072B，FreeRTOS heap15360B 已计入 ELF .bss，不重复相加。模板的三个任务启动后立即终止；其生成框架保留但不执行旧电机/按键逻辑。开启 stack overflow / malloc failed hook；发生故障将CCR归零后锁住CPU。不会自动复位进入可能悬空的速度输入状态。

## 单位与方向

| 项目 | 约定 |
|---|---|
| 内部位置 / 线速度 | m / m/s |
| 航向 / 角速度 | rad / rad/s；逆时针为正 |
| 板倾角、零偏、阈值 | degree；GY953 Euler signed BE int16 /100 |
| 电机目标 | 车体正向为正的 wheel RPM；反馈仅绝对 Hz/RPM |
| PI | 每轮一个速度 PI，固件是唯一控制器；PC只下发目标 |
| FG PPR | 每个输出轮转一周的已接受上升沿数，减速比只补偿一次 |
| 坐标 | +x沿板A→远端，+y向左；板面中线 y=0；入口位置必须声明 |
| 机器人外廓 | 以差速轮轴中心为参考的中心对称包络；非对称外形应填包住它的对称尺寸 |

`board_heading_rad` 是车头沿板 +x 时、乘 yaw_sign 后的 IMU 航向，不是启动时随意清零。指定轮偏移 `dock_x/y` 相对轮轴中心；默认0是**待测占位**，不会识别老师指定哪个轮。

## 停机限制

用户已验证蓝线悬空时电机会转。初始化先输出低再切PWM只能覆盖固件开始执行后的时段；断电、上电和复位前无法由软件保证。必须在最终硬件确认蓝线安全缺省电平及独立断开电机电源的通路，并实测后设置 CAL_POWER。调试器暂停不等于停车，PWM外设可能继续输出。当前没有独立安全控制器或经验证的硬件急停。
