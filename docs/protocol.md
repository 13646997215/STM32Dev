# PC / 蓝牙通信协议 v1

2026-10-30 软异常版本：遥测 offset 84 的质量字 bits 8..16 为锁存告警：FG 异常、超时、IMU 不可用、周期异常、边界、倾角、UART、位置失效、换向重试；上位机在“查看诊断详情”解释。这些位记录曾发生，不能都当成当前故障；新一轮启动或 CLEAR 后清除。FG 异常及位置失效时 bit0 为 0，位置不可信。原文中自动模式 FG 单次异常直接报 FEEDBACK 的描述不适用于本版本。

所有数值均显式 little-endian；float 为 IEEE754 float32，禁止NaN/Inf运动目标。GY953自身协议是另一种 big-endian 原始格式，不可混用。

| offset | 长度 | 含义 |
|---|---|---|
| 0 | 2 | A7 C3 |
| 2 | 1 | 版本01 |
| 3 | 1 | type |
| 4 | 2 | payload长度，0..96 |
| 6 | 2 | sequence |
| 8 | LEN | payload |
| 8+LEN | 2 | CRC16/CCITT-FALSE，覆盖前面所有字节，低字节先发 |

CRC初值FFFF，多项式1021，无反射，无xorout；`123456789`→29B1。总长LEN+10，100ms半帧超时，错误移一字节重新找头。UART与蓝牙透传均为115200/8N1。没有逐帧固定尾标，长度与CRC构成完整性边界。

## 命令

| type | 名称 | payload | 约束 |
|---|---|---|---|
| 01 | PING | 空 | ACK；版本由遥测schema和固件COURSE_VERSION标识 |
| 02 | STATUS | 空 | ACK；状态持续通过遥测回传 |
| 03 | CONFIG | u16 key + float value | 仅解除ARM时；成功修改普通参数会清空标定有效位 |
| 04 | ARM | DE C0 | 需有效参数及CAL_POWER、CAL_SIGNS；只允许控制，不直接运动 |
| 05 | VELOCITY | float左RPM + float右RPM | 非AUTO，需CAL_MANUAL，500ms过期停车 |
| 06 | START | u8 profile=1..7 | 仅已ARM的DIAGNOSTIC；需IMU新鲜及相应标定；重复START序号忽略 |
| 07 | STOP | 空 | 立即解除ARM/归零；幂等；不要求额外确认 |
| 08 | CLEAR | 空 | 解除ARM时清除软件故障，不复位MCU、不清除物理原因 |
| 09 | IMU_QUERY | u8 0=Acc、1=Mag、2=Euler | 单次读取：A5 C5 6A / A5 E5 8A / A5 95 3A；不会切换连续输出 |
| 0A | DUTY | float左 + float右，-1..1 | 正负表示方向；绝对值不超过max_duty；受控换向、500ms过期 |
| 0B | TASK1_BENCH_START | ASCII T1LV，即54 31 4C 56 | 显式启动Pitch调平台架mode4；flags不变；诊断态、静止且IMU新鲜；同启动序号不重启；拒绝旧T1BN |
| 0C | BENCH_KEEPALIVE | u16台架启动帧sequence | 仅续命已运行且序号匹配的台架；不能启动/重启或续命正式AUTO |

ARM不是校准完成证据；profile需要各自标志。手动速度进入AUTO前应 STOP→ARM→START。CONFIG不是Flash持久化，也没有设备参数批量读回接口；JSON保存的是输入记录，不能当成设备读回证据。成功ACK与telemetry中的cal_flags用于核对接受状态。

2026-10-19起，固件每20ms自主发送一次Euler单次查询A5 95 3A，不依赖PC连接、ARM或按钮。不会补发积压的历史周期；TX队列满时记录失败，下个周期再尝试，不阻塞控制。新帧仍必须通过原校验与时效检查，发送成功不能刷新数据年龄。额外的手动Euler读取成功后，从该时刻重新计算下个20ms周期。

兼容说明：09的帧格式及selector保持不变，但旧preflight固件执行的是非幂等连续输出开关15/35/45；新版执行C5/E5/95单次读取。须同步使用当前固件和当前上位机，新界面按钮是“读取一次”。历史发布包中的固件/界面不包含本次修复。ACK OK只说明控制器接受并排队转发，不能证明传感器响应。细节与诊断变量见[IMU采集说明](imu-acquisition.md)。

回复80 ACK /81 ERROR：相同sequence，payload `[原命令, result, fault]`。result：0 OK、1 FORMAT、2 RANGE、3 LOCKED、4 NOT_READY、5 DUPLICATE、6 FAULT。ACK也可能因TX队列满而丢弃；PC超时显示，不自动重发会改变状态的命令。

AUTO自主执行，不需要PC心跳；手动目标由上位机每100ms续发，500ms未刷新停车。串口/DMA接收错误触发锁存故障；CONFIG、手动指令在AUTO期间被拒绝，STOP例外。

任务1台架mode4使用独立100ms续命，超过500ms未收到有效续命则TIMEOUT停止；单次最多60s。Pitch正前进/负后退/近零HOLD停车，Yaw仅观察；运动时相同40–45%占空比，换向前两轮停稳。HOLD不解除ARM，可再次倾斜；没有旧3°倾覆完成条件。故障仍停止并锁存，普通ARM/START标定门槛不变，pose_valid=0，flags不变。新旧启动magic不同，必须同步更新GUI和固件；旧版本互配应FORMAT拒绝，禁止自动重试。详细判据见[任务1台架](task1-bench.md)。

## 遥测90，固定88字节，20Hz

| offset | 类型 | 内容 |
|---|---|---|
| 0..7 | 8×u8 | schema1、mode、armed、fault、state、profile、motor_phaseL/R |
| 8 | u32 | MCU ms |
| 12 | u32 | cal_flags |
| 16 | u32 | Euler年龄ms；FFFFFFFF表示未收到 |
| 20 | u32 | IMU校验通过总帧数（不能代替Euler序号） |
| 24/28 | u32 | FG累计接受上升沿L/R |
| 32/36 | float | FG Hz L/R，非有符号转速 |
| 40/44 | float | PWM L/R绝对占空比 |
| 48/52 | float | 归零后的板倾角degree / 航向rad |
| 56/60 | float | 估计x/y，m |
| 64/68 | float | 最近手动目标；MANUAL_SPEED=RPM，MANUAL_DUTY=有符号占空比，AUTO勿当自动目标 |
| 72/76 | u32 | IMU坏帧数 / 被拒绝命令数 |
| 80 | u32 | HOLD扰动事件数；不是砝码数量测量 |
| 84 | u32 | 质量位：bit0内部pose有效，bit1/2方向假设L/R，bit3/4滑行或换向等待L/R |

质量位没有“独立实测方向”状态。假设方向不是地面真实方向；滑行/回滚请结合文档边界评估。

91原始IMU观测，66字节、10Hz：0..11 Euler三float；12..29 Acc/Gyr/Mag各三int16 LE原始值；30..45四元数四float；46/50/54/58为Acc/Gyr/Mag/Quat更新时间u32；62为Euler序号u32。没有新帧的数据保留旧值，应看时间戳，四元数0不代表传感器损坏。原始单位没有猜成g、dps或uT。

## GY953解析

`5A 5A type len payload sum8`，完整帧总字节和低8位（不含最后校验）应等于sum。15/25/35/45长度6；65长度8；75/85只作有界状态观测，不参与控制。Euler signed int16 BE/100；四元数/10000；50ms半帧超时。仅45帧更新姿态年龄。

已用用户实采帧 `5A 5A 45 06 FA 88 FB 81 DD E3 BD` 验证：Roll=-14°、Pitch=-11.51°、Yaw=-87.33°。这只验证解码，不是模块零偏/精度标定。
