# 参数与标定有效位

所有普通参数在解除ARM时修改，整项校验成功才提交；失败不会部分写入。任何key≠100的成功修改都会清空cal_flags，写完全部参数后才设置标志。重启恢复Config_Default，不自动恢复JSON。

本轮任务1与专用台架使用Task1_Update，倾角环是完整PID：key30零位（当前0）、101轴（1）、33符号（+1）、34比例增益、35积分增益、36微分增益、38停车容差（.5°）、39稳定角速度（1°/s）、41倾角保护（默认18°）、20速度上限。停止后重新运动门槛为2×key38。微分项限幅为比例项的一半，保证快速回位时只减不反转；积分项在进入容差时清零并做抗饱和。任务1不使用key37平衡反号或key42倾覆阈值，Yaw不生成差速。正式任务1零位稳定1s结束；台架持续监测60s；其他平衡任务仍用Balance_Update及10s判据。PC/config.task1-bench.json含11项，flags保持0，不是装机标定完成声明。

## float参数

| key | RobotConfig字段 | 合法下限 | 合法上限 |
|---:|---|---:|---:|
| 1 | `wheel_diameter_m` | .01f | .3f |
| 2 | `track_m` | .03f | .8f |
| 3 | `length_m` | .03f | 1 |
| 4 | `width_m` | .03f | .29f |
| 5 | `ppr[0]` | 1 | 100000 |
| 6 | `ppr[1]` | 1 | 100000 |
| 7 | `wheel_scale[0]` | .5f | 1.5f |
| 8 | `wheel_scale[1]` | .5f | 1.5f |
| 10 | `max_duty` | .05f | 1 |
| 11 | `deadband[0]` | 0 | .8f |
| 12 | `deadband[1]` | 0 | .8f |
| 13 | `kp[0]` | 0 | 1 |
| 14 | `ki[0]` | 0 | 10 |
| 15 | `kp[1]` | 0 | 1 |
| 16 | `ki[1]` | 0 | 10 |
| 17 | `max_rpm` | 1 | 300 |
| 18 | `rpm_slew` | 1 | 1000 |
| 20 | `max_v` | .01f | .5f |
| 21 | `max_w` | .05f | 4 |
| 22 | `min_v` | .001f | .2f |
| 23 | `heading_kp` | .1f | 10 |
| 30 | `pitch_zero_deg` | -90 | 90 |
| 31 | `board_heading_rad` | -COURSE_PI | COURSE_PI |
| 32 | `yaw_sign` | -1 | 1 |
| 33 | `pitch_sign` | -1 | 1 |
| 34 | `balance_kp` | 0 | 1 |
| 35 | `balance_ki` | 0 | 1 |
| 36 | `balance_kd` | 0 | 1 |
| 37 | `balance_sign` | -1 | 1 |
| 38 | `stable_deg` | .05f | 2 |
| 39 | `stable_rate_dps` | .05f | 10 |
| 40 | `disturb_deg` | .1f | 10 |
| 41 | `max_tilt_deg` | 5 | 40 |
| 42 | `tip_delta_deg` | .5f | 15 |
| 43 | `seek_min_m` | .02f | 1 |
| 44 | `seek_max_m` | .1f | 1.55f |
| 45 | `climb_end_m` | .05f | .5f |
| 46 | `start_x_m` | -1 | 0 |
| 47 | `start_y_m` | -.5f | .5f |
| 48 | `return_x_m` | -.15f | .2f |
| 50 | `path_a_m` | .1f | 3 |
| 51 | `path_b_m` | .1f | 3 |
| 52 | `dock_x_m` | -.4f | .4f |
| 53 | `dock_y_m` | -.4f | .4f |
| 54 | `pos_tol_m` | .001f | .05f |
| 55 | `yaw_tol_rad` | .005f | .2f |
| 56 | `boundary_margin_m` | 0 | .05f |

后缀f表示C float字面量；COURSE_PI=π。带_m为m，带rad为rad，pitch/angle阈值为degree，rpm为轮RPM；duty/deadband为0..1，PI量纲按输出占空比与RPM/秒定义。balance_kp/ki/kd以倾角degree及实际秒输出m/s。

## 整数参数（在线同样用float32表示，但要求整数）

| key | 含义 | 合法值 |
|---:|---|---|
| 100 | cal_flags | 0..255 |
| 101 | pitch_axis，0 Roll/1 Pitch | 0或1 |
| 102/103 | 左/右车体前进方向，1释放OD/0拉低 | 0或1 |
| 104 | 上板必须观察到坡面倾角 | 0或1 |
| 105 | Euler最大年龄ms | 50..1000 |
| 106 | FG最大年龄ms | 50..2000 |
| 107 | 换向静默ms | 100..1000 |

## cal_flags 位定义

| 值 | 名称 | 用户必须先完成的实测 |
|---:|---|---|
| 1 | GEOMETRY | 轮距、包络、轮偏移/场地尺寸 |
| 2 | SCALE | PPR、有效轮周与里程比例 |
| 4 | SIGNS | 两轮车体方向符号 |
| 8 | IMU | 安装轴、零偏、板航向、干扰检查 |
| 16 | SPEED | 正反轮速闭环与限速/反馈异常 |
| 32 | POWER | 电源、复位/掉电缺省与独立停止通路 |
| 64 | ENTRY | 合法起点、斜板识别与倾覆触发 |
| 128 | BALANCE | 正负反馈符号、端高差与10秒保持 |

ARM需36；手动轮速需54；任务1/7需63；任务2/预测评需127；任务3/4/5/6平衡相关分支需255。标志是用户对已完成实测的声明，软件无法从数字自动证明其真实性。

## 额外约束

yaw_sign、pitch_sign、balance_sign只能±1；deadband必须小于max_duty；min_v≤max_v；stable_deg<disturb_deg<max_tilt_deg；seek_min<seek_max。边界关系在每次写入后校验，因此批量调整时可能需先扩大上限再提高下限。GUI遇到拒绝会停止批量发送，不继续设置标定标志。

默认轮径0.067m来自用户；轮距0.140、长0.200、宽0.190m，PPR738、board_heading0、pitch_zero0、指定轮偏移0均未实测。默认max_duty0.55、deadband0.38、max_rpm40、max_v0.10、max_w0.7。不能把默认值当成整定结果。
