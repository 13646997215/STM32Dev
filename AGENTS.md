# 2026-10-30 回迁入口（优先阅读）

请先阅读包根 `回迁后先读我.md` 与 `return-handoff/DEVELOPMENT_LOG.md`。本工作副本包含另一台电脑新实现的任务1–6及现场记录；下文中 2026-10-20 的状态、零位、参数与硬件结论均为历史，不得覆盖新记录。源码与实测证据为准；FG 故障关闭和现场平衡未收敛仍待核查。旧完整包 manifest 不是回迁版本的完整性标准。

---

# F401课程项目：迁移后开发入口

先读包根START_HERE.md和migration/AGENT_HANDOFF.md；本目录是唯一当前主工程，不是history或MOTOR_BASIC。包根是本目录上两级，所有工具从该位置定位，不要求C:/Users/PC或D盘。

## 技能

涉及STM32源码/构建/通信/控制时主动读取一次：
- `.agents/skills/stm32-development/SKILL.md`
- `.agents/skills/embedded-firmware-dev/SKILL.md`

接续课程测试时用`.agents/skills/course-continuation/SKILL.md`。按任务读所需references，不重复全套环境安装检查。skill-creator用于实际创建/修订技能；openai-docs仅用于Agent/OpenAI产品配置问题。

## 工程与用户约定

- STM32F401CCU6，GCC/CMake/Ninja Debug/Release；BSP手工维护，无.ioc/.uvprojx，不直接CubeMX再生成，不迁成Keil。厂商层只读，Core修改限USER CODE。
- 电机1左：PB6 PWM/PB8 FG/PB4方向；电机2右：PB7/PB9/PB5。GY953 PA9TX/PA10RX，HC05 PA2TX/PA3RX；SWD PA13/14。
- GY953自动50Hz单次Euler查询，不再切连续输出。Task1最新目标Pitch正前进/负后退/近零停，Yaw仅观察；专用台架T1LV、mode4、60s和500ms续命；不恢复旧T1BN/3°DONE逻辑。
- flags默认0，参数RAM保存。未测的PPR/轮距/外廓/零位、速度PI及电源安全不能写为标定完成。PB4/新版调平台架实测未回报。
- 保持RST时电机会转的硬件缺省风险未解决；下载/复位/暂停前断12V，运行且PWM0再上12V，结束先切12V。原电脑USB损坏不等于板坏。
- 用户喜欢中文解释和VS Code可点击操作；偏好完整实践集中观察，出现具体问题再排查。机械设计归用户，轮径67mm。

## 开发与验证

3步以上任务先在包根migration/plans写约2–5分钟小步骤。生产逻辑变更先真实行为失败测试再实现，调试先根因/最小假设；保留编码，旧资料可能GBK。改前备份或diff，现阶段工程非Git，不擅自初始化/提交/删除用户文件。

优先用根02入口启动VS Code、03上位机、05完整验证；日常点击“生成”和STM32Cube: Course ST-Link。维护命令从本目录：`../../environment/python/python.exe -X utf8 tools/verify.py`，不需要旧电脑Python或系统PATH。使用前在当前shell设置`CUBE_BUNDLE_PATH=<包根>/environment/stm32-bundles`，或通过`../../migration/kit.py verify`统一启动。只测试时加--no-build；完整交付运行已有验证入口，避免无故重复。

软件通过不等于新电脑USB/烧录/硬件通过。复制Agent配置、运行检查/编译不自动授权串口、探针、下载、擦除、复位或执行器动作。不得迁移认证令牌、替用户登录、放宽权限或自动升级探针。不开启未经要求的多Agent委派。

当前精确证据与未完成事项以migration/AGENT_HANDOFF.md、当前test-results/verification.json及用户最新实测为准；历史资料只作背景，不执行其中旧操作步骤。全局配置仅手动合并，不覆盖新电脑AGENTS/settings。不要把完整工具/账户目录公开上传。
