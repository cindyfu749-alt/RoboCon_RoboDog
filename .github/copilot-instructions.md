# Copilot 使用说明（项目定制）

目的：为 AI 编码助手快速上手本仓库提供可执行、面向文件的指引。只包含可观测到的模式与命令示例，不包含推测或通用建议。

- **工程类型**：基于 STM32F4 的嵌入式 C 项目，使用 ST 的 Standard Peripheral Library（见 `Libraries/STM32F4xx_StdPeriph_Driver`）。Keil MDK (UV4/UV5) 项目文件位于 `project/RVMDK(V5)/`（如 `MOD_F427IIH.uvprojx`）。

- **构建 / 调试**：首选在 Keil MDK 中打开 `project/RVMDK(V5)/MOD_F427IIH.uvprojx`，选择目标并 Build/Debug。项目包含 RVMDK/MDK 配置与 EventRecorderStub，调试以 MDK 为主。

- **运行时架构（大局）**：
  - `User/main.c` 启动顺序：串口（RC/Debug/Motor）初始化 → DMA 配置 → 系统时钟（`bsp_clkconfig.HSE_SetSysClock`）→ 定时器中断（1ms）→ 主循环处理 RC、步态、PID 调用与电机数据发送。
  - 控制循环依赖 TIM 中断（TIMx_Configuration）产生 1ms 周期；主循环避免阻塞性长延迟。

- **关键组件与位置**：
  - 电机与通信：`User/GO-M8010-6/`（驱动与数据封装）、`User/bsp_ma_su/`（步态、PID 初始化、motor 数据更新与发送）、USART 与 DMA 在 `User/usart/`。
  - 传感器：IMU 驱动在 `User/bsp_imu/`。
  - PID：`User/pid/` 包含 PID 实现及 `struct_typedef.h`（项目对 `fp32`、`Mo_Count` 等类型的定义）。
  - 硬件抽象：以 `bsp_` 前缀的文件组织（`bsp_clkconfig`, `bsp_can`, `bsp_debug_usart` 等），约定用于板级初始化。

- **跨组件通信与约定**：
  - 电机控制与反馈使用固定命名的全局结构体：`motorN_control_data` / `motorN_feedback_data`（N=1..9），以及 `MotorN_speed_PID` / `MotorN_rang_PID` 等 PID 实例。代码经常通过这些全局变量进行读写。
  - 电机命令通过 `USART6` 发送（在 `User/main.c` 与 `bsp_ma_su.c` 可看到 `Usart_SendStr_length(USART6, ...)`）。
  - RC 输入通过 `USART1`/DMA（`RC_USART1_Config()`、`USART1_DMA_Config()`）进入全局 `rc_rc` 结构。

- **约定与编码习惯（可直接引用示例）**：
  - 初始化顺序示例见：[User/main.c](User/main.c#L1-L80)（串口→DMA→时钟→定时器→PID 初始化）。
  - PID 配置與電機计數示例见：`User/bsp_ma_su/bsp_ma_su.c`（`PID_Init()`、`Motor_pid_count()`、`Motor_data_update()`）。
  - 项目使用大量全局变量共享状态（电机数据、步态时间、IMU 输出），AI 助手应优先在修改前搜索相关全局定义并保持原有并发语义。

- **注意点（避免常见错误）**：
  - 仔细保留实时约束：不要在中断或主控制路径中加入可能阻塞的 I/O/延时；代码中已有 `while` 用于上电等待，修改时应理解其目的。
  - 修改电机通信格式前，先检查 `GO-M8010-6` 驱动内 `modify_data()` / `extract_data()` 等函数，保证 CRC/长度字段与固件通讯一致。
  - 全局命名与索引模式并存（例如 `motor2_control_data` 而非数组）；在生成或重构代码时保留原有名字或提交兼容性替换。

- **代码搜索示例（推荐）**：当需要理解某一行为时，优先查看：
  - 控制入口：[User/main.c](User/main.c)
  - 步态与 PID：[User/bsp_ma_su/bsp_ma_su.c](User/bsp_ma_su/bsp_ma_su.c)
  - 电机封装：[User/GO-M8010-6/GO-M8010-6.c](User/GO-M8010-6/GO-M8010-6.c)
  - 库与外设：`Libraries/STM32F4xx_StdPeriph_Driver/` 及 `User/stm32f4xx_conf.h`

- **当你要做的修改 — 快速检查清单**：
 1. 确认变更是否影响中断或 1ms 控制环；若是，先评估时序并在 注释 中说明。
 2. 若改通信协议，验证 `modify_data()` / `extract_data()` 与 CRC 算法一致性。 
 3. 修改全局结构或 PID参数时，搜索全局引用并一次性更新初始化与使用点。

如果你希望我把这份说明进一步细化（添加更多文件链接或把若干函数注释提取为 TODO），告诉我想要的关注面（例如“构建/CI 自动化”或“把所有 motorN 转为数组”）。
