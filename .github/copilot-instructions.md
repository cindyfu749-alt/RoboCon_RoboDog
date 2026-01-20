# Copilot ʹ��˵���������������Ŀ��

## ��Ŀ����
STM32F4 Ƕ��ʽ�������������ϵͳ��ʹ�� ST Standard Peripheral Library��Keil MDK (UV5)������/DMA ͨ�š�˫������ PID ���ơ����˶�ѧ��̬�㷨��

**CMake ����**: ��Ŀ���� CMakeLists.txt��֧���� VS Code �������� IntelliSense �ʹ��뵼����

## ���������

### ʹ�� Keil MDK���Ƽ����ڱ���/���أ�
- **IDE**: Keil MDK UV5 �� [project/RVMDK(V5)/MOD_F427IIH.uvprojx](project/RVMDK(V5)/MOD_F427IIH.uvprojx)
- **Ŀ��оƬ**: STM32F427IIH��Flash: 2MB, SRAM: 256KB��
- **����������**: Build �� ���� ST-Link ������ �� Download + Debug

### VS Code IntelliSense ���ã����ڴ�������͵�����
1. ��װ��չ: **C/C++ (Microsoft)**, **CMake**, **CMake Tools**
2. ����Ŀ��CMake ���Զ����ã��� `.vscode/settings.json` �� `cmake.configureOnOpen: true`��
3. ����ʱ���� CMake: `Ctrl+Shift+P` �� `CMake: Configure`
4. ����:
   - **Go to Definition**: `Ctrl+Click` �� `F12` ��ת������
   - **Go to References**: `Shift+F12` �鿴��������
   - **Find All Definitions**: `Ctrl+Shift+O` ��������

## ���ļܹ�

### ��ʼ��˳���� User/main.c �У�
1. **���� & DMA ����**: USART1 (RCң��) + USART6 (�������) + DMA ˫�������
# AI Coding Agent Guide — RoboCon_RoboDog

目标：让 AI 代码代理快速、安全地在本仓库中实现小改动（PID 调参、步态逻辑、DMA/CRC 修复）并遵循代码约定。

快速概览
- 主循环与控制流：`User/main.c` 为高层步骤与行为调度（RC 读取 → 步态/计时 → PID 计算 → 串口发送）。
- 实时/中断：1ms 定时器由 `User/tim/bsp_general_tim.c` 配置，ISR 在 `User/stm32f4xx_it.c` 的 `GENERAL_TIM_IRQHandler`（时间敏感，处理必须 <0.5ms）。
- 电机通信：电机报文结构在 [User/GO-M8010-6/GO-M8010-6.h](User/GO-M8010-6/GO-M8010-6.h)，发送前调用 `modify_data()`，接收后用 `extract_data()` 做 CRC 校验（实现见 `User/GO-M8010-6/GO-M8010-6.c`）。
- PID 实现：项目使用位置/增量 PID（`User/pid/pid.c`），电机 PID 数组在 `User/main.c` 定义并由 `User/bsp_ma_su/bsp_ma_su.c` 的 `PID_Init()` 初始化。

重要约定（务必遵守）
- Motor 索引是 1-based：PID 数组与 `MOTOR_send`/`MOTOR_recv` 使用 `1..Mo_Count`（示例在 `User/main.c`、`User/bsp_ma_su/bsp_ma_su.c`）。
- 所有电机数据通过 `MOTOR_send`/`MOTOR_recv` 结构循环发送/接收（`GO-M8010-6.h`）。修改控制命令后调用 `modify_data(&motorX_control_data)`。
- USART DMA 均使用循环模式（circular DMA），接收中断解析后由 `extract_data()` 写入 `motor_feedback_data`，中断处理后按 `motor_id` 分发（见 `User/stm32f4xx_it.c` 的 `Motor_DMA_IRQHandler`）。
- 时序/优先级：不要在 `GENERAL_TIM_IRQHandler` 做耗时或阻塞调用；NVIC 优先级约定为 TIM > USART6_DMA > RC_DMA（在 `bsp_general_tim.c`/`stm32f4xx_it.c` 可见）。

构建与调试
- Keil MDK UV5 工程：`project/RVMDK(V5)/MOD_F427IIH.uvprojx`，常用用于下载 + ST-Link 调试。
- VSCode/CMake：仓库包含 `CMakeLists.txt`；在 VSCode 中使用 `C/C++`, `CMake`, `CMake Tools` 扩展并运行 `CMake: Configure`（`.vscode/settings.json` 推荐 `cmake.configureOnOpen: true`）。
- 日志/调试输出：使用串口 printf 实现（`User/usart/bsp_debug_usart.h`），在调试前启用 `printf_flag` 避免频繁输出卡中断。

常见改动示例（直接可用）
- 调整某电机 PID：编辑 `User/main.c` 中对应的 `const fp32 MotorN_speed_PID_data[3]`，然后确认 `PID_Init()` 在 `User/bsp_ma_su/bsp_ma_su.c` 使用该数组进行初始化。
- 增加/修改步态：修改 `foot_track()` 调用位置在 `User/main.c` 的 `if(step)` 分支，步态计算结果通过 `counter_motion()` 转换为电机目标角度。
- 修复电机 CRC/帧问题：检查 `User/GO-M8010-6/GO-M8010-6.c` 中 `modify_data()`/`extract_data()` 的实现，并通过 `bsp_usart_dma.c` 的 DMA 配置验证 `DMA_BufferSize` 与实际帧长一致。

查阅入口（优先级按重要性排序）
- 主循环：[User/main.c](User/main.c)
- PID 核心：[User/pid/pid.c](User/pid/pid.c)
- PID/电机初始化与更新：[User/bsp_ma_su/bsp_ma_su.c](User/bsp_ma_su/bsp_ma_su.c)
- 定时器设置（1ms）：[User/tim/bsp_general_tim.c](User/tim/bsp_general_tim.c)
- 中断与 ISR：[User/stm32f4xx_it.c](User/stm32f4xx_it.c)
- 电机协议/结构：[User/GO-M8010-6/GO-M8010-6.h](User/GO-M8010-6/GO-M8010-6.h)
- USART/DMA 设置：[User/bsp_usart_dma/bsp_usart_dma.c](User/bsp_usart_dma/bsp_usart_dma.c)

操作注意事项（安全与验证）
- 修改 ISR、NVIC 或 DMA 时，先在本地通过静态阅读并小范围修改，避免增加阻塞或改变优先级顺序。
- 更改 PID 常量后线下先用低频率打印验证（`printf_flag`）再放入高速循环。
- 与电机通信相关改动必须同时验证 `modify_data()`/`extract_data()` 的 CRC 与 `SENDBUFF_SIZE`/报文对齐。

需要补充或不清楚的地方请告知：我会把该文件进一步细化为“任务模板”（例如：如何安全地调低某电机 Kp 并验证）。
