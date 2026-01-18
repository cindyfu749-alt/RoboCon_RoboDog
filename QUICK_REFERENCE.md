# ? 快速参考卡 - 代码优化速查表

## ? 关键文件位置

| 功能 | 文件 | 行数 |
|------|------|------|
| **主程序** | `User/main_optimized.c` | 120 |
| **控制逻辑** | `User/control_task/control_task.c` | 200 |
| **控制接口** | `User/control_task/control_task.h` | 80 |
| **PID 参数** | `User/bsp_ma_su/motor_pid_params.h` | 120 |
| **数据结构** | `User/system_data.h` | 100 |
| **中断处理** | `User/stm32f4xx_it_optimized.c` | 50 |

---

## ? 立即可做的事

### 1. 修改 PID 参数

```c
// 文件: motor_pid_params.h

const fp32 MOTOR_SPEED_PID_PARAMS[10][3] = {
    // ...
    {0.01f, 0.0006f, 0.015f},   // 电机2 - 改这里!
    // ...
};
```

### 2. 获取电机控制数据

```c
// 使用方式
MOTOR_send *ctrl = system_data_get_motor_ctrl(2);  // 电机2
ctrl->mode = 1;
ctrl->Tau = 100;

// 或直接访问
g_motor_ctrl[2].mode = 1;
```

### 3. 获取电机反馈数据

```c
// 使用方式
MOTOR_recv *fdbk = system_data_get_motor_fdbk(2);  // 电机2
float pos = fdbk->Pos;  // 获取位置

// 或直接访问
float pos = g_motor_fdbk[2].Pos;
```

### 4. 添加新的步态模式

```c
// 文件: control_task.c

static void _calculate_leg_trajectory(int leg_idx) {
    switch(leg_idx) {
        case 0:
            if(state->step == 2 || state->step == 3) {
                // 现有逻辑
            }
            if(state->step == 10) {  // 新模式
                // 新逻辑
            }
            break;
        // ...
    }
}
```

### 5. 在中断中添加代码

```c
// 文件: stm32f4xx_it_optimized.c

void GENERAL_TIM_IRQHandler(void) {
    control_task_timer_handler();
    // 不要在这里添加复杂代码!
    // (保持 < 0.5ms)
    TIM_ClearITPendingBit(GENERAL_TIM, TIM_IT_Update);
}
```

---

## ? 查找东西

| 要找... | 查看... |
|--------|--------|
| **PID 参数** | `motor_pid_params.h` |
| **电机数据** | `system_data.h` (g_motor_ctrl, g_motor_fdbk) |
| **步态计算** | `control_task.c` (_calculate_leg_trajectory) |
| **中断逻辑** | `control_task.c` (control_task_timer_handler) |
| **主循环** | `control_task.c` (control_task_execute) |
| **控制状态** | `control_task.h` (dog_state_t) |

---

## ? 代码片段

### 获取狗的当前状态

```c
dog_state_t *state = control_task_get_state();
printf("Step: %d, Rate: %d, Time: %d\n", 
       state->step, state->step_rate, state->time_currently);
```

### 启动/停止电机

```c
control_task_set_running(1);  // 启动
control_task_set_running(0);  // 停止
```

### 设置步频

```c
control_task_set_step_rate(150);  // 低速
control_task_set_step_rate(200);  // 高速
```

### 访问足端轨迹

```c
foot_pos_t *foot = &g_foot_track[0];  // 腿1
printf("Foot X: %f, Y: %f\n", foot->x, foot->y);
```

### 访问关节角度

```c
float *angles = &g_joint_angles[0];  // 腿1
printf("Motor2: %f°, Motor3: %f°\n", angles[0], angles[1]);
```

### 调试打印

```c
state->print_flag = 1;         // 设置标志
control_task_print_debug();    // 打印调试信息
```

---

## ?? 配置

### 启动延迟 (startup_wait)

```c
// main_optimized.c
#define STARTUP_WAIT_COUNT 100000  // 改这个值
```

### 主循环延迟 (loop delay)

```c
// main_optimized.c
#define MAIN_LOOP_DELAY 5000  // 改这个值
```

### 步频范围

```c
// control_task.c
if (rate >= 150 && rate <= 250) {  // 支持的范围
    g_dog_state.step_rate = rate;
}
```

---

## ? 数组索引速查

### 电机数组索引

```
g_motor_ctrl[1]   → 电机1
g_motor_ctrl[2]   → 电机2
g_motor_ctrl[3]   → 电机3
...
g_motor_ctrl[9]   → 电机9
注意: 0 未使用
```

### 足端轨迹数组索引

```
g_foot_track[0]  → 腿1 (X, Y)
g_foot_track[1]  → 腿2 (X, Y)
g_foot_track[2]  → 腿3 (X, Y)
g_foot_track[3]  → 腿4 (X, Y)
```

### 关节角度数组索引

```
g_joint_angles[0,1]  → 腿1 (电机2,3)
g_joint_angles[2,3]  → 腿2 (电机5,4)
g_joint_angles[4,5]  → 腿3 (电机6,7)
g_joint_angles[6,7]  → 腿4 (电机9,8)
```

### PID 参数数组索引

```
MOTOR_SPEED_PID_PARAMS[0]  → 未使用
MOTOR_SPEED_PID_PARAMS[1]  → 电机1
...
MOTOR_SPEED_PID_PARAMS[9]  → 电机9
```

---

## ? 常见操作

### 修改电机1的速度Kp

```c
// motor_pid_params.h
const fp32 MOTOR_SPEED_PID_PARAMS[10][3] = {
    {0.0f,   0.0f,    0.0f},      // [0] 未使用
    {0.009f, 0.0006f, 0.05f},     // [1] 电机1 - 改第一个值
    // ...
};
```

### 修改电机所有位置Kp

```c
// motor_pid_params.h
const fp32 MOTOR_ANGLE_PID_PARAMS[10][3] = {
    {0.0f,   0.0f,  0.0f},        // [0] 未使用
    {100.0f, 0.1f,  0.0f},        // [1] - 改这里
    {40.90f, 0.0f,  40.0f},       // [2] - 和这里
    // ... 所有电机
};
```

### 添加调试日志

```c
// 在 control_task.c 中
void control_task_execute(void) {
    // ...
    printf("Motor %d: %f\n", motor_id, position);  // 添加日志
    // ...
}
```

### 测量中断时间

```c
// 在中断开始/结束设置 GPIO
void GENERAL_TIM_IRQHandler(void) {
    GPIO_SetBits(GPIOA, GPIO_Pin_1);         // 开始
    control_task_timer_handler();
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);       // 结束 (用示波器测量)
    TIM_ClearITPendingBit(GENERAL_TIM, TIM_IT_Update);
}
```

---

## ?? 常见错误

| 错误 | 症状 | 修正 |
|------|------|------|
| 数组越界 | 电机不动/崩溃 | 检查索引 (1-9) |
| 忘记初始化 | 全是 0 或乱值 | 调用 `system_data_init()` |
| extern 遗漏 | 编译错误 | 包含 `system_data.h` |
| PID 参数错误 | 电机跳动/无控制 | 检查 `motor_pid_params.h` |
| 中断时间长 | 系统不稳定 | 删除 printf/浮点 |

---

## ? 文档导航

| 想了解... | 查看... |
|----------|--------|
| **完整优化方案** | `OPTIMIZATION_GUIDE.md` |
| **迁移步骤** | `MIGRATION_CHECKLIST.md` |
| **架构对比** | `ARCHITECTURE_COMPARISON.md` |
| **优化总结** | `OPTIMIZATION_SUMMARY.md` |
| **快速参考** | 本文件 |

---

## ? 学习路径

### 新手 (1 天)
1. 阅读 `OPTIMIZATION_SUMMARY.md`
2. 查看 `main_optimized.c`
3. 查看 `control_task.h`

### 中级 (2-3 天)
1. 阅读 `OPTIMIZATION_GUIDE.md`
2. 研究 `control_task.c` 实现
3. 查看 `motor_pid_params.h` 参数

### 高级 (1 周)
1. 完整阅读所有优化文档
2. 进行代码迁移 (按 `MIGRATION_CHECKLIST.md`)
3. 进行功能和性能测试

---

## ? 调试技巧

### 问题: 电机不动

**检查清单**:
```
1. ? 电机参数是否正确?     → 查看 motor_pid_params.h
2. ? 控制数据是否设置?     → 查看 g_motor_ctrl[motor_id]
3. ? 发送是否成功?         → 查看 Motor_date_send()
4. ? 中断是否运行?         → 添加 GPIO toggle
5. ? 接收是否有反馈?       → 查看 g_motor_fdbk[motor_id]
```

### 问题: 步态不对

**检查清单**:
```
1. ? 足端轨迹是否正确?     → 打印 g_foot_track[leg_id]
2. ? 逆运动学是否正确?     → 打印 g_joint_angles[]
3. ? PID 输出是否正确?     → 打印 g_motor_speed_pid_out[]
4. ? 步频是否正确?         → 查看 state->step_rate
5. ? 时间计数是否正常?     → 查看 state->time_currently
```

### 问题: 中断时间长

**检查**:
```c
// 在 control_task_timer_handler() 中:
// ? 不要有这些:
printf(...)              // 串口 I/O 很慢!
float x = sqrt(y);       // 浮点运算很慢!
for(int i=0; i<1000; i++)  // 循环很慢!

// ? 只做这些:
state->time_currently++;   // 整数加法 ~1 个周期
if(state->time_currently >= state->step_rate) {  // 整数比较
    state->step = 0;       // 赋值
}
```

---

## ? 性能基准

**正常情况**:
```
中断时间:        < 0.5 ms  ?
主循环时间:      < 10 ms   ?
电机响应延迟:    < 50 ms   ?
步态更新频率:    1000 Hz   ?
```

**异常情况**:
```
中断时间:        > 1 ms    ? 检查代码
主循环时间:      > 50 ms   ? 添加延迟
电机无反应:      常见      ? 检查参数
崩溃:            罕见      ? 检查数组索引
```

---

**最后提示**: 遇到问题时先查看 `OPTIMIZATION_GUIDE.md`, 再看对应的源代码! ?
