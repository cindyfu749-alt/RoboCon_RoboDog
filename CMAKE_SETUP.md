# 四足机器狗 RoboCon 项目 - CMake 配置指南

## 快速开始

### 1. 安装必要的 VS Code 扩展

打开 VS Code 扩展市场，安装以下扩展：
- **C/C++** (Microsoft)
- **CMake** (Microsoft)
- **CMake Tools** (Microsoft)

或运行命令：
```bash
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools
code --install-extension twxs.cmake
```

### 2. 打开项目

```bash
cd e:\co_robocon\RoboCon-Dog\RoboCon_RoboDog
code .
```

### 3. 配置 CMake

打开命令面板 (`Ctrl+Shift+P`)，运行：
```
CMake: Configure
```

如果提示选择生成器，选择 **Ninja** 或 **Unix Makefiles**。

### 4. 启用 IntelliSense

编辑 `.vscode/c_cpp_properties.json` 中的 `compilerPath`，改为你的编译器路径：

**Windows 示例（使用 GCC ARM）**:
```json
"compilerPath": "C:/Program Files (x86)/GNU Arm Embedded Toolchain/arm-none-eabi-gcc.exe"
```

**或使用 Clang**:
```json
"compilerPath": "C:/Program Files/LLVM/bin/clang.exe"
```

## 代码导航功能

### Go to Definition
在任意符号上按 `F12` 或 `Ctrl+Click`，跳转到定义。

**示例**：点击 `motor2_feedback_data` → 跳转到 [User/main.c](User/main.c#L20) 的定义。

### Go to References
选中符号，按 `Shift+F12`，查看所有引用位置。

**示例**：选中 `Motor_pid_count` → 显示在 main.c、bsp_ma_su.c 中的所有调用。

### Find All Definitions
按 `Ctrl+Shift+O` 打开当前文件的符号列表，或 `Ctrl+T` 全局搜索符号。

### 代码补全
在任意地方输入 `motor` 或其他前缀，`Ctrl+Space` 触发自动补全。

## 项目结构概览

```
.
├── CMakeLists.txt                    # CMake 配置（支持 IntelliSense）
├── .vscode/
│   ├── settings.json                 # CMake 和 C/C++ 设置
│   ├── c_cpp_properties.json         # IntelliSense 配置
│   └── launch.json                   # 调试配置
├── User/
│   ├── main.c                        # 主程序入口
│   ├── stm32f4xx_it.c                # 中断处理程序
│   ├── stm32f4xx_conf.h              # 芯片配置
│   ├── bsp_ma_su/                    # 步态和电机控制
│   ├── pid/                          # PID 控制实现
│   ├── GO-M8010-6/                   # 电机驱动协议
│   ├── bsp_rc/                       # 遥控接收
│   ├── bsp_imu/                      # IMU 传感器
│   ├── usart/                        # 串口通信
│   ├── tim/                          # 定时器配置
│   └── ...
├── Libraries/
│   ├── CMSIS/                        # ARM CMSIS 库
│   └── STM32F4xx_StdPeriph_Driver/   # STM32 标准库
└── project/
    └── RVMDK(V5)/                    # Keil MDK 项目文件
```

## 常见问题

### Q: IntelliSense 不工作
**A**: 
1. 确认已安装 C/C++ 扩展
2. 运行 `CMake: Configure`
3. 检查 `.vscode/c_cpp_properties.json` 中的 `compilerPath` 是否正确
4. 重启 VS Code

### Q: 跳转到定义不工作
**A**: 
1. 该符号可能是外部库定义，检查 `includePath` 是否包含该路径
2. 编辑 `CMakeLists.txt`，添加缺失的 `include_directories()`
3. 重新运行 `CMake: Configure`

### Q: 如何配置 CMake 支持编译？
**A**: 当前 `CMakeLists.txt` 仅用于 IntelliSense。若需实际编译，需补充：
- ARM Cortex-M4 工具链配置
- 链接脚本 (`.ld` 文件)
- 完整的编译选项

详见 [User/.github/copilot-instructions.md](.github/copilot-instructions.md) 的 Keil MDK 编译说明。

## 编辑 CMakeLists.txt

如需添加新的源文件或 include 路径：

```cmake
# 添加新的 include 路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/User/new_module
)

# 或在 file(GLOB_RECURSE) 中自动扫描
file(GLOB_RECURSE NEW_SOURCES
    "User/new_module/*.c"
)
```

然后运行 `CMake: Configure` 重新配置。

## 推荐工作流

1. **代码浏览**: 在 VS Code 中使用 IntelliSense 和跳转功能浏览代码
2. **编辑修改**: 在 VS Code 中编辑代码
3. **编译下载**: 在 Keil MDK 中 Build 和 Download 到硬件
4. **调试**: 在 Keil MDK 的调试器中调试

这样结合了两个工具的优势：VS Code 的代码导航 + Keil MDK 的编译/调试。

## 参考资源

- [CMake 官方文档](https://cmake.org/documentation/)
- [VS Code C/C++ 扩展](https://code.visualstudio.com/docs/cpp/c-cpp-overview)
- [STM32F4 参考手册](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
