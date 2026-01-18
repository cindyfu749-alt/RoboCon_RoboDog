cmake_minimum_required(VERSION 3.10)
project(RoboCon_RoboDog C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 设置芯片类型
set(CHIP STM32F427xx)

# 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

# 包含路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Libraries/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Libraries/STM32F4xx_StdPeriph_Driver/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/User
    ${CMAKE_CURRENT_SOURCE_DIR}/User/usart
    ${CMAKE_CURRENT_SOURCE_DIR}/User/tim
    ${CMAKE_CURRENT_SOURCE_DIR}/User/pid
    ${CMAKE_CURRENT_SOURCE_DIR}/User/GO-M8010-6
    ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp_ma_su
    ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp_rc
    ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp_usart_dma
    ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp_imu
    ${CMAKE_CURRENT_SOURCE_DIR}/User/can
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Clock
    ${CMAKE_CURRENT_SOURCE_DIR}/User/CRC
    ${CMAKE_CURRENT_SOURCE_DIR}/User/key
)

# 定义宏
add_definitions(-D${CHIP})
add_definitions(-DUSE_STDPERIPH_DRIVER)
add_definitions(-DSTM32F4xx)

# 收集源文件
file(GLOB_RECURSE USER_SOURCES
    "User/*.c"
)

file(GLOB_RECURSE STM_DRIVER_SOURCES
    "Libraries/STM32F4xx_StdPeriph_Driver/src/*.c"
)

# 排除特定文件（如果需要）
list(FILTER USER_SOURCES EXCLUDE REGEX ".*\\.c~$")

# 创建伪库（用于 IntelliSense，不实际编译二进制）
add_library(RoboCon_IntelliSense INTERFACE)

target_include_directories(RoboCon_IntelliSense INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Libraries/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Libraries/STM32F4xx_StdPeriph_Driver/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/User
    ${CMAKE_CURRENT_SOURCE_DIR}/User/usart
    ${CMAKE_CURRENT_SOURCE_DIR}/User/tim
    ${CMAKE_CURRENT_SOURCE_DIR}/User/pid
    ${CMAKE_CURRENT_SOURCE_DIR}/User/GO-M8010-6
    ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp_ma_su
    ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp_rc
    ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp_usart_dma
    ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp_imu
    ${CMAKE_CURRENT_SOURCE_DIR}/User/can
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Clock
    ${CMAKE_CURRENT_SOURCE_DIR}/User/CRC
    ${CMAKE_CURRENT_SOURCE_DIR}/User/key
)

target_compile_definitions(RoboCon_IntelliSense INTERFACE
    ${CHIP}
    USE_STDPERIPH_DRIVER
    STM32F4xx
)
