# DM_02_Hero

## 项目简介

本项目基于RoboMaster机甲大师比赛官方步兵机器人代码，DM02开发板开发，将开发平台从Keil迁移到Clion上，旨在通过更现代化的集成开发环境和更开放的开源编译工具链提高开发的效率。

## 项目结构

### 1. 核心框架 (Core/)
- 系统初始化：`startup_stm32h723vgtx.s`（启动代码）
- 硬件抽象层：`Src/` 和 `Inc/` 目录下的 HAL 驱动文件
- FreeRTOS 配置：`FreeRTOSConfig.h`
- 中断处理：`stm32h7xx_it.c`
- 系统时钟：`system_stm32h7xx.c`

### 2. 驱动层 (Drivers/)
- **CMSIS/**: Cortex-M7 核心支持文件
  - 设备头文件：`stm32h7xx.h`
  - 系统配置文件：`system_stm32h7xx.c`
- **STM32H7xx_HAL_Driver/**: 硬件抽象层驱动
  - 外设驱动：GPIO、UART、I2C、SPI、TIM等
  - 低层驱动：LL库文件

### 3. 中间件 (Middlewares/)
- **FreeRTOS/**: 实时操作系统
  - 任务调度器
  - 内存管理
  - 任务间通信
- **USB_Device/**: USB设备库
  - CDC类设备驱动
  - USB描述符配置

### 4. 用户层 (User/)

#### 4.1 应用层 (application/)
- **模块管理 (module/)**
  - 底盘控制：
    - `chassis.c/h`: 底盘运动控制核心逻辑
    - `chassis_behaviour.c/h`: 底盘行为状态机
    - `chassis_power_control.c/h`: 底盘功率管理
  - 云台控制：
    - `gimbal.c/h`: 云台运动控制
    - `gimbal_behaviour.c/h`: 云台行为状态机
  - 射击控制：
    - `shoot.c/h`: 射击机构控制
  - 传感器：
    - `imu.c/h`: IMU数据处理
    - `detect.c/h`: 传感器数据检测
  - 裁判系统：
    - `referee.c/h`: 裁判系统通信
    - `protocol.h`: 通信协议定义
  - 用户界面：
    - `ui.c/h`: 用户界面显示
  - 系统时钟：
    - `tick.c/h`: 系统时间管理

- **任务调度 (task/)**
  - `chassis_task.c`: 底盘控制任务
  - `gimbal_task.c`: 云台控制任务
  - `shoot_task.c`: 射击控制任务
  - `imu_task.c`: IMU数据处理任务
  - `detect_task.c`: 传感器检测任务
  - `referee_task.c`: 裁判系统通信任务
  - `remote_control_task.c`: 遥控器数据处理任务
  - `buzzer_task.c`: 蜂鸣器控制任务
  - `led_task.c`: LED灯控制任务
  - `ui.c`: 用户界面更新任务
  - `usb_task.c`: USB通信任务
  - `watch_task.c`: 系统监控任务

#### 4.2 板级支持包 (bsp/)
- **DM02开发板驱动**
  - `bsp_adc.c/h`: ADC驱动
  - `bsp_can.c/h`: CAN总线驱动
  - `bsp_crc32.c/h`: CRC32校验
  - `bsp_gpio.c/h`: GPIO驱动
  - `bsp_pwm.c/h`: PWM输出
  - `bsp_tim.c/h`: 定时器配置
  - `bsp_uart.c/h`: 串口通信

#### 4.3 组件库 (components/)
- **算法模块 (algorithm/)**
  - AHRS姿态解算：
    - `kalman_filter.c/h`: 卡尔曼滤波
    - `MahonyAHRS.c/h`: Mahony姿态解算
    - `QuaternionEKF.c/h`: 四元数扩展卡尔曼滤波
  - 用户数学库：
    - `user_lib.c/h`: 常用数学函数

- **控制器 (controller/)**
  - PID控制器：
    - `pid.c/h`: PID控制算法实现

- **设备驱动 (device/)**
  - IMU传感器：
    - `BMI088driver.c/h`: BMI088陀螺仪驱动
    - `BMI088Middleware.c/h`: BMI088中间件
    - `ist8310driver.c/h`: IST8310磁力计驱动
  - 电机驱动：
    - `dji_3508.c/h`: DJI 3508电机驱动
    - `dji_6020.c/h`: DJI 6020电机驱动
    - `dm_4310.c/h`: DM4310电机驱动
  - 超级电容：
    - `supercap.c/h`: 超级电容管理
  - LED灯带：
    - `ws2812.c/h`: WS2812灯带控制

- **支持库 (support/)**
  - 数据结构：
    - `fifo.c/h`: FIFO队列实现
  - 通信协议：
    - `vofa.c/h`: VOFA+通信协议
  - CRC校验：
    - `CRC8_CRC16.c/h`: CRC8/CRC16校验
  - 类型定义：
    - `struct_typedef.h`: 常用数据结构定义

### 5. 配置文件 (User_Config/)
- 调试接口配置：
  - DAPLink：`daplink.cfg`
  - ST-Link：`stlink.cfg`
  - J-Link：`jlink.cfg`

## 开发环境配置

1. 安装 [CLion](https://www.jetbrains.com/clion/) IDE
2. 安装 [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) 用于生成初始化代码
3. 安装 [OpenOCD](https://openocd.org/) 用于调试和烧录
4. 安装 [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) 用于编译

## 编译和烧录

1. 打开CLion，导入项目
2. 配置工具链：
   - 设置工具链为 `GNU Arm Embedded Toolchain`
   - 配置 `CMake` 项目
   - 配置 `OpenOCD` 调试器，选择对应的配置文件
3. 点击 `Build` 编译项目
4. 点击 `Run` 烧录并运行程序

## 调试

- 使用`J-Link调试器` + `ozone调试`连接开发板
- 使用`虚拟串口`连接开发板，查看串口输出

## 项目依赖

- STM32CubeMX HAL库
- FreeRTOS 实时操作系统


## 贡献

欢迎提交问题反馈或代码贡献，直接通过 pull request 提交。

## 开发者

- **Javen** 邮箱:`jiewenshen6741@gmail.com`
- **Syh** 邮箱:`1684441286@qq.com`
- **Du.X.Y** 邮箱：`3512683509@qq.com`

## 许可证

`MIT License`
