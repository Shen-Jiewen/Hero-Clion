# DM_02_Hero

## 项目简介

本项目基于RoboMaster机甲大师比赛官方步兵机器人代码，DM02开发板开发，将开发平台从Keil迁移到Clion上，旨在通过更现代化的集成开发环境和更开放的开源编译工具链提高开发的效率。

## 项目结构

- `Core/`
  - 包含 STM32 系统初始化、启动代码、硬件抽象层 (HAL) 驱动和 FreeRTOS 配置文件。
  - 子目录：`Src/`（源文件）、`Inc/`（头文件）

- `Drivers/`
  - 包含 STM32H7 的硬件抽象层驱动文件（GPIO、UART、I2C、定时器等）。
  - 子目录：`CMSIS/`（Cortex-M7 核心相关文件）、`STM32H7xx_HAL_Driver/`（HAL 驱动文件）

- `Middlewares/`
  - 包含 FreeRTOS、USB 驱动和其他中间件文件。
  - 子目录：`FreeRTOS/`（FreeRTOS 配置与源文件）、`USB_Device/`（USB 设备驱动）

- `User/`
  - 包含用户层的源文件和头文件，通过三层抽象来实现对机器人控制代码和硬件设备的解耦
  - 子目录：
    - 应用层：`application/` 核心模块和主要任务
    - 板载层：`bsp/` 板级支持包
    - 组件层：`components/` 常见算法、设备和外设的组件库

- `User_Config/`
  - 用户自定义的配置文件，主要包括烧录接口文件的配置
  - 文件：`daplink.cfg`、`stlink.cfg`。

## 贡献

欢迎提交问题反馈或代码贡献，直接通过 pull request 提交。

## 开发者

- **Javen** 邮箱:`jiewenshen6741@gmail.com`
- **Syh** 邮箱:`1684441286@qq.com`
- **Du.X.Y** 邮箱：`3512683509@qq.com`

## 许可证

`MIT License`
