# Unit2

## 项目简介

基于 Zephyr RTOS 的 STM32F407IG (DJI RoboMaster Type-C 板) 固件项目。

### 硬件平台
- **MCU**: STM32F407IG (DJI RoboMaster Type-C 板)
- **调试器**: CMSIS-DAP (OpenOCD)

### 构建 & 烧录
```bash
# 构建
export ZEPHYR_BASE=/home/huiming/zephyrproject/zephyr
cmake -B build -DBOARD=robomaster_c -DBOARD_ROOT=.
cmake --build build

# 烧录 (CMSIS-DAP)
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 1000" \
  -c "program build/zephyr/zephyr.elf verify reset exit"
```

### 项目结构
```
Unit2/
  boards/dji/robomaster_c/   # 板级定义 (DTS, Kconfig)
  src/
    main.c                   # 主程序入口
  CMakeLists.txt             # Zephyr 构建配置
  prj.conf                   # Kconfig
  docs/                      # 文档
```
