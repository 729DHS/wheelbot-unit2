# Unit2: 关节锁定 + 轮毂PID平衡

## 定位

**锁定五连杆关节，仅控制轮毂电机实现自平衡小车。**

与 Unit1 的区别：
| | Unit1 | Unit2 |
|------|-------|-------|
| 关节电机 (DM4310) | 弹簧阻尼手感控制 | 锁定在固定位置 |
| 轮毂电机 (M3508) | 待开发 | PID 平衡控制 |
| 目标 | 五连杆运动学 + 同步 | 两轮自平衡小车 |

## 硬件平台
- **MCU**: STM32F407IG (DJI RoboMaster Type-C 板)
- **关节电机**: DM-J4310-2EC × 4 (CAN1: 左腿1,2 / CAN2: 右腿3,4)
- **轮毂电机**: M3508 × 2 (CAN1)
- **IMU**: BMI088 (SPI)
- **遥控**: SBUS
- **调试器**: CMSIS-DAP (OpenOCD)

## 电机映射
| 电机编号 | CAN 总线 | 功能 | 所属腿 |
|---------|---------|------|--------|
| 1 | CAN1 | 左关节1 | 左腿 |
| 2 | CAN1 | 左关节2 | 左腿 |
| 3 | CAN2 | 右关节1 | 右腿 |
| 4 | CAN2 | 右关节2 | 右腿 |
| M3508-1 | CAN1 | 左轮毂 | — |
| M3508-2 | CAN1 | 右轮毂 | — |

## 控制逻辑
1. 上电 bringup 四关节电机
2. 捕获关节当前位置 → MIT 位置锁定 (高刚度, KP≈90, KD≈1.8)
3. BMI088 IMU 读取姿态角
4. PID 控制轮毂电机实现直立平衡
5. SBUS 遥控前进/后退/转向

## 构建 & 烧录
```bash
export ZEPHYR_BASE=/home/huiming/zephyrproject/zephyr
cmake -B build -DBOARD=robomaster_c -DBOARD_ROOT=.
cmake --build build

openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 1000" \
  -c "program build/zephyr/zephyr.elf verify reset exit"
```

## 项目结构
```
Unit2/
  boards/dji/robomaster_c/   # 板级定义 (DTS, Kconfig)
  src/
    main.c                   # 主入口
    dm4310_motor.c/h         # DM4310 MIT 驱动 (关节锁定)
    rm_m3508.c/h             # M3508 轮毂电机驱动
    rm_imu.c/h               # BMI088 IMU 驱动
    rm_sbus.c/h              # SBUS 遥控驱动
    balance_ctrl.c/h         # 平衡PID控制
    chassis_base.c/h         # 底盘基础
    chassis_sm.c/h           # 状态机
    chassis_inputs/outputs   # 输入输出抽象
    lk_motor.c/h             # LK电机驱动
  CMakeLists.txt
  prj.conf
  docs/
```

## 关键经验 (来自 Unit1 开发)
1. 弹簧阻尼对电机角度回弹好, 但对五连杆末端极坐标(r,φ)不能保证回弹
2. 侧边检测: 滞回(ON=0.08/OFF=0.03rad) + 80ms去抖动避免机械耦合误触发
3. 浮点打印需开启 CONFIG_CBPRINTF_FP_SUPPORT
4. CMSIS-DAP Horco v0.2 固件有时需重插 USB
