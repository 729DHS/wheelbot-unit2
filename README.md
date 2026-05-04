# Unit2: 关节锁定 + 轮毂PID平衡

## 定位

**锁定五连杆关节，PID 控制轮毂电机实现两轮自平衡小车。**

启动流程：上电 → DM4310 bringup → 自动锁定关节 → 自动启用手动平衡控制。
**无需遥控器即可自平衡。**

与 Unit1 的区别：
| | Unit1 | Unit2 |
|------|-------|-------|
| 关节电机 (DM4310) | 弹簧阻尼手感控制 | 锁定在固定位置 |
| 轮毂电机 (M3508) | 待开发 | PID 平衡控制 |
| 启动方式 | SBUS 遥控 | 上电自动启用 |
| 目标 | 五连杆运动学 + 同步 | 两轮自平衡小车 |

## 硬件平台
- **MCU**: STM32F407IG (DJI RoboMaster Type-C 板)
- **关节电机**: DM-J4310-2EC × 4 (CAN1: 左腿1,2 / CAN2: 右腿3,4)
- **轮毂电机**: M3508 × 2 (CAN1)
- **IMU**: BMI088 (SPI)
- **遥控**: SBUS (可选，上电自动启用平衡)
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
1. 上电 → DM4310 四关节 bringup (MIT 模式)
2. 自动捕获关节当前位置 → MIT 位置锁定 (高刚度 KP=90, KD=1.8)
3. 自动启用平衡控制 → IMU pitch 归零
4. BMI088 IMU 读取姿态角 (Mahony 姿态解算, 1kHz)
5. 级联 PID 控制轮毂电机实现直立平衡

### 平衡 PID 参数 (需要根据实际车体调参)
| 参数 | 默认值 | 说明 |
|------|--------|------|
| PITCH_KP | 90.0 | 角度环 P 增益 |
| PITCH_KD | 4.5 | 角度环 D 增益 (角速度阻尼) |
| SPEED_KP | 20.0 | 速度环 P 增益 |
| SPEED_KI | 0.05 | 速度环 I 增益 |
| WHEEL_SPEED_LIMIT | 3000 RPM | 目标轮速限幅 |
| WHEEL_CURRENT_LIMIT | 16384 | 输出电流限幅 |

### PID 调参指南
1. 先调 PITCH_KD (角速度阻尼): 增大直到小车能"站稳"不振荡
2. 再调 PITCH_KP (角度刚度): 增大直到小车能快速回正
3. 最后调 SPEED_KP/KI (轮速环): 抑制长周期漂移

参数可通过 OpenOCD 在线修改 `g_balance_ctrl` 进行实时调参。

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
    main.c                   # 主入口 (上电自动启用平衡)
    dm4310_motor.c/h         # DM4310 MIT 驱动 (关节锁定)
    rm_m3508.c/h             # M3508 轮毂电机驱动
    rm_imu.c/h               # BMI088 IMU 驱动 (Mahony AHRS)
    rm_sbus.c/h              # SBUS 遥控驱动
    balance_ctrl.c/h         # 平衡级联PID控制
    chassis_base.c/h         # 底盘初始化
    chassis_sm.c/h           # 状态机 (系统/关节/底盘)
    chassis_inputs.c/h       # 输入抽象层
    chassis_outputs.c/h      # 输出抽象层
    chassis_threads.c/h      # IMU + 控制线程
    chassis_telemetry.c/h    # UART 遥测输出
    lk_motor.c/h             # LK电机驱动 (保留)
    x_axis_motor_map.c/h     # 五连杆 x 轴标定
  CMakeLists.txt
  prj.conf
  docs/
```

## 遥测输出
UART6 (USART6, 115200 baud) 每 10ms 输出 CSV:
```
pitch, left_speed_target, pitch_p_out, pitch_d_out, left_speed_p, left_speed_i,
m3508_1_speed, m3508_2_speed, joint[0..3]_pos, ...
```

## 关键经验
1. DM4310 bringup 完整流程: MIT 寄存器写入 → DISABLE → ZERO → ENABLE，跳过 ZERO 会导致零点异常
2. ENABLE=0xFC, DISABLE=0xFD, ZERO=0xFE
3. 反馈帧解析以 C++ 参考实现和实际 CAN 抓包为准
4. CAN 滤波器: 接受全部帧比窄滤波更可靠
5. 浮点打印需开启 CONFIG_CBPRINTF_FP_SUPPORT
6. CMSIS-DAP Horco v0.2 固件有时需重插 USB
