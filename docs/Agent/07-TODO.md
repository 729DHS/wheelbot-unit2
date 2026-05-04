# 待办清单 — Unit2

## Unit2 定位
关节锁定 + 轮毂电机 PID 平衡小车

## 已完成

- [x] 从 chassis_base 导入 DM4310/M3508/IMU/SBUS 驱动 (2026-05-04)
- [x] 文档同步自 Unit1 (2026-05-04)

## 待处理

- [ ] **main.c 整合**: 集成关节锁定 + IMU + 轮毂PID平衡 + SBUS
- [ ] **关节锁定**: bringup 四关节 → 捕获位置 → 高刚度 MIT 位置锁定
- [ ] **IMU 姿态读取**: BMI088 SPI 驱动验证
- [ ] **轮毂平衡**: 轮毂电机 PID 直立控制
- [ ] **SBUS 遥控**: 前进/后退/转向
- [ ] **整机调参**: PID 参数整定

## 参考

- Unit1 feature/spring-damper-sync: DM4310 驱动和弹簧阻尼实现
- docs/Agent/08-弹簧阻尼极坐标发现.md: 极坐标回弹分析
