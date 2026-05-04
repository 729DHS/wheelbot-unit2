# 待办清单

## 已完成

- [x] ENABLE/DISABLE 尾字节定义修正 (chassis_base 版本)
- [x] 反馈帧字节布局修正
- [x] Bringup 序列: MIT寄存器写入 → DISABLE → ZERO → ENABLE → DONE
- [x] `put_le_u32` / `dm4310_write_u32_register` 移出 #if 块, 修正 CAN 总线路由
- [x] RST 引脚连接 (上电复位问题)
- [x] bringup 可靠性: 四电机全部进入 MIT 模式

## 待处理

- [x] **验证 MIT 位置闭环**: 已实现自适应力矩位置闭环自动归位 (2026-05-04)
- [x] **调优 KP/KD 参数**: KP 自适应线性 ramp (30-200), KD 同比缩放 (2026-05-04)
- [ ] **整机断电重启验证**: 全流程稳定
