/**
 * @file chassis_sm.h
 * @brief 底盘状态机共享常量和结构体定义
 */

#ifndef CHASSIS_SM_H_
#define CHASSIS_SM_H_

#include <stdint.h>

#include <zephyr/sys/util.h>

/** @brief 关节电机数量 (4 台 DM4310: 左髋/左膝 + 右髋/右膝) */
#define CHASSIS_SM_JOINT_COUNT 4
#define CHASSIS_SM_JOINT_REQUIRED_MASK 0x0FU

#define CHASSIS_SM_MAGIC 0x4348334DU

/* 系统状态 */
#define CHASSIS_SM_SYSTEM_DISABLED 0U
#define CHASSIS_SM_SYSTEM_ENABLED  1U
#define CHASSIS_SM_SYSTEM_ESTOP    2U

/* 关节状态 */
#define CHASSIS_SM_JOINT_RELAX 0U
#define CHASSIS_SM_JOINT_HOLD  1U

/* 底盘控制状态 */
#define CHASSIS_SM_CHASSIS_OFF 0U
#define CHASSIS_SM_CHASSIS_RUN 1U

/* 事件位 */
#define CHASSIS_SM_EVENT_SYSTEM_ENABLED  BIT(0)
#define CHASSIS_SM_EVENT_SYSTEM_ESTOP    BIT(1)
#define CHASSIS_SM_EVENT_JOINT_CAPTURED  BIT(2)
#define CHASSIS_SM_EVENT_JOINT_HOLD      BIT(3)
#define CHASSIS_SM_EVENT_JOINT_RELAX     BIT(4)
#define CHASSIS_SM_EVENT_CHASSIS_RUN     BIT(5)
#define CHASSIS_SM_EVENT_CHASSIS_OFF     BIT(6)

struct chassis_sm {
	uint32_t magic;
	uint32_t update_count;
	uint32_t last_ms;
	uint32_t events;
	uint8_t sbus_connected;
	uint8_t fault;
	uint8_t ch5;
	uint8_t ch6;
	uint8_t ch8;
	uint8_t ch5_on;
	uint8_t ch6_on;
	uint8_t ch8_on;
	uint8_t ch5_prev_on;
	uint8_t ch6_prev_on;
	uint8_t ch8_prev_on;
	uint8_t system_state;
	uint8_t joint_state;
	uint8_t chassis_state;
	uint8_t allow_output;
	uint8_t estop_active;
	uint8_t joint_hold_active;
	uint8_t chassis_control_active;
	uint8_t reserved[2];
	float joint_hold_ref[CHASSIS_SM_JOINT_COUNT];
	uint32_t joint_hold_valid_mask;
};

struct chassis_sm_input {
	uint32_t now_ms;
	uint32_t joint_online_mask;
	uint8_t sbus_connected;
	uint8_t fault;
	uint8_t ch5;
	uint8_t ch6;
	uint8_t ch8;
	uint8_t reserved[3];
	float joint_position[CHASSIS_SM_JOINT_COUNT];
};

extern volatile struct chassis_sm g_chassis_sm;

void chassis_sm_init(volatile struct chassis_sm *sm);
void chassis_sm_update(volatile struct chassis_sm *sm, const struct chassis_sm_input *input);

#endif /* CHASSIS_SM_H_ */
