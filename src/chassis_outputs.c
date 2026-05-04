#include "chassis_outputs.h"

#include "balance_ctrl.h"
#include "chassis_base.h"
#include "chassis_inputs.h"
#include "chassis_sm.h"
#include "dm4310_motor.h"
#include "rm_m3508.h"
#include "rm_sbus.h"

static uint8_t last_m3508_mode;
static uint8_t last_joint_output_active;
static uint8_t last_chassis_output_active;

uint8_t chassis_apply_joint_state(void)
{
	float hold_ref[CHASSIS_SM_JOINT_COUNT];

	if (g_chassis_sm.allow_output == 0U || g_chassis_sm.joint_hold_active == 0U) {
		if (last_joint_output_active != 0U ||
		    (g_chassis_sm.events &
		     (CHASSIS_SM_EVENT_SYSTEM_ESTOP | CHASSIS_SM_EVENT_JOINT_RELAX)) != 0U) {
			dm4310_stop_all();
			g_chassis_base.joint_hold_ret = 0;
			last_joint_output_active = 0U;
			return 1U;
		}
		g_chassis_base.joint_hold_ret = 0;
		last_joint_output_active = 0U;
		return 0U;
	}

	for (int i = 0; i < CHASSIS_SM_JOINT_COUNT; i++) {
		hold_ref[i] = g_chassis_sm.joint_hold_ref[i];
	}
	if (last_joint_output_active == 0U ||
	    (g_chassis_sm.events & CHASSIS_SM_EVENT_JOINT_HOLD) != 0U) {
		dm4310_hold_reset();
	}
	g_chassis_base.joint_hold_ret =
		dm4310_hold_positions(hold_ref);
	last_joint_output_active = 1U;
	return 1U;
}

void chassis_apply_m3508_command(void)
{
	if (g_chassis_debug_override.magic == 0x44424731U &&
	    g_chassis_debug_override.force_m3508_current != 0U) {
		/* Debug override: bypass SM gate, send current directly */
		last_chassis_output_active = 1U;
		g_chassis_base.m3508_apply_ret = rm_m3508_set_all_current(
			g_chassis_m3508_cmd.current[RM_M3508_CAN1_ID201],
			g_chassis_m3508_cmd.current[RM_M3508_CAN2_ID202]);
		if (g_chassis_base.m3508_apply_ret == 0) {
			g_chassis_base.m3508_apply_ret = rm_m3508_send_currents();
		}
		last_m3508_mode = CHASSIS_M3508_MODE_CURRENT;
		return;
	}

	if (g_chassis_debug_override.magic == 0x44424731U &&
	    g_chassis_debug_override.force_m3508_current == 0U) {
		/* Balance mode: skip SM gate, send PID output */
	}

	if (g_chassis_m3508_cmd.mode == CHASSIS_M3508_MODE_DISABLED) {
		if (last_chassis_output_active != 0U ||
		    last_m3508_mode != CHASSIS_M3508_MODE_DISABLED ||
		    (g_chassis_sm.events &
		     (CHASSIS_SM_EVENT_SYSTEM_ESTOP | CHASSIS_SM_EVENT_CHASSIS_OFF)) != 0U) {
			rm_m3508_stop();
		}
		g_chassis_base.m3508_apply_ret = 0;
		last_chassis_output_active = 0U;
		last_m3508_mode = CHASSIS_M3508_MODE_DISABLED;
		return;
	}

	last_chassis_output_active = 1U;

	if (g_chassis_m3508_cmd.mode == CHASSIS_M3508_MODE_DISABLED &&
	    last_m3508_mode != CHASSIS_M3508_MODE_DISABLED) {
		rm_m3508_stop();
		g_chassis_base.m3508_apply_ret = 0;
		last_m3508_mode = g_chassis_m3508_cmd.mode;
		return;
	}

	switch (g_chassis_m3508_cmd.mode) {
	case CHASSIS_M3508_MODE_CURRENT:
		g_chassis_base.m3508_apply_ret = rm_m3508_set_all_current(
			g_chassis_m3508_cmd.current[RM_M3508_CAN1_ID201],
			g_chassis_m3508_cmd.current[RM_M3508_CAN2_ID202]);
		if (g_chassis_base.m3508_apply_ret == 0) {
			g_chassis_base.m3508_apply_ret = rm_m3508_send_currents();
		}
		break;
	case CHASSIS_M3508_MODE_DISABLED:
	default:
		g_chassis_base.m3508_apply_ret = 0;
		break;
	}

	last_m3508_mode = g_chassis_m3508_cmd.mode;
}

void chassis_apply_balance_control(uint32_t now)
{
	struct balance_ctrl_input input;
	struct balance_ctrl_output output;

	chassis_build_balance_input(&input, now);
	balance_ctrl_update(&g_balance_ctrl, &input, &output);

	g_chassis_base.balance_updates++;
	g_chassis_base.balance_active = output.active;
	g_chassis_base.balance_left_current = output.wheel_current[0];
	g_chassis_base.balance_right_current = output.wheel_current[1];

	if (output.active != 0U) {
		g_chassis_m3508_cmd.mode = CHASSIS_M3508_MODE_CURRENT;
		g_chassis_m3508_cmd.current[RM_M3508_CAN1_ID201] = output.wheel_current[0];
		g_chassis_m3508_cmd.current[RM_M3508_CAN2_ID202] = output.wheel_current[1];
	}
}

void chassis_update_common_status(uint32_t now)
{
	g_chassis_base.last_ms = now;
	g_chassis_base.sbus_seq = g_sbus_snapshot.seq;
	g_chassis_base.sbus_connected = g_sbus_snapshot.connected;
	g_chassis_base.lk_online_mask = g_dm4310.online_mask;
	g_chassis_base.lk_encoder_valid_mask = g_dm4310.online_mask;
	g_chassis_base.m3508_online_mask =
		(g_rm_m3508.motor[RM_M3508_CAN1_ID201].online ? 0x01U : 0U) |
		(g_rm_m3508.motor[RM_M3508_CAN2_ID202].online ? 0x02U : 0U);
	g_chassis_base.sm_events = g_chassis_sm.events;
	g_chassis_base.sm_joint_hold_valid_mask = g_chassis_sm.joint_hold_valid_mask;
	g_chassis_base.sm_system_state = g_chassis_sm.system_state;
	g_chassis_base.sm_joint_state = g_chassis_sm.joint_state;
	g_chassis_base.sm_chassis_state = g_chassis_sm.chassis_state;
	g_chassis_base.sm_fault = g_chassis_sm.fault;
	g_chassis_base.sbus_ch5 = g_chassis_sm.ch5;
	g_chassis_base.sbus_ch6 = g_chassis_sm.ch6;
	g_chassis_base.sbus_ch8 = g_chassis_sm.ch8;
}
