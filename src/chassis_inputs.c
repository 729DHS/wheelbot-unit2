#include "chassis_inputs.h"

#include "chassis_base.h"
#include "dm4310_motor.h"
#include "rm_imu.h"
#include "rm_m3508.h"
#include "rm_sbus.h"

#define CONTROL_DT_S 0.001f
#define SBUS_RAW_MIN 240
#define SBUS_RAW_MAX 1807
#define SBUS_CH0_ZERO_RAW 1024
#define SBUS_CH1_ZERO_RAW 1027
#define SBUS_RAW_DEADBAND 8
#define RAD_TO_DEG 57.295779513f

static int32_t sbus_raw_to_target(uint16_t raw, uint16_t zero_raw)
{
	int32_t delta = (int32_t)raw - (int32_t)zero_raw;
	int32_t span;

	if (delta > -SBUS_RAW_DEADBAND && delta < SBUS_RAW_DEADBAND) {
		return 0;
	}

	if (delta >= 0) {
		span = SBUS_RAW_MAX - (int32_t)zero_raw;
	} else {
		span = (int32_t)zero_raw - SBUS_RAW_MIN;
	}

	if (span <= 0) {
		return 0;
	}

	return (delta * 1684) / span;
}

void chassis_build_balance_input(struct balance_ctrl_input *input, uint32_t now)
{
	input->now_ms = now;
	input->dt_s = CONTROL_DT_S;
	input->enabled =
		(g_chassis_sm.allow_output != 0U && g_chassis_sm.chassis_control_active != 0U) ? 1U : 0U;
	input->position_hold_enabled = input->enabled;
	input->rc_forward = sbus_raw_to_target(g_sbus_snapshot.raw[1], SBUS_CH1_ZERO_RAW);
	input->rc_turn = sbus_raw_to_target(g_sbus_snapshot.raw[0], SBUS_CH0_ZERO_RAW);
	if (g_chassis_debug_override.magic == 0x44424731U &&
	    g_chassis_debug_override.force_rc != 0U) {
		input->rc_forward = g_chassis_debug_override.rc_forward;
		input->rc_turn = g_chassis_debug_override.rc_turn;
	}
	input->pitch_deg = g_rm_imu.pitch;
	input->pitch_rate_dps = g_rm_imu.gyro[1] * RAD_TO_DEG;

	for (int i = 0; i < BALANCE_CTRL_WHEEL_COUNT; i++) {
		input->wheel_online[i] = g_rm_m3508.motor[i].online;
		input->wheel_angle_raw[i] = g_rm_m3508.motor[i].angle_raw;
		input->wheel_speed_rpm[i] = g_rm_m3508.motor[i].speed_rpm;
	}
}

void chassis_build_sm_input(struct chassis_sm_input *input, uint32_t now)
{
	input->now_ms = now;
	input->joint_online_mask = g_dm4310.online_mask;
	input->sbus_connected = g_sbus_snapshot.connected;
	input->fault = 0U;
	input->ch5 = g_sbus_snapshot.channel[4];
	input->ch6 = g_sbus_snapshot.channel[5];
	input->ch8 = g_sbus_snapshot.channel[7];
	if (g_chassis_debug_override.magic == 0x44424731U) {
		if (g_chassis_debug_override.force_sbus_connected != 0U) {
			input->sbus_connected = g_chassis_debug_override.force_sbus_connected;
		}
		if (g_chassis_debug_override.force_ch5 != 0U) {
			input->ch5 = g_chassis_debug_override.force_ch5;
		}
		if (g_chassis_debug_override.force_ch6 != 0U) {
			input->ch6 = g_chassis_debug_override.force_ch6;
		}
		if (g_chassis_debug_override.force_ch8 != 0U) {
			input->ch8 = g_chassis_debug_override.force_ch8;
		}
		if (g_chassis_debug_override.force_sbus_connected != 0U &&
		    g_chassis_debug_override.force_ch5 == 2U) {
			input->joint_online_mask = CHASSIS_SM_JOINT_REQUIRED_MASK;
		}
	}

	for (int i = 0; i < CHASSIS_SM_JOINT_COUNT; i++) {
		input->joint_position[i] = g_dm4310.motor[i].pos_rad;
	}
}
