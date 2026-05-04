#include "chassis_threads.h"

#include "chassis_base.h"
#include "chassis_inputs.h"
#include "chassis_outputs.h"
#include "chassis_sm.h"
#include "chassis_telemetry.h"
#include "dm4310_motor.h"
#include "rm_imu.h"
#include "rm_m3508.h"
#include "x_axis_motor_map.h"

#include <zephyr/kernel.h>

#define IMU_THREAD_PERIOD_MS 2
#define CONTROL_THREAD_PERIOD_MS 1
#define IMU_THREAD_STACK_SIZE 2048
#define CONTROL_THREAD_STACK_SIZE 2048
#define IMU_THREAD_PRIORITY 2
#define CONTROL_THREAD_PRIORITY 3

static K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(control_thread_stack, CONTROL_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;
static struct k_thread control_thread_data;

static void chassis_update_x_axis_calibration(void)
{
	static uint32_t last_calib_seq;
	static uint32_t last_can1_home_save_seq;
	float motor_pos_rad[CHASSIS_SM_JOINT_COUNT];
	size_t count;

	if (g_chassis_debug_override.can1_home_save_seq != last_can1_home_save_seq) {
		last_can1_home_save_seq = g_chassis_debug_override.can1_home_save_seq;
		g_chassis_debug_override.can1_home_save_ret = dm4310_save_can1_home_current();
	}

	if (g_chassis_debug_override.x_axis_calib_seq == last_calib_seq) {
		return;
	}
	last_calib_seq = g_chassis_debug_override.x_axis_calib_seq;

	for (int motor = 0; motor < CHASSIS_SM_JOINT_COUNT; motor++) {
		motor_pos_rad[motor] = g_dm4310.motor[motor].pos_rad;
	}

	g_chassis_debug_override.x_axis_calib_ret =
		x_axis_motor_map_set_point(g_chassis_debug_override.x_axis_calib_x, motor_pos_rad);
	(void)x_axis_motor_map_points(&count);
	g_chassis_debug_override.x_axis_calib_count = count;
}

static void imu_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		const uint32_t now = k_uptime_get_32();

		if (g_chassis_base.imu_init_ret == 0) {
			g_chassis_base.imu_update_ret = rm_imu_update();
			g_chassis_base.imu_ready = g_chassis_base.imu_update_ret == 0 ? 1U : 0U;
		}
		g_chassis_base.imu_last_ms = now;
		g_chassis_base.imu_loops++;
		k_sleep(K_MSEC(IMU_THREAD_PERIOD_MS));
	}
}

static void control_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		const uint32_t now = k_uptime_get_32();
		struct chassis_sm_input sm_input = { 0 };

		if (g_chassis_base.lk_init_ret == 0) {
			dm4310_poll_rx();
			chassis_update_x_axis_calibration();
		}
		chassis_build_sm_input(&sm_input, now);
		chassis_sm_update(&g_chassis_sm, &sm_input);
		chassis_apply_joint_state();
		chassis_apply_balance_control(now);

		if (g_chassis_base.lk_init_ret == 0) {
			dm4310_tick();
		}

		if (g_chassis_base.m3508_init_ret == 0) {
			rm_m3508_poll();
			chassis_apply_m3508_command();
		}

		chassis_update_common_status(now);
		g_chassis_base.control_last_ms = now;
		g_chassis_base.control_loops++;
		k_sleep(K_MSEC(CONTROL_THREAD_PERIOD_MS));
	}
}

void chassis_threads_start(void)
{
	k_thread_create(&imu_thread_data, imu_thread_stack, K_THREAD_STACK_SIZEOF(imu_thread_stack),
			imu_thread, NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread_data, "imu");

	k_thread_create(&control_thread_data, control_thread_stack,
			K_THREAD_STACK_SIZEOF(control_thread_stack), control_thread, NULL, NULL,
			NULL, CONTROL_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&control_thread_data, "control");

	chassis_telemetry_start();
}
