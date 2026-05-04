#include "chassis_telemetry.h"

#include "balance_ctrl.h"
#include "chassis_base.h"
#include "dm4310_motor.h"
#include "rm_imu.h"
#include "rm_m3508.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define TELEMETRY_THREAD_PERIOD_MS 10
#define TELEMETRY_THREAD_STACK_SIZE 2048
#define TELEMETRY_THREAD_PRIORITY 6
#define TELEMETRY_BUF_SIZE 384

static K_THREAD_STACK_DEFINE(telemetry_thread_stack, TELEMETRY_THREAD_STACK_SIZE);
static struct k_thread telemetry_thread_data;
static const struct device *const telemetry_uart = DEVICE_DT_GET(DT_NODELABEL(usart6));

static void telemetry_uart_write(const char *data, int len)
{
	if (len <= 0 || !device_is_ready(telemetry_uart)) {
		return;
	}

	for (int i = 0; i < len; i++) {
		uart_poll_out(telemetry_uart, data[i]);
	}
}

static void telemetry_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		char line[TELEMETRY_BUF_SIZE];
		int len;

		len = snprintk(line, sizeof(line),
			       "%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%d,%d,%.6f,%.6f,%.6f,%.6f,%u,%d,%u,%u,%u,%d,%d,%.6f,%.6f,%u,%d,%u,%u,%u,%u\n",
			       (double)g_rm_imu.pitch,
			       (double)g_balance_ctrl.left_speed_target_rpm,
			       (double)g_balance_ctrl.pitch_p_out_rpm,
			       (double)g_balance_ctrl.pitch_d_out_rpm,
			       (double)g_balance_ctrl.left_speed_p_out,
			       (double)g_balance_ctrl.left_speed_i_out,
			       g_rm_m3508.motor[RM_M3508_CAN1_ID201].speed_rpm,
			       g_rm_m3508.motor[RM_M3508_CAN2_ID202].speed_rpm,
			       (double)g_dm4310.motor[0].pos_rad,
			       (double)g_dm4310.motor[1].pos_rad,
			       (double)g_dm4310.motor[2].pos_rad,
			       (double)g_dm4310.motor[3].pos_rad,
			       g_chassis_debug_override.x_axis_calib_count,
			       g_chassis_debug_override.x_axis_calib_ret,
			       g_dm4310.can1_home_valid,
			       g_dm4310.can1_home_active,
			       g_dm4310.can1_home_auto_saved,
			       g_dm4310.can1_home_load_ret,
			       g_dm4310.can1_home_save_ret,
			       (double)g_dm4310.can1_home_pos_rad[0],
			       (double)g_dm4310.can1_home_pos_rad[1],
			       g_dm4310.online_mask,
			       g_dm4310.last_send_ret,
			       g_dm4310.motor[0].rx_count,
			       g_dm4310.motor[1].rx_count,
			       g_dm4310.motor[2].rx_count,
			       g_dm4310.motor[3].rx_count);
		if (len > (int)sizeof(line)) {
			len = sizeof(line);
		}
		telemetry_uart_write(line, len);
		k_sleep(K_MSEC(TELEMETRY_THREAD_PERIOD_MS));
	}
}

void chassis_telemetry_start(void)
{
	k_thread_create(&telemetry_thread_data, telemetry_thread_stack,
			K_THREAD_STACK_SIZEOF(telemetry_thread_stack), telemetry_thread, NULL, NULL,
			NULL, TELEMETRY_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&telemetry_thread_data, "telemetry");
}
