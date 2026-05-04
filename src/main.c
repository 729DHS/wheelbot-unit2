#include "chassis_base.h"
#include "chassis_threads.h"
#include "dm4310_motor.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	printk("Unit2: joint-hold + hub-motor PID balance\n");

	chassis_base_init();
	chassis_threads_start();

	/* Wait up to 5s for DM4310 bringup (joints need 24V power) */
	printk("Waiting for DM4310 bringup (max 5s)...\n");
	for (int i = 0; i < 100 && !g_dm4310.bringup_done; i++) {
		k_sleep(K_MSEC(50));
	}
	if (g_dm4310.bringup_done) {
		printk("Bringup done (%u motors online), holding joints\n", g_dm4310.online_mask);
	} else {
		printk("Bringup timeout - joint hold disabled, balance only\n");
	}

	/* Auto-enable without remote: force SBUS connected + all switches on */
	g_chassis_debug_override.magic = 0x44424731U;
	g_chassis_debug_override.force_sbus_connected = 1U;
	g_chassis_debug_override.force_ch5 = 2U;
	g_chassis_debug_override.force_ch6 = 2U;
	g_chassis_debug_override.force_ch8 = 2U;

	printk("Balance control auto-enabled. Stand by.\n");

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
