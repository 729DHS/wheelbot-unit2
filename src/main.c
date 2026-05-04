#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

void main(void)
{
	printk("Unit2 booted on RoboMaster Type-C Board (STM32F407IG)\n");

	while (1) {
		k_sleep(K_SECONDS(1));
	}
}
