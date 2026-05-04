#ifndef CHASSIS_OUTPUTS_H_
#define CHASSIS_OUTPUTS_H_

#include <stdint.h>

uint8_t chassis_apply_joint_state(void);
void chassis_apply_balance_control(uint32_t now);
void chassis_apply_m3508_command(void);
void chassis_update_common_status(uint32_t now);

#endif
