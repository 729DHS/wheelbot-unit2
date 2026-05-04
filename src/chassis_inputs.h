#ifndef CHASSIS_INPUTS_H_
#define CHASSIS_INPUTS_H_

#include "balance_ctrl.h"
#include "chassis_sm.h"

#include <stdint.h>

void chassis_build_balance_input(struct balance_ctrl_input *input, uint32_t now);
void chassis_build_sm_input(struct chassis_sm_input *input, uint32_t now);

#endif
