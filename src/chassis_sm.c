#include "chassis_sm.h"

#include <string.h>

volatile struct chassis_sm g_chassis_sm = {
	.magic = CHASSIS_SM_MAGIC,
};

static uint8_t switch_is_on(uint8_t value)
{
	return value == 2U ? 1U : 0U;
}

void chassis_sm_init(volatile struct chassis_sm *sm)
{
	memset((void *)sm, 0, sizeof(*sm));
	sm->magic = CHASSIS_SM_MAGIC;
	sm->system_state = CHASSIS_SM_SYSTEM_DISABLED;
	sm->joint_state = CHASSIS_SM_JOINT_RELAX;
	sm->chassis_state = CHASSIS_SM_CHASSIS_OFF;
}

void chassis_sm_update(volatile struct chassis_sm *sm, const struct chassis_sm_input *input)
{
	uint8_t system_state;
	uint8_t joint_state;
	uint8_t chassis_state;

	sm->update_count++;
	sm->last_ms = input->now_ms;
	sm->events = 0U;
	sm->sbus_connected = input->sbus_connected;
	sm->fault = input->fault;
	sm->ch5 = input->ch5;
	sm->ch6 = input->ch6;
	sm->ch8 = input->ch8;
	sm->ch5_on = switch_is_on(input->ch5);
	sm->ch6_on = switch_is_on(input->ch6);
	sm->ch8_on = switch_is_on(input->ch8);

	system_state = sm->system_state;
	if (input->sbus_connected == 0U || input->fault != 0U) {
		if (system_state != CHASSIS_SM_SYSTEM_ESTOP) {
			sm->events |= CHASSIS_SM_EVENT_SYSTEM_ESTOP;
		}
		system_state = CHASSIS_SM_SYSTEM_ESTOP;
	} else if (sm->ch5_on != 0U) {
		if (system_state != CHASSIS_SM_SYSTEM_ENABLED) {
			if ((input->joint_online_mask & CHASSIS_SM_JOINT_REQUIRED_MASK) ==
			    CHASSIS_SM_JOINT_REQUIRED_MASK) {
				for (int i = 0; i < CHASSIS_SM_JOINT_COUNT; i++) {
					sm->joint_hold_ref[i] = input->joint_position[i];
				}
				sm->joint_hold_valid_mask =
					input->joint_online_mask & CHASSIS_SM_JOINT_REQUIRED_MASK;
				sm->events |= CHASSIS_SM_EVENT_SYSTEM_ENABLED |
					      CHASSIS_SM_EVENT_JOINT_CAPTURED;
				system_state = CHASSIS_SM_SYSTEM_ENABLED;
			} else {
				system_state = CHASSIS_SM_SYSTEM_DISABLED;
			}
		} else {
			system_state = CHASSIS_SM_SYSTEM_ENABLED;
		}
	} else {
		if (system_state == CHASSIS_SM_SYSTEM_ENABLED) {
			system_state = CHASSIS_SM_SYSTEM_ESTOP;
			sm->events |= CHASSIS_SM_EVENT_SYSTEM_ESTOP;
		} else if (system_state != CHASSIS_SM_SYSTEM_ESTOP) {
			system_state = CHASSIS_SM_SYSTEM_DISABLED;
		}
	}
	sm->system_state = system_state;

	joint_state = (system_state == CHASSIS_SM_SYSTEM_ENABLED && sm->ch6_on != 0U) ?
			      CHASSIS_SM_JOINT_HOLD :
			      CHASSIS_SM_JOINT_RELAX;
	if (joint_state != sm->joint_state) {
		sm->events |= (joint_state == CHASSIS_SM_JOINT_HOLD) ?
				      CHASSIS_SM_EVENT_JOINT_HOLD :
				      CHASSIS_SM_EVENT_JOINT_RELAX;
		sm->joint_state = joint_state;
	}

	chassis_state = (system_state == CHASSIS_SM_SYSTEM_ENABLED && sm->ch8_on != 0U) ?
				CHASSIS_SM_CHASSIS_RUN :
				CHASSIS_SM_CHASSIS_OFF;
	if (chassis_state != sm->chassis_state) {
		sm->events |= (chassis_state == CHASSIS_SM_CHASSIS_RUN) ?
				      CHASSIS_SM_EVENT_CHASSIS_RUN :
				      CHASSIS_SM_EVENT_CHASSIS_OFF;
		sm->chassis_state = chassis_state;
	}

	sm->allow_output = system_state == CHASSIS_SM_SYSTEM_ENABLED ? 1U : 0U;
	sm->estop_active = system_state == CHASSIS_SM_SYSTEM_ESTOP ? 1U : 0U;
	sm->joint_hold_active = sm->joint_state == CHASSIS_SM_JOINT_HOLD ? 1U : 0U;
	sm->chassis_control_active =
		sm->chassis_state == CHASSIS_SM_CHASSIS_RUN ? 1U : 0U;
	sm->ch5_prev_on = sm->ch5_on;
	sm->ch6_prev_on = sm->ch6_on;
	sm->ch8_prev_on = sm->ch8_on;
}
