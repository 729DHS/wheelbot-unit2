cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unit2_app)

target_sources(app PRIVATE
	src/main.c
	src/balance_ctrl.c
	src/chassis_base.c
	src/chassis_inputs.c
	src/chassis_outputs.c
	src/chassis_telemetry.c
	src/chassis_threads.c
	src/chassis_sm.c
	src/x_axis_motor_map.c
	src/rm_sbus.c
	src/dm4310_motor.c
	src/rm_m3508.c
	src/rm_imu.c
	src/lk_motor.c
)
