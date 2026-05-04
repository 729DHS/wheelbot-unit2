cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unit2_app)

target_sources(app PRIVATE
	src/main.c
	src/balance_ctrl.c
	src/chassis_base.c
	src/lk_motor.c
)
