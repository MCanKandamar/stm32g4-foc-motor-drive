#pragma once
#include "state_machine.hpp"

// Work done in each motor state. Called by the Supervisor:
// enter = once on entering the state, run = every 1 ms while in the state.

motor_control::MotorEvent prepare_start();     // at START: fix the mode, return START_HALL / START_SENSORLESS

void enter_align();
void run_align();

void enter_open_loop();
void run_open_loop();

void enter_closed_loop();
void run_closed_loop();

void enter_calibrating();
void run_calibrating();
