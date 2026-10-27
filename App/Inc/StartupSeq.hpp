#pragma once
#include "state_machine.hpp"

// Motor durumlarının işi. Supervisor çağırır:
// enter = duruma girişte bir kez, run = durumda kaldıkça her 1 ms.

motor_control::MotorEvent prepare_start();     // START anında: modu sabitle, START_HALL / START_SENSORLESS döndür

void enter_align();
void run_align();

void enter_open_loop();
void run_open_loop();

void enter_closed_loop();
void run_closed_loop();

void enter_calibrating();
void run_calibrating();
