#pragma once

// Motor durumlarının işi. Supervisor çağırır:
// enter = duruma girişte bir kez, run = durumda kaldıkça her 1 ms.

void enter_adc_cal();
void run_adc_cal();

void enter_align();
void run_align();

void enter_open_loop();
void run_open_loop();

void enter_closed_loop();
void run_closed_loop();

void enter_calibrating();
void run_calibrating();
