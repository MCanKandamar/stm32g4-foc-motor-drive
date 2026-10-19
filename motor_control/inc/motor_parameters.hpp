#pragma once

namespace motor_control
{
    struct MotorParameters
    {
        // Motor identification
        const char* model;
        
        int number_of_poles;
        int number_of_phases;

        // Rated values
        float nominal_voltage;        // V
        float rated_angular_speed;    // rad/s
        float continuous_torque;      // N·m
        float rated_current;          // A
        float output_power;           // W

        // Peak values
        float peak_torque;            // N·m
        float peak_current;           // A

        // Motor constants
        float torque_constant;        // N·m/A
        float back_emf_constant;      // V·s/rad

        // Mechanical parameters
        float rotor_inertia;          // kg·m²
    };

    constexpr MotorParameters BLF02 =
    {
        .model = "57BLF02",

        .number_of_poles        = 8,
        .number_of_phases       = 3,

        .nominal_voltage        = 24.0f,
        .rated_angular_speed    = 314.1593f,
        .continuous_torque      = 0.4f,
        .rated_current          = 7.8f,
        .output_power           = 125.0f,

        .peak_torque            = 1.2f,
        .peak_current           = 23.5f,

        .torque_constant        = 0.066f,
        .back_emf_constant      = 0.06016f,

        .rotor_inertia          = 1.70e-5f,
    };
}