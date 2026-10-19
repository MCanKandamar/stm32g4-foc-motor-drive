#pragma once

namespace motor_control {

    struct PiConfig_ {
        float Kp;  // Proportional gain
        float Ki;  // Integral gain
        float step_time;  // Time step for integration
        float out_max;  // Maximum output
        float out_min;  // Minimum output
    };

        struct PidConfig_ {
        float Kp;  // Proportional gain
        float Ki;  // Integral gain
        float Kd;  // Derivative gain
        float step_time;  // Time step for integration
        float out_max;  // Maximum output
        float out_min;  // Minimum output
    };

    class PiController {

        private:
            PiConfig_ config;
            float integral{0.0f};  // Current integral value

        public:
            PiController(const PiConfig_& cfg);

            float update(float reference, float feedback);

            void setLimits(float out_min, float out_max);

            void setIntegral(float value)   {integral = value; }

            void reset();

            float getIntegral() const;

    };

    class PidController {

        private:
            PidConfig_ config;
            float integral{0.0f};  // Current integral value
            float previous_error{0.0f};  // Previous error value for derivative calculation
            float derivative{0.0f};  // Current derivative value

        public:
            PidController(const PidConfig_& cfg);

            float update(float reference, float feedback);

            void setLimits(float out_min, float out_max);

            void reset();

            float getIntegral() const;
            float getDerivative() const;
    };
}

