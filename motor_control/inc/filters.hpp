#pragma once

namespace motor_control
{
    struct LowPassFilter
    {
        float alpha;
        float output;
        bool initialized;

        float update(float input)
        {
            if (!initialized)
            {
                output = input;
                initialized = true;
            }
            else
            {
                output += alpha * (input - output);
            }

            return output;
        }
    };
}