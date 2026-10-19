#include "pi_controller.hpp"
#include <algorithm>
#include <cassert>

namespace motor_control
{
	PiController::PiController(const PiConfig_& cfg)
		: config(cfg)
		{
			assert(config.step_time > 0.0f && "Step time must be positive");
			assert(config.out_max >= config.out_min && "Output max must be greater than or equal to output min");
		}
	
	PidController::PidController(const PidConfig_& cfg)
		: config(cfg)
		{
			assert(config.step_time > 0.0f && "Step time must be positive");
			assert(config.out_max >= config.out_min && "Output max must be greater than or equal to output min");
		}

	float PiController::update(float reference, float feedback)
	{
		float error = reference - feedback;

		float p_term = config.Kp * error;

		const bool saturated_high = p_term + integral >= config.out_max && error > 0.0f;
		const bool saturated_low  = p_term + integral <= config.out_min && error < 0.0f;

		if (!saturated_high && !saturated_low)
		{
			integral += config.Ki * error * config.step_time;
		}

		integral = std::clamp(integral, config.out_min, config.out_max);

		float output = p_term + integral;

		// output saturation
		output = std::clamp(output, config.out_min, config.out_max);

		return output;
	}

	float PidController::update(float reference, float feedback)
	{
		float error = reference - feedback;

		float p_term = config.Kp * error;

		const bool saturated_high = p_term + integral >= config.out_max && error > 0.0f;
		const bool saturated_low = p_term + integral <= config.out_min && error < 0.0f;

		if (!saturated_high && !saturated_low)
		{
			integral += config.Ki * error * config.step_time;
		}

		integral = std::clamp(integral, config.out_min, config.out_max);

		derivative = (error - previous_error) / config.step_time;
		float d_term = config.Kd * derivative;

		float output = p_term + integral + d_term;

		// output saturation
		output = std::clamp(output, config.out_min, config.out_max);

		previous_error = error;

		return output;
	}

	void PiController::setLimits(float out_min, float out_max)
	{
		assert(out_max >= out_min && "Output max must be greater than or equal to output min");
		config.out_min = out_min;
		config.out_max = out_max;
	}

	void PidController::setLimits(float out_min, float out_max)
	{
		assert(out_max >= out_min && "Output max must be greater than or equal to output min");
		config.out_min = out_min;
		config.out_max = out_max;
	}

	float PiController::getIntegral() const
	{
		return integral;
	}

	float PidController::getIntegral() const
	{
		return integral;
	}

	float PidController::getDerivative() const
	{
		return derivative;
	}

	void PiController::reset()
	{
		integral = 0.0f;
	}

	void PidController::reset()
	{
		integral = 0.0f;
		previous_error = 0.0f;
		derivative = 0.0f;
	}
}
