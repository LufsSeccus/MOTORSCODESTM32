#include "main.h"
#include "MOTORDRiVE.h"

void MOTOR::setMotor() {
	if (m_speed >= 0) {
		dutyCycle = m_speed * 21;
		_HAL_TIM_SET_COMPARE(M_TIM1, m_channel1, dutyCycle);
		_HAL_TIM_SET_COMPARE(M_TIM2, m_channel2, 0);
	}
	else {
		dutyCycle = m_speed * 21;
		_HAL_TIM_SET_COMPARE(M_TIM1, m_channel1, 0);
		_HAL_TIM_SET_COMPARE(M_TIM2, m_channel2, dutyCycle);
	}
}

void MOTOR::Init() {
    // Start PWM on both timers
    HAL_TIM_PWM_Start(M_TIM1, m_channel1);
    HAL_TIM_PWM_Start(M_TIM2, m_channel2);

    // Optional: initialize motor with stopped state
    HAL_TIM_PWM_Stop(M_TIM1, m_channel1);
    HAL_TIM_PWM_Stop(M_TIM2, m_channel2);
}

float PIDController :: update(float setpoint, float measurement) {
    float error = setpoint - measurement;//measurement is the output from the encoder -> go straight = measurement(speed) of both motors should be the same and so on 

    // Proportional
    float P = Kp * error;

    // Integral
    integrator += error * dt;
    float I = Ki * integrator;

    // Derivative (with low-pass filter)
    float derivative = (error - prev_error) / dt;
    float D = Kd * lowPassFilter(derivative);
    prev_error = error;

    output = P + I + D;
    return output;
}

void PIDController :: reset() {
    integrator = 0;
    prev_error = 0;
    prev_derivative = 0;
    output = 0;
}
