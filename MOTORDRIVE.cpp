#include "main.h"
#include "MOTORDRiVE.h"

void MOTOR::setMotor() {
	if (m_speed > 0) {
		dutyCycle = m_speed * 21;
		_HAL_TIM_SET_COMPARE(M_TIM1, m_channel1, dutyCycle);
		_HAL_TIM_SET_COMPARE(M_TIM2, m_channel2, 0);
	}
	else if(m_speed < 0) {
		dutyCycle = abs(m_speed) * 21;
		_HAL_TIM_SET_COMPARE(M_TIM1, m_channel1, 0);
		_HAL_TIM_SET_COMPARE(M_TIM2, m_channel2, dutyCycle);
	}
	else {
		_HAL_TIM_SET_COMPARE(M_TIM1, m_channel1, 0);
		_HAL_TIM_SET_COMPARE(M_TIM2, m_channel2, 0);
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

void Movements::goStraight(PIDController* leftPID, PIDController* rightPID,
                           float setpoint, float leftMeas, float rightMeas) {
    float leftOutput = leftPID->update(setpoint, leftMeas);
    float rightOutput = rightPID->update(setpoint, rightMeas);

    // Clamp outputs to safe range (e.g., -1000 to +1000)
    if (leftOutput > 1000) leftOutput = 1000;
    if (leftOutput < -1000) leftOutput = -1000;
    if (rightOutput > 1000) rightOutput = 1000;
    if (rightOutput < -1000) rightOutput = -1000;

    leftMotor->setSpeed((int32_t)leftOutput);
    rightMotor->setSpeed((int32_t)rightOutput);

    leftMotor->setMotor();
    rightMotor->setMotor();
}


void Movements::rotateLeft(PIDController* leftPID, PIDController* rightPID,
    float setpoint, float leftMeas, float rightMeas) {

    float leftOutput = leftPID->update(-setpoint, leftMeas);   // Reverse left motor
    float rightOutput = rightPID->update(setpoint, rightMeas); // Forward right motor

    leftOutput = std::clamp(leftOutput, -1000.0f, 1000.0f);
    rightOutput = std::clamp(rightOutput, -1000.0f, 1000.0f);

    leftMotor->setSpeed((int32_t)leftOutput);
    rightMotor->setSpeed((int32_t)rightOutput);

    leftMotor->setMotor();
    rightMotor->setMotor();
}

void Movements::rotateRight(PIDController* leftPID, PIDController* rightPID,
    float setpoint, float leftMeas, float rightMeas) {

    float leftOutput = leftPID->update(setpoint, leftMeas);    // Forward left motor
    float rightOutput = rightPID->update(-setpoint, rightMeas); // Reverse right motor

    leftOutput = std::clamp(leftOutput, -1000.0f, 1000.0f);
    rightOutput = std::clamp(rightOutput, -1000.0f, 1000.0f);

    leftMotor->setSpeed((int32_t)leftOutput);
    rightMotor->setSpeed((int32_t)rightOutput);

    leftMotor->setMotor();
    rightMotor->setMotor();
}


void Movements::stop() {
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
    leftMotor->setMotor();
    rightMotor->setMotor();
}


float PIDController :: update(float setpoint, float measurement) {
    float error = setpoint - measurement;
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
