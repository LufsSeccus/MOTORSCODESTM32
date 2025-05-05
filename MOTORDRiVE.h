#ifndef MOTOR_DRIVE_WITH_FILTERED_PID
#define MOTOR_DRIVE_WITH_FILTERED_PID

/*  Set up the PWM timers as per instructions, prescaler = 3, period = 999 -> f = 21000Hz */

#include"main.h"

#define CH1 TIM_CHANNEL_1
#define CH2 TIM_CHANNEL_2
#define CH3 TIM_CHANNEL_3
#define CH4 TIM_CHANNEL_4

uint 32_t dutyCycle;

class MOTOR {
private: 
	uint16_t IN_1;
	uint16_t IN_2;
	uint32_t m_speed; // m_speed is a value ranging from -1000-1000
	TIM_HandleTypeDef* M_TIM1;
	TIM_HandleTypeDef* M_TIM2;
	uint32_t m_channel1
	uint32_t m_channel2
public:
	MOTOR(uint16_t PIN_1, uint16_t PIN_2, TIM_HandleTypeDef* TIM1, TIM_HandleTypeDef* TIM2, uint32_t CHANNEL1, uint32_t CHANNEL2)
	: IN_1{ PIN_1 }, IN_2{ PIN_2] ,  M_TIM1 { TIM1 },  M_TIM2 { TIM2 }, m_channel1 { CHANNEL1}, m_channel2 { CHANNEL2};
	void setSpeed(uint32_t speed) { m_speed = speed; };
	void setMotor();
	void emergencyStop() { setSpeed(0); };
	void Innit(); 
}

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float dt, float filter_tau)
        : Kp(kp), Ki(ki), Kd(kd), dt(dt), tau(filter_tau),
        integrator(0), prev_error(0), prev_derivative(0), output(0) {
    }
    float update(float setpoint, float measurement);
    void reset();

private:
    float Kp, Ki, Kd;
    float dt;         // Sample time
    float tau;        // Filter time constant
    float integrator;
    float prev_error;
    float prev_derivative;
    float output;

    // First-order low-pass filter on derivative term
    float lowPassFilter(float new_derivative) {
        float alpha = tau / (tau + dt);
        prev_derivative = alpha * prev_derivative + (1 - alpha) * new_derivative;
        return prev_derivative;
    }
    };

