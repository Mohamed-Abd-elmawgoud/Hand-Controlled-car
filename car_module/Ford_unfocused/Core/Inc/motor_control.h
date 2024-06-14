	/*
 * motor_control.h
 *
 *  Created on: Apr 25, 2024
 *      Author: malsy
 */
#include <stm32f103xb.h>

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_
enum direction {
	BRAKE=0,
	FORWARD,
	BACKWARD,
	RIGHT,
	LEFT
};
enum gear_box {
	neutral=0,
	gear1=100,
	gear2=200,
	gear3=400
};

void start_engine();
void TIM3_IC_PWM_init();
void TIM2_IC_PWM_init();
void update_dutyR(float res);
void update_dutyL(float res);
void calc_rpm();
void GPIO_init();
void PWM_init();
void PID_controller();
void drive(enum gear_box desired_rpm_R , enum gear_box desired_rpm_L,enum direction);
float abs_value(float number);
void update_direction(enum direction dir);
void car_process(void* steering);
#endif /* MOTOR_CONTROL_H_ */
