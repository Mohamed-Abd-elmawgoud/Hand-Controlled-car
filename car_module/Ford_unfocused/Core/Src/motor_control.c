/*
 * motor_control.c
 *
 *  Created on: Apr 25, 2024
 *      Author: malsy
 */
#include "motor_control.h"
#include "USART_rx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"

#define Kp 0.7
#define Kd 0.5
#define Ki 0.01
double dt =0.01;
int period_R,period_L;
double integral_R=0,integral_L=0;
double previos_error_R,previos_error_L;
float RPM_R=0,RPM_L=0,error_R,error_L,duty_R,duty_L;
TickType_t lastWakeTime1;
/*********************************************************  */
void TIM2_IC_PWM_init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN ; //EN TIM2 clk
	TIM2->ARR = 0xffff;
	TIM2->CCMR1 = 0x0021; //EN PWM1 on TIM3 CH1,CH2 ,preload EN
	TIM2->SMCR = 0x0057;
	TIM2->CNT = 0;
	//TIM2->CR1 |= TIM_CR1_ARPE; //en auto reload/preload
    TIM2->CR1 &= ~TIM_CR1_DIR ; // up counting
    TIM2->CR1 |= TIM_CR1_URS ; // only over/under flow generates update event
    //NVIC_EnableIRQ(TIM2_IRQn);//en tim2 interrupts
    //TIM2->DIER |= (TIM_DIER_CC2IE | TIM_DIER_UIE);  //Enable enterupt for CH3,CH4,overflow
    TIM2->CCER &= ~TIM_CCER_CC1P; // positive edge on TIM3_CH3
   	//TIM2->EGR |= TIM_EGR_UG ; //Update generation
   	TIM2->CCER |= (TIM_CCER_CC1E) ; //TIM3 CH1,CH2, EN
   	TIM2->CR1 |= TIM_CR1_CEN; // EN TIM3

}

void TIM3_IC_PWM_init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN ; //EN TIM2 clk
	TIM3->ARR = 0xFFFF;
	TIM3->CCMR1 = 0x0021; //EN PWM1 on TIM3 CH1,CH2 ,preload EN
	TIM3->SMCR = 0x0057;
	TIM3->CNT = 0;
	//TIM3->CR1 |= TIM_CR1_ARPE; //en auto reload/preload
    TIM3->CR1 &= ~TIM_CR1_DIR ; // up counting
    TIM3->CR1 |= TIM_CR1_URS ; // only over/under flow generates update event
   // NVIC_EnableIRQ(TIM3_IRQn);//en tim2 interrupts
    //TIM3->DIER |= (TIM_DIER_CC2IE | TIM_DIER_UIE);  //Enable enterupt for CH3,CH4,overflow
    TIM3->CCER &= ~TIM_CCER_CC1P; // positive edge on TIM3_CH3
	//TIM3->EGR |= TIM_EGR_UG ; //Update generation
	TIM3->CCER |= (TIM_CCER_CC1E) ; //TIM3 CH1,CH2, EN
	TIM3->CR1 |= TIM_CR1_CEN; // EN TIM3
}

void PWM_init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // ENABLE TIMER CLK
    TIM4->PSC = 20-1;
    TIM4->ARR = 0xFFFF;
    TIM4->CCMR2 = 0x6868; // PWM on TIM4 CH1,CH2
    TIM4->EGR |= TIM_EGR_UG ; //Update generation
    NVIC_EnableIRQ(TIM4_IRQn);//en tim4 interrupts
    TIM4->DIER |= (TIM_DIER_UIE);
    TIM4->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC4E );
    TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM4_IRQHandler(){
	period_R = TIM2->CNT;
	period_L = TIM3->CNT;
	TIM2->CNT =0;
	TIM3->CNT =0;

	TIM4->SR &= ~TIM_SR_UIF;
}

void update_dutyR(float res){
	if(res > 100){
		TIM4->CCR3= 0xffff;
	}
	else{
	    TIM4->CCR3 =(uint16_t)((res*0xFFFF)/100.0);
	}
}

void update_dutyL(float res){
	if(res>100){
		TIM4->CCR4= 0xffff;
	}
	else{
		TIM4->CCR4 = (uint16_t)((res*0xFFFF)/100.0);
	}
}

void calc_rpm(){
	if(period_R != 0)
		RPM_R = (period_R/(20.0*0.16384))*60 ;
	else
		RPM_R = 0;
	if(period_L != 0)
		RPM_L = (period_L/(20.0*0.16384))*60 ;
	else
		RPM_L = 0;

}

void PID_controller(){

	integral_R = (error_R + previos_error_R)/2 * dt; //calculating integral part
	duty_R = error_R * Kp + Ki * integral_R + Kd *(error_R - previos_error_R)/dt; //updating duting cycle
	previos_error_R = error_R;
	integral_L = (error_R + previos_error_R)/2 * dt;
	duty_L = error_L * Kp + Ki * integral_L + Kd *(error_L - previos_error_L)/dt;
	previos_error_L = error_L;


}

void GPIO_init(){
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN);
	GPIOB->CRH = 0x444444BB;//PB8, PB9 PWM for right and left motor
	GPIOA->CRL = 0x48333348; //PA0, PA6 input from encoder, PA2-PA5 direction for motors
    GPIOA->ODR &= ~((1<<0) | (1<<6));
}

void start_engine(){
    GPIO_init();
    TIM3_IC_PWM_init();
    TIM2_IC_PWM_init();
    PWM_init();
}
void drive(enum gear_box desired_rpm_R , enum gear_box desired_rpm_L,enum direction dir){
    	calc_rpm();
    	update_direction(dir);
    	error_R = desired_rpm_R - RPM_R;
    	error_L = desired_rpm_L - RPM_L;

    	PID_controller();
    	update_dutyR(abs_value(duty_R));
    	update_dutyL(abs_value(duty_L));

}

float abs_value(float number){
	if(number < 0){
		number = -1*number;
	}
	return number;
}

void update_direction(enum direction dir){
	GPIOA->ODR &= ~(0xf<<2);
	switch(dir){
	case FORWARD : GPIOA->ODR |= (0b1010<<2); break;
	case BACKWARD : GPIOA->ODR |= (0b0101<<2); break;
	case RIGHT : GPIOA->ODR |= (0b1011<<2); break;
	case LEFT : GPIOA->ODR |= (0b1110<<2); break;
	default : GPIOA->ODR |= (0b1111<<2);
	}
}


void car_process(void* data)
{
    lastWakeTime1 = xTaskGetTickCount();
	while(1){
	char steering = get_steering();
	enum direction dir ;
	enum gear_box speed;
	switch (steering){
	case 'w' :
    case 'w'+1 :
	case 'w'+2 : dir = FORWARD;break;
	case 's' :
	case 's'+1 :
	case 's'+2 : dir = BACKWARD;break;
	case 'd' :
	case 'd'+1 :
	case 'd'+2 : dir = RIGHT ;break;
	case 'a' :
	case 'a'+1 :
	case 'a'+2 : dir = LEFT ; break;
	default : dir = BRAKE;
	}
	switch (steering){
	case 'w' :
	case 'a' :
	case 's' :
	case 'd' : 	speed = gear1 ; break;
	case 'w'+1 :
	case 'a'+1 :
	case 's'+1 :
	case 'd'+1 : 	speed = gear2 ; break;
	case 'w'+2 :
	case 'a'+2 :
	case 's'+2 :
	case 'd'+2 : 	speed = gear3 ; break;
	default : speed = neutral;
	}
	drive(speed,speed,dir);
	vTaskDelayUntil(&lastWakeTime1,pdMS_TO_TICKS(10));
	}

}

