/*
 * flex.c
 *
 *  Created on: Apr 25, 2024
 *      Author: Mohamed Abdelmawgoud
 */

#include "FreeRTOS.h"
#include "task.h"
#include "flex.h"
#include "USART_tx.h"


#define size 5
float fingers_v[5] ;
float fingers_r[5] ;
float fingers_position[5] = {0};
char reading ;
char direction;
char gear ;
uint8_t channel =0 ;

void flex_init()
{
//	__disable_irq();
	RCC->APB2ENR |= (1<<9) ; // Enable Clock for ADC1.
	GPIOA->CRL = 0x44400000 ; // Configure adc pins as analog input.
	ADC1->CR2 = 0 ; // Clear the control register.
	ADC1->CR2 |= (1<<0) ;  // Enable ADC (power on);
	ADC1->CR2 |= (1<<2) ; // Start calibration.
	while((ADC1->CR2 & (1<<2))) ; //Wait for the calibration to finish.
	ADC1->SQR1 |= (0x4<<20) ; // set number of conversions to 1.
	ADC1->SQR3 = 0x0 ; // Select channel 0.
	ADC1->SQR3 |=(0x0<<0) ; // Select channel 0.
	ADC1->CR2 |= (7<<17) ;  // Select SWSTRT as the external trigger.
	ADC1->CR2 |= (1<<20) ;  // Enable external trigger.
//	ADC1->CR1 |= (1<<5) ; // Enable EOC interrupt.
//	NVIC_EnableIRQ(ADC1_IRQn) ;
//	__enable_irq();

}

char get_reading()
{
	return reading ;
}

void set_fingers_v(){
	fingers_v[channel] = (ADC1->DR*2.8)/4095.0;// Clear the EOC Flag.
	if(channel == 4)
	{
		channel = -1 ;
		flex_calc();
		read_hand() ;
	}
			channel ++ ;
			ADC1->SQR3 = channel ;	//ADC2->SQR3 = channel ;
}

void flex_calc()
{
	uint8_t i ;
	for(i =0 ;i<5;i++)
	{
		fingers_r[i] = (fingers_v[i]*22)/(3.3-fingers_v[i]);
		fingers_position[i] = (fingers_r[i]>105)?(1):(0) ;
	}
}
char get_direction()
{
	return direction ;
}

char get_gear()
{
	return gear ;
}
void read_hand()
{
	int speed = 0;
	if(fingers_r[0]>0 && fingers_r[0]<=88)
		{
			reading = '0' ;
			direction = 'S' ;
			gear = 0 ;
			return ;
		}
		else if(fingers_r[0] >88 && fingers_r[0]<=95)
		{
			speed = 0 ;
			gear = 1;
		}
		else if(fingers_r[0] >95 && fingers_r[0]<=100)
		{
			speed = 1 ;
			gear = 2 ;
		}
		else if(fingers_r[0] >100 && fingers_r[0]<=110)
		{
			speed = 2 ;
			gear = 3 ;
		}

	if(fingers_position[3] == 1)
	{
		reading = 'd'+speed ;
		direction = 'R' ;
	}
	else if(fingers_position[4] == 1)
	{
		reading = 'a'+speed ;
		direction = 'L' ;
	}
	else if(fingers_position[1] == 1)
	{
		reading = 'w'+speed ;
		direction = 'F' ;
	}
	else if(fingers_position[2] == 1)
	{
			reading = 's'+speed ;
			direction = 'B' ;
	}
	else
	{
		reading = '0' ;
		direction = 'S' ;
	}
}

