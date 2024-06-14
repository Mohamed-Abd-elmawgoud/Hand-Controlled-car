/*
 * USART_rx.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Mohamed Abdelmawgoud
 */
#include "USART_rx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
char steering ;
TickType_t lastWakeTime2;

void usart_init()
{
	RCC->APB2ENR |= (1<<14)|(1<<2)|(1<<0);
	GPIOA->CRH = 0x444448B4 ;
	GPIOA->ODR |= (1<<10) ;

	USART1->CR1 |= (1<<13) ;
	//USART1->CR1 |= (1<<3) ;
	USART1->CR1 |= (1<<2) ;
	USART1->BRR = 833 ; // BaudRate 9600
}

void usart_rx(void* data)
{
	lastWakeTime2 = xTaskGetTickCount();
	while(1){
	while(!(USART1->SR & (1<<5))) ;
	steering = USART1->DR ;
	USART1->SR &= ~(1<<5) ;
	vTaskDelayUntil(&lastWakeTime2,pdMS_TO_TICKS(5));
	}
}

char get_steering(){
	return steering;
}
