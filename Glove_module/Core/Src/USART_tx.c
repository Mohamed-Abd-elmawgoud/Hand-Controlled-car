/*
 * USART_tx.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Mohamed Abdelmawgoud
 */
#include "USART_tx.h"
uint8_t data_tx ;

void USART1_IRQHandler()
{
	USART1->DR = data_tx ;
}

void usart_init()
{
	__disable_irq() ;
	RCC->APB2ENR |= (1<<14)|(1<<2)|(1<<0);
	GPIOA->CRH = 0x444448B4 ;
	GPIOA->ODR |= (1<<10) ;

	USART1->CR1 |= (1<<3) ;
	//USART1->CR1 |= (1<<2) ;

	USART1->BRR = 833 ; // BaudRate 9600
	USART1->CR1 |= (1<<13) ;
	USART1->CR1 |= (1<<7) ;  //Enable TXE interrupt
	NVIC_EnableIRQ(USART1_IRQn) ;
	__enable_irq();
}


void usart_tx(uint8_t data)
{
//	while((USART1->SR & (1<<7)) == 0x00);
	data_tx = data ;
}


