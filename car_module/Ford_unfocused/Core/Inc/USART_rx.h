/*
 * USART_rx.h
 *
 *  Created on: Apr 23, 2024
 *      Author: Mohamed Abdelmawgoud
 */

#ifndef USART_RX_H_
#define USART_RX_H_
#include <stdint.h>
#include "stm32f103xb.h"

void usart_init();
void usart_rx(void* data) ;
char get_steering();
#endif /* USART_RX_H_ */
