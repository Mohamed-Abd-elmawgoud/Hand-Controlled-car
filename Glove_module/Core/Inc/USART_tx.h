/*
 * USART_tx.h
 *
 *  Created on: Apr 23, 2024
 *      Author: Mohamed Abdelmawgoud
 */

#ifndef USART_TX_H_
#define USART_TX_H_
#include <stdint.h>
#include "stm32f103xb.h"



void usart_init();
void usart_tx(uint8_t data) ;
void usart_send(char msg[3]);

#endif /* USART_TX_H_ */
