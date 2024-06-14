/*
 * flex.h
 *
 *  Created on: Apr 25, 2024
 *      Author: Mohamed Abdelmawgoud
 */

#ifndef FLEX_H_
#define FLEX_H_
#include "stm32f103xb.h"


void flex_init() ;
char get_reading();
void ADC_Read();
void flex_calc();
void read_hand() ;
void set_fingers_v() ;
char get_direction() ;
char get_gear() ;
#endif /* FLEX_H_ */
