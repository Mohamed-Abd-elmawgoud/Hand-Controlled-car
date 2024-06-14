#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stm32f103xb.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
#include "motor_control.h"
#include "USART_rx.h"

int main(){
	usart_init();
		start_engine();
		xTaskCreate(usart_rx,"BLUETOOTH",128,NULL,tskIDLE_PRIORITY+1,NULL);
		xTaskCreate(car_process,"Car Drive",128,NULL,tskIDLE_PRIORITY+2,NULL);

		vTaskStartScheduler();

}
