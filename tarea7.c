/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    tarea7.c
 * @brief   Application entry point.
 */

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "freeRTOS.h"
#include "task.h"
#include "semphr.h"

/**************************************************
 * PROTOTYPES
 *************************************************/
void initialize_peripherals(void);

/**************************************************
 * INTERRUPTS
 *************************************************/
void PORTA_IRQHandler( void * pvParameters )
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	 /* The event has occurred, use the semaphore to unblock the task so the task
	 can process the event. */
	 xSemaphoreGiveFromISR(pvParameters, &xHigherPriorityTaskWoken );
	 /* Clear the interrupt here. */
	 /* Now the task has been unblocked a context switch should be performed if
	 xHigherPriorityTaskWoken is equal to pdTRUE. NOTE: The syntax required to perform
	 a context switch from an ISR varies from port to port, and from compiler to
	 compiler. Check the web documentation and examples for the port being used to
	 find the syntax required for your application. */
	 portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void PORTC_IRQHandler( void * pvParameters )
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	 /* The event has occurred, use the semaphore to unblock the task so the task
	 can process the event. */
	 xSemaphoreGiveFromISR(pvParameters, &xHigherPriorityTaskWoken );
	 /* Clear the interrupt here. */
	 /* Now the task has been unblocked a context switch should be performed if
	 xHigherPriorityTaskWoken is equal to pdTRUE. NOTE: The syntax required to perform
	 a context switch from an ISR varies from port to port, and from compiler to
	 compiler. Check the web documentation and examples for the port being used to
	 find the syntax required for your application. */
	 portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

/**************************************************
 * TASKS
 *************************************************/
void sw2_dealing_task(void * pvParameters)
{
	TickType_t xLastWakeTime;
	SemaphoreHandle_t binarySemaphore;

	const TickType_t xPeriod = pdMS_TO_TICKS( 10 );
	binarySemaphore = xSemaphoreCreateBinary();

	if (binarySemaphore != NULL)
	{
		/**semaphore was created*/
	}

	for(;;)
	{

		 if( xSemaphoreTake(binarySemaphore, portMAX_DELAY ) == pdTRUE )
		 {
				PORT_ClearPinsInterruptFlags(PORTA, 1<<4);

				GPIO_TogglePinsOutput(GPIOB, 21);
		 }
		 vTaskDelayUntil( &xLastWakeTime, xPeriod );

	}


}

void sw3_dealing_task(void * pvParameters)
{
	TickType_t xLastWakeTime;
	SemaphoreHandle_t countingSemaphore;

	uint8_t index;
	const TickType_t xPeriod = pdMS_TO_TICKS( 10 );
	countingSemaphore = xSemaphoreCreateCounting(10, 0);

	if ( NULL != countingSemaphore )
	{
		/**semaphore was created*/
	}

	for(;;)
	{

		 if( 10 == uxSemaphoreGetCount(countingSemaphore) )
		 {
				PORT_ClearPinsInterruptFlags(PORTC, 1<<6);
				GPIO_TogglePinsOutput(GPIOB, 22);
				//restart sem and start again
				for(index = 0; index < 10; index++)
				{
					xSemaphoreTake( countingSemaphore, portMAX_DELAY );
				}
		 }

		 vTaskDelayUntil( &xLastWakeTime, xPeriod );

	}


}

/**************************************************
 * MAIN CODE
 *************************************************/
int main(void)
{

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	initialize_peripherals();

	xTaskCreate(sw2_dealing_task, "sw2", 110, (void *) 0, configMAX_PRIORITIES, NULL);
	xTaskCreate(sw3_dealing_task, "sw3", 110, (void *) 0, configMAX_PRIORITIES, NULL);

	vTaskStartScheduler();

	for(;;)
	{

	}
	return 0;
}

void initialize_peripherals(void)
{
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortC);

	port_pin_config_t config_led = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };

	port_pin_config_t config_switch = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };

	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(PORTB, 21, &config_led);
    PORT_SetPinConfig(PORTB, 22, &config_led);

	PORT_SetPinConfig(PORTA, 4, &config_switch);
    PORT_SetPinConfig(PORTC, 6, &config_switch);


	gpio_pin_config_t led_config_gpio = { kGPIO_DigitalOutput, 1 };
	gpio_pin_config_t switch_config_gpio = { kGPIO_DigitalInput, 1 };

	GPIO_PinInit(GPIOB, 21, &led_config_gpio);
	GPIO_PinInit(GPIOB, 22, &led_config_gpio);

	GPIO_PinInit(GPIOA, 4, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 6, &switch_config_gpio);


	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);
}
