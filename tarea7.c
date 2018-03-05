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
 * GLOBAL VARIABLES
 *************************************************/
SemaphoreHandle_t binarySemaphore;
SemaphoreHandle_t countingSemaphore;

/**************************************************
 * PROTOTYPES
 *************************************************/
void initialize_peripherals(void);

/**************************************************
 * INTERRUPTS
 *************************************************/
void PORTA_IRQHandler( void )
{
	BaseType_t xHigherPriorityTaskWoken;
	PORT_ClearPinsInterruptFlags(PORTA, 1<<4);
	xHigherPriorityTaskWoken = pdFALSE;
	 /* The event has occurred, use the semaphore to unblock the task so the task
	 can process the event. */
	 xSemaphoreGiveFromISR(binarySemaphore, &xHigherPriorityTaskWoken );
	 /* Clear the interrupt here. */
	 /* Now the task has been unblocked a context switch should be performed if
	 xHigherPriorityTaskWoken is equal to pdTRUE. NOTE: The syntax required to perform
	 a context switch from an ISR varies from port to port, and from compiler to
	 compiler. Check the web documentation and examples for the port being used to
	 find the syntax required for your application. */
	 portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void PORTC_IRQHandler( void )
{

	BaseType_t xHigherPriorityTaskWoken;
	PORT_ClearPinsInterruptFlags(PORTC, 1<<6);
	xHigherPriorityTaskWoken = pdFALSE;
	 /* The event has occurred, use the semaphore to unblock the task so the task
	 can process the event. */
	 xSemaphoreGiveFromISR(countingSemaphore, &xHigherPriorityTaskWoken );
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
	for(;;)
	{

		xSemaphoreTake(binarySemaphore, portMAX_DELAY);
		GPIO_TogglePinsOutput(GPIOB, 1<<21);

	}


}

void sw3_dealing_task(void * pvParameters)
{

	static uint8_t index;

	for(;;)
	{

		 if( 10 == uxSemaphoreGetCount(countingSemaphore) )
		 {
				GPIO_TogglePinsOutput(GPIOB, 1<<22);
				//restart sem and start again
				for(index = 0; index < 10; index++)
				{
					xSemaphoreTake( countingSemaphore, portMAX_DELAY );
				}
		 }

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

	countingSemaphore = xSemaphoreCreateCounting(10, 0);
	binarySemaphore = xSemaphoreCreateBinary();

	xTaskCreate(sw2_dealing_task, "sw2", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
	xTaskCreate(sw3_dealing_task, "sw3", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);

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

	GPIO_WritePinOutput(GPIOB,21,1);
	GPIO_WritePinOutput(GPIOB,22,1);

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_SetPriority(PORTA_IRQn,2);
	NVIC_SetPriority(PORTC_IRQn,3);
}
