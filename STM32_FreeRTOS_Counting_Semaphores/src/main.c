/**
  ******************************************************************************
  * @file    main.c
  * @author  Tipu khan
  * @version V1.0
  * @date    02-April-2021
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// function prototypes
static void prvSetupHardware(void);
static void prvSetupUART(void);
static void prvSetupGPIO(void);
void printMsg(char *msg);

// task prototypes
void vHandlerTask(void *params);
void vPeriodicTask(void *params);

/* Enable the software interrupt and set its priority. */
static void prvSetupSoftwareInterrupt();

/* Variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xCountingSemaphore;

int main(void)
{
	// Reset the RCC clock configuration to the default reset state.
	// HSI ON, PLL OFF, HSE OFF, system clock = 16MHz, cpu_clock = 16MHz
	RCC_DeInit();

	// Update the SystemCoreClock variable
	SystemCoreClockUpdate();

	prvSetupHardware();

	xCountingSemaphore = xSemaphoreCreateCounting(10, 0);

	if(xCountingSemaphore != NULL)
	{
		// Enable the button interrupt and set its priority.
		prvSetupSoftwareInterrupt();

		// Create handler task
		/* This is the task that will be synchronized with the interrupt.
		 * The handler task is created with a high priority to ensure it runs
		 * immediately after the interrupt exits. */
		xTaskCreate(vHandlerTask, "Handler", 500, NULL, 3, NULL);

		// Create periodic task
		/* It will periodically generate a software interrupt. This is created with
		 * a priority below the handler task to ensure it will get preempted each
		 * time the handler task exit the Blocked state. */
		xTaskCreate(vPeriodicTask, "Periodic", 500, NULL, 1, NULL);

		// 5. Start the scheduler
		vTaskStartScheduler();
	}

	for(;;);
}

void vHandlerTask(void *params)
{
	while(1)
	{
		/* Use the semaphore to wait for the event.  The semaphore was created
		before the scheduler was started so before this task ran for the first
		time.  The task blocks indefinitely meaning this function call will only
		return once the semaphore has been successfully obtained - so there is no
		need to check the returned value. */
		xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);

		// To get here the event must have occurred.
		// Process the event (in this case we just print out a message).
		printMsg("Handler task - Processing event.\r\n");
	}
}

void vPeriodicTask(void *params)
{
	while(1)
	{
		// This task is just used to 'simulate' an interrupt.
		// This is done by periodically generating a software interrupt.

		vTaskDelay( pdMS_TO_TICKS(500) );

		/* Generate the interrupt, printing a message both before hand and
		afterwards so the sequence of execution is evident from the output. */
        printMsg("Periodic task - Pending the interrupt.\r\n" );

        // pend the interrupt
        NVIC_SetPendingIRQ(EXTI0_IRQn);

        printMsg("Periodic task - Resuming.\r\n" );
	}
}

void EXTI0_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* 'Give' the semaphore multiple times.  The first will unblock the handler
	task, the following 'gives' are to demonstrate that the semaphore latches
	the events to allow the handler task to process them in turn without any
	events getting lost.  This simulates multiple interrupts being taken by the
	processor, even though in this case the events are simulated within a single
	interrupt occurrence.*/

	printMsg("==>Button_Handler\r\n");

	for(uint8_t i = 0; i<5 ; i++)
	{
		xSemaphoreGiveFromISR(xCountingSemaphore, &xHigherPriorityTaskWoken);
	}

    /* Clear the software interrupt bit using the interrupt controllers  */
	EXTI_ClearITPendingBit(EXTI_Line0);

	/* once the ISR exits, the below macro makes higher priority task which got unblocked to resume on the CPU */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void prvSetupSoftwareInterrupt()
{
	// here were simulating the button interrupt by manually setting the interrupt
	// enable bit in the NVIC enable register

	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
	function so the interrupt priority must be at or below the priority defined
	by configSYSCALL_INTERRUPT_PRIORITY. */

	NVIC_SetPriority(EXTI0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );

	// Enable the interrupt.
	NVIC_EnableIRQ(EXTI0_IRQn);
}

static void prvSetupHardware(void)
{
	// setup button and LED
	prvSetupGPIO();
	//setup UART2
	prvSetupUART();
}

void printMsg(char *msg)
{
	for(uint32_t i=0; i < strlen(msg); i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET);
		USART_SendData(USART2, msg[i]);
	}
}

static void prvSetupUART(void)
{
	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef uart2_init;

	// 1. Enable the UART2 and GPIOA Peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// PA2 is UART2_TX, PA3 is UART2_RX

	// 2. Alternate function configuration of MCU pins to behave as UART2_TX and RX
	memset(&gpio_uart_pins, 0, sizeof(gpio_uart_pins));
	gpio_uart_pins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF;
	gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpio_uart_pins);

	// 3. AF mode settings for the pins
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // PA2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // PA3

	// 4. UART parameter initializations
	memset(&uart2_init, 0, sizeof(uart2_init));
	uart2_init.USART_BaudRate = 115200;
	uart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart2_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	uart2_init.USART_Parity = USART_Parity_No;
	uart2_init.USART_StopBits = USART_StopBits_1;
	uart2_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &uart2_init);

	// 5. Enable the UART2 peripheral
	USART_Cmd(USART2, ENABLE);
}

static void prvSetupGPIO(void)
{
	// This function is board specific

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitTypeDef ledInit, buttonInit;
	ledInit.GPIO_Mode = GPIO_Mode_OUT;
	ledInit.GPIO_OType = GPIO_OType_PP;
	ledInit.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	ledInit.GPIO_Speed = GPIO_Low_Speed;
	ledInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &ledInit);

	buttonInit.GPIO_Mode = GPIO_Mode_IN;
	buttonInit.GPIO_OType = GPIO_OType_PP;
	buttonInit.GPIO_Pin = GPIO_Pin_0;
	buttonInit.GPIO_Speed = GPIO_Low_Speed;
	buttonInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &buttonInit);

}
