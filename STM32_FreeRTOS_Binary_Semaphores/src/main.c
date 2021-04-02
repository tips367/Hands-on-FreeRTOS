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
void vManagerTask(void *params);
void vEmployeeTask(void *params);

/* Variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize both manager and employee task */
xSemaphoreHandle xWorkSemaphore;

// Queue which manager uses to put the work ticket id
xQueueHandle xWorkQueue;

int main(void)
{
	// Reset the RCC clock configuration to the default reset state.
	// HSI ON, PLL OFF, HSE OFF, system clock = 16MHz, cpu_clock = 16MHz
	RCC_DeInit();

	// Update the SystemCoreClock variable
	SystemCoreClockUpdate();

	prvSetupHardware();

	vSemaphoreCreateBinary(xWorkSemaphore);

	xWorkQueue = xQueueCreate(1, sizeof(unsigned int));

	if( (xWorkSemaphore != NULL) && (xWorkQueue != NULL) )
	{
		// 'Manager' task. This is the task that will be synchronized with the Employee task.
		xTaskCreate(vManagerTask, "Manager", 500, NULL, 1, NULL);

		// Create employee task
		xTaskCreate(vEmployeeTask, "Employee", 500, NULL, 1, NULL);

		// 5. Start the scheduler
		vTaskStartScheduler();
	}

	for(;;);
}

void vManagerTask(void *params)
{
	unsigned int xWorkTicketId;
	portBASE_TYPE xStatus;

	/* The semaphore is created in the 'empty' state, meaning the semaphore must first be given
	 * using the xSemaphoreGive() API function before it can subsequently be taken (obtained) */
	xSemaphoreGive(xWorkSemaphore);

	while(1)
	{
		// get a work ticket id (some random number)
		xWorkTicketId = ( rand() & 0x1FF );

		/* Sends work ticket id to the work queue */
		xStatus = xQueueSend(xWorkQueue, &xWorkTicketId , portMAX_DELAY);
		if( xStatus != pdPASS )
		{
			printMsg("Could not send to the queue.\r\n");
		}
		else
		{
			/* Manager notifying the employee by giving semaphore */
			xSemaphoreGive(xWorkSemaphore);
			taskYIELD();
		}
	}
}

void employeeDoWork(uint8_t ticket_id)
{
	char msg[20];
	snprintf(msg, sizeof(msg), "Ticket ID: %d\r\n", ticket_id);
	printMsg(msg);
	vTaskDelay(ticket_id);
}

void vEmployeeTask(void *params)
{
	char msg[50];
	unsigned char xWorkTicketId;
	portBASE_TYPE xStatus;

    while(1)
    {
		/* First Employee tries to take the semaphore, if it is available that means there
		 * is a task assigned by manager, otherwise employee task will be blocked */
		xSemaphoreTake(xWorkSemaphore, 0);

		// get the ticket id from the work queue
		xStatus = xQueueReceive(xWorkQueue, &xWorkTicketId, 0);

		if( xStatus == pdPASS )
		{
			employeeDoWork(xWorkTicketId);
		}
		else
		{
			/* We did not receive anything from the queue.  This must be an error
			 * as this task should only run when the manager assigns at least one work. */
			sprintf(msg,"Employee task : Queue is empty , nothing to do.\r\n");
		    printMsg(msg);
		}
    }
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
