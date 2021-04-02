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
#include "semphr.h"

// function prototypes
static void prvSetupHardware(void);
static void prvSetupUART(void);
static void prvSetupGPIO(void);
void printMsg(char *msg);

// task prototypes
void vTask1(void *params);
void vTask2(void *params);

// binary semaphore handle
xSemaphoreHandle xBinarySemaphore;

int main(void)
{
	// Reset the RCC clock configuration to the default reset state.
	// HSI ON, PLL OFF, HSE OFF, system clock = 16MHz, cpu_clock = 16MHz
	RCC_DeInit();

	// Update the SystemCoreClock variable
	SystemCoreClockUpdate();

	prvSetupHardware();

	vSemaphoreCreateBinary(xBinarySemaphore);

	if(xBinarySemaphore != NULL)
	{
		// Create task-1
		xTaskCreate(vTask1, "Task-1", 500, NULL, 1, NULL);

		// Create task-2
		xTaskCreate(vTask2, "Task-2", 500, NULL, 1, NULL);

		//makes semaphore available for the first time
		xSemaphoreGive(xBinarySemaphore);

		// Start the scheduler
		vTaskStartScheduler();
	}

	for(;;);
}

void vTask1(void *params)
{
	while(1)
	{
		// before printing, lets own the semaphore
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

		printMsg("Task 1 is running\r\n");

		// give the semaphore here
		xSemaphoreGive(xBinarySemaphore);

		// Now this task will be blocked for 500 ms
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void vTask2(void *params)
{
	while(1)
	{
		// before printing, lets own the semaphore
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

		printMsg("Task 2 is running\r\n");

		// give the semaphore here
		xSemaphoreGive(xBinarySemaphore);

		// Now this task will be blocked for 500 ms
		vTaskDelay(pdMS_TO_TICKS(500));
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
