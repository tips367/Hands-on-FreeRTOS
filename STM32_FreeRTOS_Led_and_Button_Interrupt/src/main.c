/**
  ******************************************************************************
  * @file    main.c
  * @author  Tipu khan
  * @version V1.0
  * @date    27-March-2021
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define NOT_PRESSED 0
#define PRESSED		1


// function prototypes
static void prvSetupHardware(void);
static void prvSetupUART(void);
static void prvSetupGPIO(void);
void printmsg(char *msg);

// task prototypes
void ledTaskHandler(void *params);
void buttonHandler(void *params);

// global space for some variable
uint8_t buttonStatusFlag = NOT_PRESSED;

int main(void)
{
	DWT->CTRL |= (1 << 0); // Enable CYCCNT in DWT_CTRL

	// 1. Reset the RCC clock configuration to the default reset state.
	// HSI ON, PLL OFF, HSE OFF, system clock = 16MHz, cpu_clock = 16MHz
	RCC_DeInit();

	// 2. Update the SystemCoreClock variable
	SystemCoreClockUpdate();

	prvSetupHardware();

	// Start recording
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	// 3. Create Led task
	xTaskCreate(ledTaskHandler, "LED-TASK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// 4. Start the scheduler
	vTaskStartScheduler();

	for(;;);
}

void ledTaskHandler(void *params)
{
	while(1)
	{
		if(buttonStatusFlag == PRESSED)
		{
			// Turn on the LEDs
			GPIO_WriteBit(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 , Bit_SET);
		}
		else
		{
			// Turn off the LEDs
			GPIO_WriteBit(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, Bit_RESET);
		}
	}
}

void buttonHandler(void *params)
{
	buttonStatusFlag ^= 1;
}

static void prvSetupHardware(void)
{
	// setup button and LED
	prvSetupGPIO();
	//setup UART2
	prvSetupUART();
}

void printmsg(char *msg)
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
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // PA3

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

	// Interrupt configuration for the user button (PA0)

	// 1. System configuration for EXTI line (SYSCFG settings)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	// 2. EXTI Line configuration
	EXTI_InitTypeDef extiInit;
	extiInit.EXTI_Line = EXTI_Line0;
	extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
	extiInit.EXTI_Trigger = EXTI_Trigger_Falling;
	extiInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&extiInit);

	// 3. NVIC settings (IRQ settings for the selected EXTI line)
	NVIC_SetPriority(EXTI0_IRQn, 5);
	NVIC_EnableIRQ(EXTI0_IRQn);

}

void EXTI0_IRQHandler(void)
{
	traceISR_ENTER();
	// 1. Clear the interrupt pending bit of the EXTI line
	EXTI_ClearITPendingBit(EXTI_Line0);
	buttonHandler(NULL);
	traceISR_EXIT();
}
