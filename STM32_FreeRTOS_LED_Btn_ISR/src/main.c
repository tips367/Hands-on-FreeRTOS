/**
  ******************************************************************************
  * @file    main.c
  * @author  Tipu khan
  * @version V1.0
  * @date    30-March-2021
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define LED_GREEN_PIN	GPIO_Pin_12
#define LED_RED_PIN		GPIO_Pin_14
#define LED_ORANGE_PIN	GPIO_Pin_13


// function prototypes
static void prvSetupHardware(void);
static void prvSetupUART(void);
static void prvSetupGPIO(void);
void printmsg(char *msg);

// task prototypes
static void ledGreenHandler(void* parameters);
static void ledOrangeHandler(void* parameters);
static void ledRedHandler(void* parameters);

// Task handles
TaskHandle_t ledGreenTaskHandle;
TaskHandle_t ledOrangeTaskHandle;
TaskHandle_t ledRedTaskHandle;
TaskHandle_t volatile nextTaskHandle = NULL;

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

	BaseType_t status;

	// 3. Create Tasks
	status = xTaskCreate(ledGreenHandler, "LED_green_task", 200, NULL, 3, &ledGreenTaskHandle);
	configASSERT(status == pdPASS);

	nextTaskHandle = ledGreenTaskHandle;

	status = xTaskCreate(ledOrangeHandler, "LED_orange_task", 200, NULL, 2, &ledOrangeTaskHandle);
	configASSERT(status == pdPASS);

	status = xTaskCreate(ledRedHandler, "LED_red_task", 200,NULL, 1, &ledRedTaskHandle);
	configASSERT(status == pdPASS);

	// 4. Start the scheduler
	vTaskStartScheduler();

	for(;;);
}

static void ledGreenHandler(void* parameters)
{
	BaseType_t  status;
	while(1)
	{
		SEGGER_SYSVIEW_PrintfTarget("Toggling green LED");
		GPIO_ToggleBits(GPIOD, LED_GREEN_PIN);
		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(1000));
		if(status == pdTRUE)
		{
			portENTER_CRITICAL();
			nextTaskHandle = ledOrangeTaskHandle;
			GPIO_WriteBit(GPIOD, LED_GREEN_PIN , Bit_SET);
			SEGGER_SYSVIEW_PrintfTarget("Delete green LED task");
			portEXIT_CRITICAL();
			vTaskDelete(NULL);
		}
	}
}

static void ledOrangeHandler(void* parameters)
{
	BaseType_t  status;
	while(1)
	{
		SEGGER_SYSVIEW_PrintfTarget("Toggling orange LED");
		GPIO_ToggleBits(GPIOD, LED_ORANGE_PIN);
		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(800));
		if(status == pdTRUE)
		{
			portENTER_CRITICAL();
			nextTaskHandle = ledRedTaskHandle;
			GPIO_WriteBit(GPIOD, LED_ORANGE_PIN , Bit_SET);
			SEGGER_SYSVIEW_PrintfTarget("Delete orange LED task");
			portEXIT_CRITICAL();
			vTaskDelete(NULL);
		}
	}
}

static void ledRedHandler(void* parameters)
{
	BaseType_t  status;
	while(1)
	{
		SEGGER_SYSVIEW_PrintfTarget("Toggling red LED");
		GPIO_ToggleBits(GPIOD, LED_RED_PIN);
		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(400));
		if(status == pdTRUE)
		{
			portENTER_CRITICAL();
			nextTaskHandle = NULL;
			GPIO_WriteBit(GPIOD, LED_RED_PIN , Bit_SET);
			SEGGER_SYSVIEW_PrintfTarget("Delete red LED task");
			portEXIT_CRITICAL();
			vTaskDelete(NULL);
		}
	}
}

void buttonInterruptHandler(void)
{
	BaseType_t pxHigherPriorityTaskWoken;
	pxHigherPriorityTaskWoken = pdFALSE;

	traceISR_ENTER();
	xTaskNotifyFromISR(nextTaskHandle,0,eNoAction,&pxHigherPriorityTaskWoken);
	/* once the ISR exits, the below macro makes higher priority task which got unblocked to resume on the CPU */
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	traceISR_EXIT();
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
	NVIC_SetPriority(EXTI0_IRQn, 6);
	NVIC_EnableIRQ(EXTI0_IRQn);

}

void EXTI0_IRQHandler(void)
{
	// 1. Clear the interrupt pending bit of the EXTI line
	EXTI_ClearITPendingBit(EXTI_Line0);
	buttonInterruptHandler();
}
