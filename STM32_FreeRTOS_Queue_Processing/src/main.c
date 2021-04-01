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
#include <stdio.h>
#include "stm32f4xx.h"

#include "main.h"
#include "task.h"
#include "queue.h"

// task handles
TaskHandle_t xTask1_handle = NULL;
TaskHandle_t xTask2_handle = NULL;
TaskHandle_t xTask3_handle = NULL;
TaskHandle_t xTask4_handle = NULL;

// Queue handles
QueueHandle_t commandQueue = NULL;
QueueHandle_t uartWriteQueue = NULL;

// software timer handler
TimerHandle_t ledTimerHandle = NULL;

uint8_t cmdBuffer[20];
uint8_t cmdLen = 0;

//This is the menu
char menu[]={"\
\r\nLED_ON             ----> 1 \
\r\nLED_OFF            ----> 2 \
\r\nLED_TOGGLE         ----> 3 \
\r\nLED_TOGGLE_OFF     ----> 4 \
\r\nLED_READ_STATUS    ----> 5 \
\r\nRTC_PRINT_DATETIME ----> 6 \
\r\nEXIT_APP           ----> 0 \
\r\nType your option here : "};

int main(void)
{
	DWT->CTRL |= (1 << 0); // Enable CYCCNT in DWT_CTRL

	// Reset the RCC clock configuration to the default reset state.
	// HSI ON, PLL OFF, HSE OFF, system clock = 16MHz, cpu_clock = 16MHz
	RCC_DeInit();

	// Update the SystemCoreClock variable
	SystemCoreClockUpdate();

	prvSetupHardware();

	printmsg("\r\nThis is Queue Command Processing Demo\r\n");

	// Start recording
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	BaseType_t status;

	// Create command queue
	commandQueue = xQueueCreate(10, sizeof(APP_CMD_t*));

	// Create uart write queue
	uartWriteQueue = xQueueCreate(10, sizeof(char*));

	if((commandQueue != NULL) && (uartWriteQueue != NULL))
	{
		// Create Tasks
		status = xTaskCreate(vTask1_menu_display, "TASK1-MENU", 200, NULL, 1, &xTask1_handle);
		configASSERT(status == pdPASS);

		status = xTaskCreate(vTask2_cmd_handling, "TASK2-CMD-HANDLING", 200, NULL, 2, &xTask2_handle);
		configASSERT(status == pdPASS);

		status = xTaskCreate(vTask3_cmd_processing, "TASK3-CMD-PROCESS", 200, NULL, 2, &xTask3_handle);
		configASSERT(status == pdPASS);

		status = xTaskCreate(vTask4_uart_write, "TASK4-UART-WRITE", 200, NULL, 2, &xTask4_handle);
		configASSERT(status == pdPASS);

		// Start the scheduler
		vTaskStartScheduler();
	}
	else
	{
		printmsg("Queue creation failed\r\n");
	}

	for(;;);
}

static void vTask1_menu_display(void *params)
{
	char* pData = menu;
	while(1)
	{
		xQueueSend(uartWriteQueue, &pData, portMAX_DELAY);

		// wait here until someone notifies.
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
	}
}

static void vTask2_cmd_handling(void *params)
{
	uint8_t command_code=0;
	APP_CMD_t *new_cmd;
	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		new_cmd = (APP_CMD_t*) pvPortMalloc(sizeof(APP_CMD_t));

		taskENTER_CRITICAL();
		command_code = getCommandCode(cmdBuffer);
		new_cmd->CMD_NUM = command_code;
		getArguments(new_cmd->CMD_ARGS);
		taskEXIT_CRITICAL();

		//send the command to the command queue
		xQueueSend(commandQueue,&new_cmd,portMAX_DELAY);
	}
}

static void vTask3_cmd_processing(void *params)
{
	APP_CMD_t *pNewCmd;
	char taskMsg[50];
	uint32_t toggleDuration = pdMS_TO_TICKS(500);

	while(1)
	{
		xQueueReceive(commandQueue,(void*)&pNewCmd,portMAX_DELAY);

		if(pNewCmd->CMD_NUM == LED_ON_COMMAND)
		{
			turnOnLed();
		}
		else if(pNewCmd->CMD_NUM  == LED_OFF_COMMAND)
		{
			turnOffLed();
		}
		else if(pNewCmd->CMD_NUM  == LED_TOGGLE_COMMAND)
		{
			startLedToggle(toggleDuration);
		}
		else if(pNewCmd->CMD_NUM  == LED_TOGGLE_STOP_COMMAND)
		{
			stopLedToggle();
		}
		else if(pNewCmd->CMD_NUM  == LED_READ_STATUS_COMMAND)
		{
			readLedStatus(taskMsg);
		}
		else if(pNewCmd->CMD_NUM  == RTC_READ_DATE_TIME_COMMAND )
		{
			readRtcInfo(taskMsg);
		}
		else
		{
			printErrorMessage(taskMsg);
		}

		// free the allocated memory for the new command
		vPortFree(pNewCmd);
	}
}

static void vTask4_uart_write(void *params)
{
	char *pData = NULL;
	while(1)
	{
		xQueueReceive(uartWriteQueue,&pData,portMAX_DELAY);
		printmsg(pData);
	}
}

void vApplicationIdleHook(void)
{
	//send the cpu to normal sleep
	__WFI();
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

	// 5. Enable the UART byte reception interrupt in the microcontroller
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	// 6. Set the priority in NVIC for the UART2 interrupt
	NVIC_SetPriority(USART2_IRQn, 5);

	// 7. Enable the UART2 IRQ in the NVIC
	NVIC_EnableIRQ(USART2_IRQn);

	// 8. Enable the UART2 peripheral
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

void USART2_IRQHandler(void)
{
	uint16_t dataByte;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE))
	{
		//a data byte is received from the user
		dataByte = USART_ReceiveData(USART2);
		cmdBuffer[cmdLen++] = (dataByte & 0xFF);
		if(dataByte == '\r')
		{
			// user finished entering the data
			// reset the command_len variable
			cmdLen = 0;

			// Notify the command handling task
			xTaskNotifyFromISR(xTask2_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
			xTaskNotifyFromISR(xTask1_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
		}
	}

	// if the above freeRTOS apis wake up any higher priority task, then yield the processor to the
	// higher priority task which is just woken up.
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t getCommandCode(uint8_t *buffer)
{
	return buffer[0]-48;
}

void getArguments(uint8_t *buffer)
{

}

void turnOnLed(void)
{
	GPIO_WriteBit(GPIOD, LED_ALL_PIN , Bit_SET);
}

void turnOffLed(void)
{
	GPIO_WriteBit(GPIOD, LED_ALL_PIN , Bit_RESET);
}

void ledToggle(TimerHandle_t xTimer)
{
	GPIO_ToggleBits(GPIOD,LED_ALL_PIN);
}

void startLedToggle(uint32_t duration)
{
	if(ledTimerHandle == NULL)
	{
		//1.Create the software timer
		ledTimerHandle = xTimerCreate("LED-TIMER",duration,pdTRUE,NULL,ledToggle);
	}

	//2. Start the software timer
	xTimerStart(ledTimerHandle,portMAX_DELAY);
}

void stopLedToggle(void)
{
	xTimerStop(ledTimerHandle,portMAX_DELAY);
}

void readLedStatus(char *taskMsg)
{
	sprintf(taskMsg , "\r\nLED status is : %d\r\n", GPIO_ReadOutputDataBit(GPIOD,LED_ALL_PIN));
	xQueueSend(uartWriteQueue,&taskMsg,portMAX_DELAY);
}

void readRtcInfo(char *taskMsg)
{
	RTC_TimeTypeDef RTC_time;
	RTC_DateTypeDef RTC_date;
	//read time and date from RTC peripheral of the microcontroller
	RTC_GetTime(RTC_Format_BIN, &RTC_time);
	RTC_GetDate(RTC_Format_BIN, &RTC_date);

	sprintf(taskMsg,"\r\nTime: %02d:%02d:%02d \r\nDate: %02d-%2d-%2d \r\n",RTC_time.RTC_Hours,RTC_time.RTC_Minutes,RTC_time.RTC_Seconds, \
									RTC_date.RTC_Date,RTC_date.RTC_Month,RTC_date.RTC_Year );
	xQueueSend(uartWriteQueue,&taskMsg,portMAX_DELAY);
}

void printErrorMessage(char *taskMsg)
{
	sprintf(taskMsg,"\r\nInvalid command received\r\n");
	xQueueSend(uartWriteQueue,&taskMsg,portMAX_DELAY);
}
