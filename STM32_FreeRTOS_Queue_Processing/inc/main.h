/**
  ******************************************************************************
  * @file    main.h
  * @author  Tipu khan
  * @version V1.0
  * @date    02-April-2021
  * @brief   Default main function.
  ******************************************************************************
*/

#ifndef MAIN_H
#define MAIN_H

#include "FreeRTOS.h"
#include "timers.h"

// defines for LED pins
#define LED_GREEN_PIN	GPIO_Pin_12
#define LED_ORANGE_PIN	GPIO_Pin_13
#define LED_RED_PIN		GPIO_Pin_14
#define LED_BLUE_PIN	GPIO_Pin_15
#define LED_ALL_PIN		LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN

// define for commands
#define LED_ON_COMMAND 					1
#define LED_OFF_COMMAND 				2
#define LED_TOGGLE_COMMAND 				3
#define LED_TOGGLE_STOP_COMMAND 		4
#define LED_READ_STATUS_COMMAND 		5
#define RTC_READ_DATE_TIME_COMMAND		6


// function prototypes
static void prvSetupHardware(void);
static void prvSetupUART(void);
static void prvSetupGPIO(void);
void printmsg(char *msg);
void getArguments(uint8_t *buffer);
uint8_t getCommandCode(uint8_t *buffer);

// prototypes command helper functions
void turnOnLed(void);
void turnOffLed(void);
void startLedToggle(uint32_t duration);
void stopLedToggle(void);
void readLedStatus(char *taskMsg);
void readRtcInfo(char *taskMsg);
void printErrorMessage(char *taskMsg);

// Software timer callback function prototype
void ledToggle(TimerHandle_t xTimer);

// task prototypes
static void vTask1_menu_display(void *params);
static void vTask2_cmd_handling(void *params);
static void vTask3_cmd_processing(void *params);
static void vTask4_uart_write(void *params);

// command structure
typedef struct APP_CMD
{
	uint8_t CMD_NUM;
	uint8_t CMD_ARGS[10];
}APP_CMD_t;

#endif /* MAIN_H */
