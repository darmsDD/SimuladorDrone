/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_base_functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern UART_HandleTypeDef hlpuart1;

/* USER CODE END Variables */
/* Definitions for microROSTask */
osThreadId_t microROSTaskHandle;
const osThreadAttr_t microROSTask_attributes = {
  .name = "microROSTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 3000 * 4
};
/* Definitions for EscreverSetpoin */
osThreadId_t EscreverSetpoinHandle;
const osThreadAttr_t EscreverSetpoin_attributes = {
  .name = "EscreverSetpoin",
  .priority = (osPriority_t) osPriorityBelowNormal5,
  .stack_size = 500 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//bool cubemx_transport_open(struct uxrCustomTransport * transport);
//bool cubemx_transport_close(struct uxrCustomTransport * transport);
//size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
//size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
//
//void * microros_allocate(size_t size, void * state);
//void microros_deallocate(void * pointer, void * state);
//void * microros_reallocate(void * pointer, size_t size, void * state);
//void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

//extern int clock_gettime( int clock_id, struct timespec * tp );
extern void UTILS_NanosecondsToTimespec( int64_t llSource, struct timespec * const pxDestination );
/* USER CODE END FunctionPrototypes */

void microROSTaskFunction(void *argument);
void StartEscreverSetpoint(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of microROSTask */
  microROSTaskHandle = osThreadNew(microROSTaskFunction, NULL, &microROSTask_attributes);

  /* creation of EscreverSetpoin */
  EscreverSetpoinHandle = osThreadNew(StartEscreverSetpoint, NULL, &EscreverSetpoin_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_microROSTaskFunction */
/**
  * @brief  Function implementing the microROSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_microROSTaskFunction */
void microROSTaskFunction(void *argument)
{
  /* USER CODE BEGIN microROSTaskFunction */
	vMicrorosConfiguration();
	vCreateNode();
	 //time sync
	if( rmw_uros_sync_session(1000) != RMW_RET_OK)
		  printf("Error on time sync (line %d)\n", __LINE__);

	vCreatePublisher();
	vCreateSubscriber();
	vCreateExecutor();
	// Run executor
	rclc_executor_spin(&executor);
	/* Infinite loop */
	for(;;)
	{
	  osDelay(1);
	}
  /* USER CODE END microROSTaskFunction */
}

/* USER CODE BEGIN Header_StartEscreverSetpoint */
/**
* @brief Function implementing the EscreverSetpoin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEscreverSetpoint */
void StartEscreverSetpoint(void *argument)
{
  /* USER CODE BEGIN StartEscreverSetpoint */
  /* Infinite loop */
	float a_velocity[] = {500,500,500,500};
	for(;;)
	{
		osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);
		vSetActuatorMsg(a_velocity);
		HAL_GPIO_TogglePin(LD2_GPIO_Port , LD2_Pin);
		osDelay(1);
	}
  /* USER CODE END StartEscreverSetpoint */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

