/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Monitor.h"
#include "Supervisor.h"
#include "scrutiny.h"
#include "canopen_app.h"
#include "tusb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Stack sizes in 32-bit words, not bytes: 512 words = 2 KB */
#define SUPERVISOR_TASK_STACK_WORDS   512
#define CANOPEN_TASK_STACK_WORDS      512
#define MONITOR_TASK_STACK_WORDS      256
#define USB_TASK_STACK_WORDS          512
#define SCRUTINY_TASK_STACK_WORDS     512

/* Higher number = higher priority. initTask is 6 (osPriorityRealtime in the .ioc) */
#define SUPERVISOR_TASK_PRIO          5
#define CANOPEN_TASK_PRIO             4
#define MONITOR_TASK_PRIO             3
#define USB_TASK_PRIO                 2
#define SCRUTINY_TASK_PRIO            1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim8;

static StackType_t supervisor_task_stack[SUPERVISOR_TASK_STACK_WORDS];
static StackType_t canopen_task_stack[CANOPEN_TASK_STACK_WORDS];
static StackType_t monitor_task_stack[MONITOR_TASK_STACK_WORDS];
static StackType_t usb_task_stack[USB_TASK_STACK_WORDS];
static StackType_t scrutiny_task_stack[SCRUTINY_TASK_STACK_WORDS];

static StaticTask_t supervisor_task_tcb;
static StaticTask_t canopen_task_tcb;
static StaticTask_t monitor_task_tcb;
static StaticTask_t usb_task_tcb;
static StaticTask_t scrutiny_task_tcb;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void supervisor_task(void *argument);
static void canopen_task(void *argument);
static void monitor_task(void *argument);
static void usb_task(void *argument);
static void scrutiny_task(void *argument);
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  RtosFatalError();
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Stack overflow or failed configASSERT. ADC3 and EXTI0 are never masked, so the FOC would
   keep running without its tasks: cut the power stage first. */
void RtosFatalError(void)
{
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
  HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
  Error_Handler();
}

/* Highest priority: the other tasks start only after this one deletes itself */
void InitTask(void const *argument)
{
  ScrutinyInit();
  SupervisorInit();
  MonitorInit();

  xTaskCreateStatic(supervisor_task, "Supervisor", SUPERVISOR_TASK_STACK_WORDS, NULL,
                    SUPERVISOR_TASK_PRIO, supervisor_task_stack, &supervisor_task_tcb);
  xTaskCreateStatic(canopen_task, "CANopen", CANOPEN_TASK_STACK_WORDS, NULL,
                    CANOPEN_TASK_PRIO, canopen_task_stack, &canopen_task_tcb);
  xTaskCreateStatic(monitor_task, "Monitor", MONITOR_TASK_STACK_WORDS, NULL,
                    MONITOR_TASK_PRIO, monitor_task_stack, &monitor_task_tcb);
  xTaskCreateStatic(usb_task, "USB", USB_TASK_STACK_WORDS, NULL,
                    USB_TASK_PRIO, usb_task_stack, &usb_task_tcb);
  xTaskCreateStatic(scrutiny_task, "Scrutiny", SCRUTINY_TASK_STACK_WORDS, NULL,
                    SCRUTINY_TASK_PRIO, scrutiny_task_stack, &scrutiny_task_tcb);

  vTaskDelete(NULL);
}

static void supervisor_task(void *argument)
{
  while (1)
  {
    SupervisorRun();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void canopen_task(void *argument)
{
  CanopenInit();
  while (1)
  {
    CanopenRun();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void monitor_task(void *argument)
{
  while (1)
  {
    MonitorRun();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void usb_task(void *argument)
{
  const tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL };

  MX_USB_PCD_Init();  /* USB clock and NVIC setup from CubeMX */
  if (!tusb_init(0, &dev_init))
  {
    Error_Handler();
  }

  while (1)
  {
    tud_task();  /* sleeps until a USB event arrives */
  }
}

static void scrutiny_task(void *argument)
{
  while (1)
  {
    ScrutinyRun();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
/* USER CODE END Application */

