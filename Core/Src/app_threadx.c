/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Monitor.h"
#include "Supervisor.h"
#include "ux_api.h"
#include "scrutiny.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MS_TO_TICKS(ms) ((ms) * TX_TIMER_TICKS_PER_SECOND / 1000)



#define INIT_THREAD_STACK_SIZE          1024 * 2
#define SUPERVISOR_THREAD_STACK_SIZE    1024 * 2
#define MONITOR_THREAD_STACK_SIZE       1024

#define INIT_THREAD_PRIO          1
#define SUPERVISOR_THREAD_PRIO    3
#define MONITOR_THREAD_PRIO       8

#define INIT_THREAD_PREEMPTION_THRESHOLD        1
#define SUPERVISOR_THREAD_PREEMPTION_THRESHOLD  3
#define MONITOR_THREAD_PREEMPTION_THRESHOLD     8

#define INIT_THREAD_TIME_SLICE            TX_NO_TIME_SLICE
#define SUPERVISOR_THREAD_TIME_SLICE      TX_NO_TIME_SLICE
#define MONITOR_THREAD_TIME_SLICE         TX_NO_TIME_SLICE

#define INIT_THREAD_START_OPTION          TX_AUTO_START
#define SUPERVISOR_THREAD_START_OPTION    TX_DONT_START
#define MONITOR_THREAD_START_OPTION       TX_DONT_START

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
ULONG init_thread_stack[INIT_THREAD_STACK_SIZE / sizeof(ULONG)];
ULONG supervisor_thread_stack[SUPERVISOR_THREAD_STACK_SIZE / sizeof(ULONG)];
ULONG monitor_thread_stack[MONITOR_THREAD_STACK_SIZE / sizeof(ULONG)];

TX_THREAD init_thread_pointer;
TX_THREAD supervisor_thread_pointer;
TX_THREAD monitor_thread_pointer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID init_thread_entry(ULONG init_input);
VOID supervisor_thread_entry(ULONG init_input);
VOID monitor_thread_entry(ULONG init_input);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  if(tx_thread_create( &init_thread_pointer,
                        "InitThread",
                        init_thread_entry,
                        0x0,
                        init_thread_stack,
                        INIT_THREAD_STACK_SIZE,
                        INIT_THREAD_PRIO,
                        INIT_THREAD_PREEMPTION_THRESHOLD,
                        INIT_THREAD_TIME_SLICE,
                        INIT_THREAD_START_OPTION))        { return TX_THREAD_ERROR; }

  if(tx_thread_create( &supervisor_thread_pointer,
                        "SupervisorThread",
                        supervisor_thread_entry,
                        0x0,
                        supervisor_thread_stack,
                        SUPERVISOR_THREAD_STACK_SIZE,
                        SUPERVISOR_THREAD_PRIO,
                        SUPERVISOR_THREAD_PREEMPTION_THRESHOLD,
                        SUPERVISOR_THREAD_TIME_SLICE,
                        SUPERVISOR_THREAD_START_OPTION))   { return TX_THREAD_ERROR; }

  if(tx_thread_create( &monitor_thread_pointer,
                        "MonitorThread",
                        monitor_thread_entry,
                        0x0,
                        monitor_thread_stack,
                        MONITOR_THREAD_STACK_SIZE,
                        MONITOR_THREAD_PRIO,
                        MONITOR_THREAD_PREEMPTION_THRESHOLD,
                        MONITOR_THREAD_TIME_SLICE,
                        MONITOR_THREAD_START_OPTION))   { return TX_THREAD_ERROR; }

  
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
VOID init_thread_entry(ULONG init_input)
{
    ScrutinyInit();
    SupervisorInit();
    MonitorInit();
    tx_thread_resume(&supervisor_thread_pointer);
    tx_thread_resume(&monitor_thread_pointer);
}

VOID supervisor_thread_entry(ULONG init_input)
{
    while (1)
    {
      SupervisorRun();
      tx_thread_sleep(MS_TO_TICKS(1));
    }
}

VOID monitor_thread_entry(ULONG init_input)
{
    while (1)
    {
      MonitorRun();
      tx_thread_sleep(MS_TO_TICKS(10));
    }
}

/* USER CODE END 1 */
