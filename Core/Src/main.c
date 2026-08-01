/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "math_ops.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG(rad) 	((rad) * 57.29577951308232f)  // 180/π
#define DEG_TO_RAD(deg) 	((deg) * 0.017453292519943f)  // π/180

#define RPM_TO_RADS(rpm)	((rpm) * 0.1047197551f)				// 2π/60
#define RADS_TO_RPM(rads)	((rads)* 9.549296586f)				// 60/2π

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* can setup*/
CANTxMessage can_tx;
CANRxMessage can_rx;
JointCommand_t joint_cmd;
JointState_t joint_state;

JointRobot_t motorID_1;
JointRobot_t motorID_2;
JointRobot_t motorID_3;
JointRobot_t motorID_4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void StartDefaultTask(void *argument);
void Task1(void *argument);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  /*CAN setup   */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  Task1Handle = osThreadNew(Task1, NULL, &Task1_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2)
  {
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) 
    {
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data) == HAL_OK) 
      {
        switch (can_rx.rx_header.StdId){
          case 0x02:
            Master_Unpack_State_2(&can_rx, &motorID_1.state);
            break;
          
          case 0x03:
            Master_Unpack_State_2(&can_rx, &motorID_2.state);
            break;
            
          case 0x04:
            Master_Unpack_State_2(&can_rx, &motorID_3.state);
            break;

          case 0x05:
            Master_Unpack_State_2(&can_rx, &motorID_4.state);
            break;
          
          default:
            break;
        }
      }
    }
    
  }
}

void Task1(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t TxMailbox;
  osDelay(1000);

  /* Infinite loop */
  for(;;)
  {

    DataCmdJoint(&motorID_3.cmd, DEG_TO_RAD(0),RPM_TO_RADS(10),0);
    MasterSendData_2(&hcan2, &can_tx, &motorID_3.cmd);
    osDelay(1000); 
    DataCmdJoint(&motorID_3.cmd, DEG_TO_RAD(180),RPM_TO_RADS(-10),0);
    MasterSendData_2(&hcan2, &can_tx, &motorID_3.cmd);
    osDelay(1000);    
    DataCmdJoint(&motorID_3.cmd, DEG_TO_RAD(360),RPM_TO_RADS(50),0);
    MasterSendData_2(&hcan2, &can_tx, &motorID_3.cmd);
    osDelay(1000);

    DataCmdJoint(&motorID_3.cmd, DEG_TO_RAD(0),RPM_TO_RADS(-50),0);
    MasterSendData_2(&hcan2, &can_tx, &motorID_3.cmd);
    osDelay(1000); 
    DataCmdJoint(&motorID_3.cmd, DEG_TO_RAD(180),RPM_TO_RADS(100),0);
    MasterSendData_2(&hcan2, &can_tx, &motorID_3.cmd);
    osDelay(1000);    
    DataCmdJoint(&motorID_3.cmd, DEG_TO_RAD(360),RPM_TO_RADS(-100),0);
    MasterSendData_2(&hcan2, &can_tx, &motorID_3.cmd);
    osDelay(1000);

    
  }
  

  /* USER CODE END 5 */
}

void testCan(){
  // uint32_t TxMailbox;

  // DataCmdJoint(&motorID_1.cmd, DEG_TO_RAD(0),RPM_TO_RADS(10),0);
  // MasterSendData(&hcan2, &can_tx, &motorID_1.cmd);
  // osDelay(500); 
  // DataCmdJoint(&motorID_1.cmd, DEG_TO_RAD(180),RPM_TO_RADS(10),0);
  // MasterSendData(&hcan2, &can_tx, &motorID_1.cmd);
  // osDelay(500);    
  // DataCmdJoint(&motorID_1.cmd, DEG_TO_RAD(360),RPM_TO_RADS(10),0);
  // MasterSendData(&hcan2, &can_tx, &motorID_1.cmd);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t TxMailbox;


  motorID_1.cmd.can_id = 0x02;
  motorID_2.cmd.can_id = 0x03;
  motorID_3.cmd.can_id = 0x04;
  motorID_4.cmd.can_id = 0x05;
  
  /* Infinite loop */
  for(;;)
  {
    
    DataCmdJoint(&motorID_2.cmd, motorID_1.state.p_act, motorID_1.state.v_act, 0);
    MasterSendData_2(&hcan2, &can_tx, &motorID_2.cmd);

    DataCmdJoint(&motorID_1.cmd, motorID_2.state.p_act, motorID_2.state.v_act, 0);
    MasterSendData_2(&hcan2, &can_tx, &motorID_1.cmd);



    osDelay(10);
  }
  

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
