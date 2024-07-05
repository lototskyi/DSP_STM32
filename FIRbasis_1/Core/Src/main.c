/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#define ARM_MATH_CM4

#include "arm_math.h"
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
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

uint16_t sampleIn, sampleOut, sampleOld;
float dataIn;
uint8_t state, trigData;
float dataOut, dataInArr[1030], dataOutArr[1030];
uint32_t usRefCnt, usCnt;

q15_t dataInArrayQ15[1030], dataOutArrayQ15[1030];

float32_t aFIR_F32_Coeffs[28] = {
	-0.001821241467076479,-0.001523620707748456,0.0001228441101925434,0.003820799959061135,
	0.008072184266199139,0.008291788641077663,-0.0004281934827583651,-0.01776708438165226,
	-0.03413899183390525,-0.03284191918908606,0.0008089579223558705,0.0682453749765863,
	0.152190845630833,0.2222204654828175,0.2494955801462077,0.2222204654828175,
	0.152190845630833,0.06824537497658631,0.0008089579223558706,-0.03284191918908606,
	-0.03413899183390526,-0.01776708438165227,-0.0004281934827583656,0.008291788641077666,
	0.008072184266199142,0.003820799959061135,0.0001228441101925433,-0.001523620707748457
};
//float32_t aFIR_F32_Coeffs[28] = {
//	0.001814565325827099,0.001518035557597149,-0.000122393799431381,-0.003806794018238107,
//	-0.008042593987630161,-0.008261393356773764,0.0004266238500519741,0.01770195542041025,
//	0.03401384822405738,0.03272153027011476,-0.0008059925180137861,-0.06799520729086367,
//	-0.1516329582772549,-0.2214058699210067,0.7496423156872672,-0.2214058699210067,
//	-0.1516329582772549,-0.06799520729086368,-0.0008059925180137862,0.03272153027011476,
//	0.03401384822405739,0.01770195542041026,0.0004266238500519746,-0.008261393356773767,
//	-0.008042593987630164,-0.003806794018238108,-0.0001223937994313809,0.00151803555759715
//};

float32_t firStateF32[1024 + 28 - 1];
q15_t firStateQ15[1024 + 28 - 1];

q15_t aFIR_Q15_Coeffs[28];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
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

  arm_fir_instance_f32 FIR_F32_Struct;
  arm_fir_instance_q15 FIR_Q15_Struct;


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
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  sampleIn = 0;
  sampleOld = 0;
  sampleOut = 1;
  dataIn = 0;
  dataOut = 0;
  trigData = 0;

  memset(dataInArr, 0, sizeof(dataInArr));
  memset(dataOutArr, 0, sizeof(dataOutArr));

//  arm_fir_init_f32(&FIR_F32_Struct, 28, (float *)&aFIR_F32_Coeffs[0], &firStateF32[0], 1024);

  arm_float_to_q15((float32_t *)&aFIR_F32_Coeffs[0], (q15_t *)&aFIR_Q15_Coeffs[0] , 28);

  arm_fir_init_q15(&FIR_Q15_Struct, 28, &aFIR_Q15_Coeffs[0], &firStateQ15[0], 1024);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	switch(state) {
	  //Copy each sample to input array
		case 0:
			if (sampleIn != sampleOld) {
				dataInArr[sampleIn - 1] = dataIn;
				sampleOld++;
			}

			if (sampleOld == 1026) {
				sampleOld = 0;
				sampleOut = 1;
				state = 1;
				arm_float_to_q15(dataInArr, dataInArrayQ15, 1024);
			}

		    break;
		//Copy data input arr to data output arr
		case 1:
			//memcpy(dataOutArr, dataInArr, sizeof(dataInArr));
			usRefCnt = __HAL_TIM_GET_COUNTER(&htim5);
//			arm_fir_f32(&FIR_F32_Struct, dataInArr, dataOutArr, 1024);

			arm_fir_q15(&FIR_Q15_Struct, dataInArrayQ15, dataOutArrayQ15, 1024);
			usCnt = __HAL_TIM_GET_COUNTER(&htim5) - usRefCnt;

			arm_q15_to_float(dataOutArrayQ15, dataOutArr, 1024);

			state = 2;
			break;
		//With some proper sampling time, copy each output array sample to data out variable
		case 2:
			dataOut = dataOutArr[sampleOut - 1];
			HAL_Delay(25);
			trigData = 1;
			HAL_Delay(25);
			trigData = 0;
			sampleOut++;

			if (sampleOut == 1026) {
				sampleOut = 1;
				state = 3;
			}
			break;
		case 3:

			break;

	}
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
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  HAL_TIM_Base_Start(&htim5);
  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
