/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t RX1_Char = 0x00;
uint8_t a=0,b=0,c=0,d=0,e=0,f=0,g=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance==USART1) {
       HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
   }
}


#define PCA9685_ADDRESS 0x80
// Datasheet link --> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PCA9685_MODE1         0x0         // as in the datasheet page no 10/52
#define PCA9685_PRE_SCALE     0xFE        // as in the datasheet page no 13/52
#define PCA9685_LED0_ON_L     0x6         // as in the datasheet page no 10/52
#define PCA9685_MODE1_SLEEP_BIT      4    // as in the datasheet page no 14/52
#define PCA9685_MODE1_AI_BIT         5    // as in the datasheet page no 14/52
#define PCA9685_MODE1_RESTART_BIT    7    // as in the datasheet page no 14/52
void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
 uint8_t readValue;
 // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
 HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
 if (Value == 0) readValue &= ~(1 << Bit);
 else readValue |= (1 << Bit);
 HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
 HAL_Delay(1);
}
void PCA9685_SetPWMFrequency(uint16_t frequency)
{
 uint8_t prescale;
 if(frequency >= 1526) prescale = 0x03;
 else if(frequency <= 24) prescale = 0xFF;
 //  internal 25 MHz oscillator as in the datasheet page no 1/52
 else prescale = 25000000 / (4096 * frequency);
 // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
 PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
 HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
 PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
 PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}
void PCA9685_Init(uint16_t frequency)
{
 PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo
 PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
 uint8_t registerAddress;
 uint8_t pwm[4];
 registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
 // See example 1 in the datasheet page no 18/52
 pwm[0] = OnTime & 0xFF;
 pwm[1] = OnTime>>8;
 pwm[2] = OffTime & 0xFF;
 pwm[3] = OffTime>>8;
 HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}
void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
 float Value;
 // 50 Hz servo then 4095 Value --> 20 milliseconds
 // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
 Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
 PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}


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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
  PCA9685_Init(50); // 50Hz for servo

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  PCA9685_SetServoAngle(3, 0);
  PCA9685_SetServoAngle(4, 0);
  PCA9685_SetServoAngle(6, 0);
  PCA9685_SetServoAngle(8, 0);
  PCA9685_SetServoAngle(10, 0);
  PCA9685_SetServoAngle(13, 0);
  while (1)
  {
	  if (RX1_Char)
	  {
	  switch (RX1_Char) {
	     case '1':
	          a++;
	          if(a>18){a=18;}//18*10=180 Degree & MG996R MAX Servo Angle is 180
	          PCA9685_SetServoAngle(3,a*10);
	          break;

	     case '2':
	          a--;
	          if(a<0){a=0;}
	          PCA9685_SetServoAngle(3,a*10);
	          break;



	   case '4':
	          b++;
	          if(b>18){b=18;}
	          PCA9685_SetServoAngle(4,b*10);
	          break;

	     case '3':
	          b--;
	          if(b<0){b=0;}
	          PCA9685_SetServoAngle(4,b*10);
	          break;

	   case '5':
	          c++;
	          if(c>18){c=18;}
	          PCA9685_SetServoAngle(6,c*10);
	          break;

	     case '6':
	          c--;
	          if(c<0){c=0;}
	          PCA9685_SetServoAngle(6,c*10);
	          break;

	    case '8':
	          d++;
	          if(d>18){d=18;}
	          PCA9685_SetServoAngle(8,d*10);
	          break;

	     case '7':
	          d--;
	          if(d<0){d=0;}
	          PCA9685_SetServoAngle(8,d*10);
	          break;
	   case '9':
	          e++;
	          if(e>18){e=18;}
	          PCA9685_SetServoAngle(10,e*10);
	          break;

	     case '*':
	          e--;
	          if(e<0){e=0;}
	          PCA9685_SetServoAngle(10,e*10);
	          break;


	   case '+':
	          f++;
	          if(f>18){f=18;}
	          PCA9685_SetServoAngle(13,f*3);//Calibrate for My Robotic Hand Gripper
	          break;

	     case '-':
	          f--;
	          if(f<0){f=0;}
	          PCA9685_SetServoAngle(13,f*3);
	          break;

	   }
	  RX1_Char = 0;
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
