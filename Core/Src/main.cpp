/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RUN_EI_SAMPLE 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Callback function declaration
static int get_signal_data(size_t offset, size_t length, float *out_ptr);

// Raw features copied from test sample (Edge Impulse > Model testing)
static float input_buf[] = {
    /* Paste your raw features here! */ 
		-0.6560, 0.0287, 16.1034, -0.7853, 0.1772, 13.7152, -0.6512, -0.0970, 11.5305, -0.7063, -0.3460, 9.6295, -0.6847, -0.4058, 7.5214, -0.3472, -0.2131, 6.6104, -0.0850, 0.1197, 4.6747, 0.1891, 0.3172, 2.9688, 0.4645, 0.4202, 1.2055, 0.6955, 0.9637, -0.6393, 0.8188, 1.0463, -2.1045, 1.1588, 0.6452, -2.8455, 1.3324, 0.3089, -3.7673, 1.4533, 0.8296, -4.5598, 0.9014, 0.8978, -4.7980, 0.3340, 0.9529, -4.2198, -0.3472, 0.6129, -2.4684, -0.7913, 0.3017, 0.1604, -1.4437, -0.0982, 3.3076, -1.7262, -0.3903, 7.0066, -1.6939, -0.2430, 10.8553, -1.6951, 0.1796, 15.0523, -1.8543, 0.7961, 18.6604, -1.9166, 0.6931, 20.4417, -1.8196, 0.1006, 23.6978, -0.7099, -0.5926, 26.4224, -0.0730, -1.3743, 27.7225, -0.8583, -0.9134, 28.3007, -0.6416, -0.1413, 25.9879, -1.5203, 0.5064, 21.8519, -1.7262, 0.8116, 18.1804, -1.7334, 0.7829, 16.1393, -1.8938, 0.9876, 12.3672, -1.6951, 0.5399, 9.5732, -1.3240, 0.1628, 7.1048, -0.8368, -0.0227, 5.3810, -0.2011, 0.1125, 3.6583, 0.1760, 0.4429, 1.4114, 0.1365, 0.3735, -0.6823, 0.7027, 0.2550, -2.0973, 0.8236, 0.0730, -2.7234, 1.1061, 0.1616, -3.2980, 0.9577, 0.0251, -3.7146, 0.8440, 0.1748, -3.9840, 0.3974, 0.1927, -3.4512, 0.0718, 0.4693, -2.0016, -0.1640, 0.3591, -0.3065, -0.7697, 0.4298, 1.3838, -1.4880, 0.0263, 3.9684, -1.6616, -0.1748, 5.3905, -1.5598, 0.2861, 8.4982, -1.6640, 0.7207, 11.9267, -1.8040, 1.4581, 14.7327, -2.2865, 1.6508, 17.6644, -2.2613, 1.9154, 20.6081, -2.0889, 1.5622, 23.2226, -1.0427, 0.5136, 25.6898, -1.3527, 0.6321, 27.3669, -0.6883, 0.3352, 27.8972, -0.2993, 0.4764, 27.2029, -1.2450, 0.5423, 23.1627, -1.6795, 1.2019, 19.0926, -2.3715, 1.7406, 15.0224, -2.0219, 1.5742, 11.8740, -1.5251, 1.0439, 9.5624, -1.1420, 0.6476, 7.8949, -0.9601, 0.5351, 6.1399, -0.6560, 0.5231, 5.1092, -0.2753, 0.7398, 2.8276, -0.2035, 1.0918, 0.2514, 0.1377, 1.0463, -1.3982, 0.3424, 0.8835, -2.8120, 0.7123, 0.3615, -3.2106, 0.9780, 0.2693, -3.2597, 0.9984, 0.2167, -3.7745, 0.9517, 0.2682, -3.8954, 0.8164, 0.4681, -3.7254, 0.6512, 0.7985, -2.6959, 0.5567, 1.2665, -0.9589, -0.0371, 0.7087, 0.9565, -0.4812, -0.1796, 3.8092, -0.5997, -0.6536, 7.1467, -0.7997, -0.3496, 10.8996, -1.5251, 0.0994, 14.5268, -1.8471, 0.2622, 16.7534, -2.4134, 0.5291, 20.4034, -2.5307, 1.2354, 23.6152, -2.1632, 1.8567, 26.4009, -0.6955, 1.1911, 27.8254, -1.2007, 1.1277, 27.6662, -0.2502, 0.5926, 25.0912, -1.1372, -0.0946, 20.9552, -1.4533, 0.4705, 17.0455, -1.4736, 0.6847, 15.3217, -1.3515, 0.7853, 12.2966, -1.3228, -0.0934, 9.8162, -0.6452, -0.3843, 7.7692, -0.5926, -0.2526, 5.8682, -0.1365, 0.3304, 4.2318, 0.1879, 0.6333, 2.7162, 0.5171, 0.4202, 1.2653, 0.8475, 0.1568, -0.4046, 0.8140, -0.0096, -1.0092, 1.5215, 0.3460, -1.6340, 1.5323, 0.4657, -2.2936, 1.8866, 0.4776, -2.3056, 1.8304, 0.8775, -2.5642, 1.5826, 1.0870, -2.2913, 1.0451, 1.6652, -1.5263, 0.4262, 2.2853, -0.2897, -0.1736, 2.1261, 1.9465, -1.1121, 1.3623, 3.1364, -1.4724, 0.7195, 6.1962, -1.6496, 0.2466, 10.1131, -1.2558, 0.4202, 14.5472, -1.2869, 0.0383, 17.1784, -1.4964, -0.2933, 19.1620, -1.0750, -0.3220, 20.3076, -1.6544, 0.6153, 20.1735, 0.9505, -0.6560, 19.4242, -0.0515, -0.1401, 18.0930, 0.1413, -0.0587, 16.4984, -0.0024, -0.2933, 14.0552, -0.0658, 0.0587, 11.6909, -0.7398, 1.1157, 9.8701
};

char test_str[] = "Test F429\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_CRC_Init(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __cplusplus
 extern "C" {
#endif
int __io_putchar(int ch)
{
	 HAL_UART_Transmit(&huart1,(uint8_t*)&ch, 1, 0xFFFF);
	 return ch;
}
#ifdef __cplusplus
}
#endif
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
  MX_CRC_Init();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  //RetargetInit(&huart1);

  /* USER CODE BEGIN 2 */
#if RUN_EI_SAMPLE
  printf("EI: run_classifier\r\n");
  signal_t signal;            // Wrapper for raw input buffer
    ei_impulse_result_t result; // Used to store inference output
    EI_IMPULSE_ERROR res;       // Return code from inference

    // Calculate the length of the buffer
    size_t buf_len = sizeof(input_buf) / sizeof(input_buf[0]);

    // Make sure that the length of the buffer matches expected input length
    if (buf_len != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        printf("ERROR: The size of the input buffer is not correct.\r\n");
        printf("Expected %d items, but got %d\r\n", 
                EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, 
                (int)buf_len);
        return 1;
    }

    // Assign callback function to fill buffer used for preprocessing/inference
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    // Perform DSP pre-processing and inference
    res = run_classifier(&signal, &result, false);

    // Print return code and how long it took to perform inference
    printf("run_classifier returned: %d\r\n", res);
    printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n", 
            result.timing.dsp, 
            result.timing.classification, 
            result.timing.anomaly);

    // Print the prediction results (object detection)
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n", 
                bb.label, 
                bb.value, 
                bb.x, 
                bb.y, 
                bb.width, 
                bb.height);
    }

    // Print the prediction results (classification)
#else
    printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        printf("  %s: ", ei_classifier_inferencing_categories[i]);
        printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
    //return 0;
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  printf("Predictions:\r\n");
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart1, (uint8_t*)test_str, strlen(test_str), 0xffffff); // send message via UART
	  HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
// Callback: fill a section of the out_ptr buffer when requested
static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (input_buf + offset)[i];
    }

    return EIDSP_OK;
}

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string, fmt, argp)) // build string
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void ei_printf(const char *format, ...)
{
    va_list myargs;
    va_start(myargs, format);
    vprint(format, myargs);
    va_end(myargs);
}
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

