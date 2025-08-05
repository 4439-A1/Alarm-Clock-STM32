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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include <stdio.h>
#include <string.h>  // For strtok, strstr, strlen
#include <stdlib.h>  // For atoi, atof
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_BUFFER_LEN 128

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t gps_byte;

char gps_buffer[GPS_BUFFER_LEN];
int gps_index = 0;

// Parsed values
int hours = 0, minutes = 0, seconds = 0;
int day = 0, month = 0, year = 0;
double latitude = 0.0, longitude = 0.0;
char lat_dir = 'N', lon_dir = 'E';
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
double convertToDecimalDegrees(char *nmea_coord, char dir);
void process_gprmc(char *sentence);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_UART_Receive(&huart1, &gps_byte, 1, 10) == HAL_OK) {
		  if (gps_index < GPS_BUFFER_LEN - 1) {
			  gps_buffer[gps_index++] = gps_byte;
		  }

		  if (gps_byte == '\n') {
			  gps_buffer[gps_index] = '\0';

			  if (strncmp(gps_buffer, "$GPRMC", 6) == 0) {
				  process_gprmc(gps_buffer);
			  }

			  gps_index = 0;
		  }
	  }


//	  // Basic code for reading push buttons via GPIO
//	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) {
//	      // Button is pressed (logic 0 because pull-up)
//	      lcd_put_cursor(0, 0);
//	      lcd_send_string("BTN: Pressed     ");
//	  } else {
//	      lcd_put_cursor(0, 0);
//	      lcd_send_string("BTN: Released    ");
//	  }

//	  // Basic code for writing to the LCD
//	  lcd_put_cursor(0,0);
//	  lcd_send_string("Hello World");
//	  lcd_put_cursor(1,0);
//	  lcd_send_string("From STM32NUCLEO");
//	  HAL_Delay(50);
  }

//	  //
//	  if (HAL_UART_Receive(&huart1, &gps_byte, 1, HAL_MAX_DELAY) == HAL_OK) {
//		  printf("%c", gps_byte);
//	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void process_gprmc(char *sentence){
	int field = 0;
	char *p = sentence;
	char token[32];
	int i = 0;

	char time_str[16] = {0};
	char date_str[16] = {0};

	char lcd_line1[17];  // Assuming 16x2 LCD
	char lcd_line2[17];

	while (*p){
		if (*p == ',' || *p == '\n' || *p == '\r') {
			token[i] = '\0';
			field++;

//			printf("Field %d: %s\r\n", field, token);

			switch (field) {
				case 2:
					strncpy(time_str, token, sizeof(time_str)-1);
					time_str[sizeof(time_str)-1] = '\0';
					break;
				case 10:
					strncpy(date_str, token, sizeof(date_str)-1);
					date_str[sizeof(date_str)-1] = '\0';
					break;
				default: break;
			}

			i = 0;
		} else {
			if (i < sizeof(token) - 1){
				token[i++] = *p;
			}
		}
		// increment pointer
		p++;
	}

	// Parse and print formatted date and time
	if (strlen(time_str) >= 6) {
		int hh = (time_str[0] - '0') * 10 + (time_str[1] - '0');
		int mm = (time_str[2] - '0') * 10 + (time_str[3] - '0');
		int ss = (time_str[4] - '0') * 10 + (time_str[5] - '0');
		printf("Time: %02d:%02d:%02d\r\n", hh, mm, ss);

		sprintf(lcd_line1, "Time: %02d:%02d:%02d", hh, mm, ss);

//		lcd_put_cursor(0, 0);
//		lcd_send_string("                ");  // 16 spaces
		lcd_put_cursor(0,0);
		lcd_send_string(lcd_line1);
	} else {
		printf("Time: Invalid\r\n");

//		lcd_put_cursor(0,0);
//		lcd_send_string("                ");  // 16 spaces
		lcd_put_cursor(0,0);
		lcd_send_string("Time: Invalid");
	}

	if (strlen(date_str) == 6) {
		int dd = (date_str[0] - '0') * 10 + (date_str[1] - '0');
		int mm = (date_str[2] - '0') * 10 + (date_str[3] - '0');
		int yy = (date_str[4] - '0') * 10 + (date_str[5] - '0');
		printf("Date: %02d/%02d/%02d\r\n", mm, dd, yy);

		sprintf(lcd_line2, "Date: %02d/%02d/%02d", mm, dd, yy);

//		lcd_put_cursor(1, 0);
//		lcd_send_string("                ");  // 16 spaces
		lcd_put_cursor(1,0);
		lcd_send_string(lcd_line2);
	} else {
		printf("Date: Invalid\r\n");

//		lcd_put_cursor(1,0);
//		lcd_send_string("                ");  // 16 spaces
		lcd_put_cursor(1,0);
		lcd_send_string("Date: Invalid");
	}

//	printf("Time: %s\r\n", time_str);
//	printf("Date: %s\r\n", date_str);

	printf("----------\r\n");
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
