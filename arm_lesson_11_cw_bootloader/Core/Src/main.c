/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
	Это тестовый вариант загрузчика. Устройство читает три строки формата ihex
	из массива. Первая содержит расширенный адрес, вторая - 4 слова(16 байт)данных,
	третья - признак конца файла. Прочитанные данные записываются в флэш память по считанному
	адресу предварительно стирается нужный сектор в флэш памяти (в данном случае сектор 3 )
	При удачном стирании сектора флэш и при корректном значении контрольной суммы в строке с
	данными устройство мигает светодиодом.

 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef struct {

	uint32_t extended_adress; 	//дополнительный адрес (старшие 16 бит адреса)
	uint8_t size_data;			//размер данных
	uint8_t type_data;			//тип данных
	uint8_t check_sum;			//контрольная сумма
	uint16_t address_data;		//младшие 16 бит адреса
	uint32_t program_data;		//слово(32 бит) которое пишется во флеш

} ihexData;

ihexData fwIhexData;

uint8_t calculation_check_sum = 0;		// расчитываемая контрольная сумма
uint32_t fwBuff_count = 0;				// счетчик символов в буфере с "прошивкой"
uint32_t i, j;
bool flagFwUpdate = true; 	// флаг признак обновления прошивки
bool flagFwEof = false;		// флаг приизнак конца файла
char tempFwBuff[8] = { 0, };  // временный буфер для считывания символов

// Extended Address  {":020000040800F2"};
// Data 			 {":10C00000E81B0020D5000008451F0008451D00085A"};
// End Of File 		 {":00000001FF"};

char fwBuff[] = {":020000040800F2:10C00000E81B0020D5000008451F0008451D00085A:00000001FF"};// буфер с "прошивкой"
/*
 в реальном устройстве прошивка будет находится на карте памяти или загружаться
 построчно через какой либо интерфейс(UART,I2C,CAN) или по воздуху
 */
uint32_t fwToFlashBuff[4] = { 0, }; // буфер с данными из каждой строки ihex (максимум 4 слова )

uint32_t flash_address ;  // адрес по которому будут записываться данные в флэш память
uint32_t flash_page_error;  // адрес страницы флэш памяти которую не удалось стереть

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void Char_To_Hex(uint8_t *buff, uint8_t count);
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
  /* USER CODE BEGIN 2 */



	if(flagFwUpdate){	//если нужно обновиться

		HAL_FLASH_Unlock();	//разблокируем флэш память перед стиранием

		  static FLASH_EraseInitTypeDef EraseInitStruct;
		  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS;
		  EraseInitStruct.Sector = FLASH_SECTOR_3;
		  EraseInitStruct.NbSectors = 1;
		  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

		  if (HAL_FLASHEx_Erase(&EraseInitStruct, &flash_page_error) == HAL_OK)	// если удачно стёрли ...
		     {
			  	// мигнём светодиодом
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				HAL_Delay(100);
		     }
		  HAL_FLASH_Lock(); // заблокируем флэш память


		while (!flagFwEof) {		// пока не конец файла

			if (fwBuff[fwBuff_count] == ':') {		// если найден признак начала строки

				fwBuff_count++;

				for (i = 0; i < 8; i++, fwBuff_count++) {	//считываем 8 первых симовлов с полем длинны,адреса и типа
					tempFwBuff[i] = fwBuff[fwBuff_count];
				}

				Char_To_Hex(tempFwBuff, 8);	// конвертируем символы в хекс

				fwIhexData.size_data = 2 * (tempFwBuff[1] + 16 * tempFwBuff[0]); //находим размер данных
				fwIhexData.address_data = tempFwBuff[5] + 16 * tempFwBuff[4] + 256 * tempFwBuff[3] + 4096 * tempFwBuff[2]; //адрес

				fwIhexData.type_data = tempFwBuff[7] + 16 * tempFwBuff[6]; //тип данных
				calculation_check_sum = fwIhexData.size_data / 2 + (uint8_t) fwIhexData.address_data + (uint8_t) (fwIhexData.address_data >> 8)	+ fwIhexData.type_data; //считаем часть контрольной суммы

				switch (fwIhexData.type_data) {

				case 0x00:	//данные

					while (fwIhexData.size_data > 0) {  //пока не считаем все данные...

						for (i = 0; i < 8; i++, fwBuff_count++) { // считываем 8 символов во временный буфер

							tempFwBuff[i] = fwBuff[fwBuff_count];
						}

						Char_To_Hex(tempFwBuff, 8);	// конвертируем символы в хекс

						for (i = 0; i < 8; i = i + 2) {	//формируем 32-битное слово(4 байта) для записи во флэш

							tempFwBuff[i] <<= 4;
							tempFwBuff[i] = tempFwBuff[i] | tempFwBuff[i + 1];
							fwIhexData.program_data |= tempFwBuff[i] << (i * 4);
						}

						//продолжаем расчёт контрольной суммы
						calculation_check_sum += (uint8_t) fwIhexData.program_data + (uint8_t) (fwIhexData.program_data >> 8) + (uint8_t) (fwIhexData.program_data >> 16) + (uint8_t) (fwIhexData.program_data >> 24);

						fwToFlashBuff[j++] = fwIhexData.program_data; //записываем готовое слово в массив(флэш)

						fwIhexData.size_data -= 8;
						fwIhexData.program_data = 0;
					}

					//досчитываем контрольную сумму
					calculation_check_sum = ~(calculation_check_sum) + 1;


					for (i = 0; i < 2; i++, fwBuff_count++) { // считываем 2 символа контрольной суммы  во временный буфер

						tempFwBuff[i] = fwBuff[fwBuff_count];
					}

					Char_To_Hex(tempFwBuff, 2);		// конвертируем символы в хекс

					fwIhexData.check_sum = tempFwBuff[1] + 16 * tempFwBuff[0];

					if (calculation_check_sum == fwIhexData.check_sum) {	// если контрольная сумма совпадает...

						// мигнём светодиодом
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
						HAL_Delay(100);
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
						HAL_Delay(100);

						flash_address += fwIhexData.address_data;// досчитаем адрес

						// и запишем данные в флэш память
						HAL_FLASH_Unlock();
						for (uint8_t i,j; i < sizeof(fwToFlashBuff);i+=4,j++ )
						{
							HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address+i, fwToFlashBuff[j]);
						}
						 HAL_FLASH_Lock();


					} else {
						// если контрольная сумма не совпадает ...
						//можно вывести сообщение об ошибке и выйти
					}

					calculation_check_sum = 0;		//обнуляем контрольную сумму

					break;

				case 0x01:	//конец файла

					//досчитываем контрольную сумму
					calculation_check_sum = ~(calculation_check_sum) + 1;

					for (i = 0; i < 2; i++, fwBuff_count++) { // считываем 2 символа контрольной суммы  во временный буфер
						tempFwBuff[i] = fwBuff[fwBuff_count];
					}

					Char_To_Hex(tempFwBuff, 4);		// конвертируем символы в хекс
					fwIhexData.check_sum = tempFwBuff[1] + 16 * tempFwBuff[0];

					if (calculation_check_sum == fwIhexData.check_sum) {	//если контрольная сумма совпадает...
						flagFwEof = true;	//поднимаем флаг конца файла
					}
					else{

						// если контрольная сумма не совпадает ...
						//можно вывести сообщение об ошибке и выйти
					}

					fwBuff_count = 0; // обнуляем счетчик символа в буфере с прошивкой
					calculation_check_sum = 0; //обнуляем чек сумму

					break;

				case 0x04:	//дополнительный адрес

					for (i = 0; i < 4; i++, fwBuff_count++) { // считываем 4 символа во временный буфер
						tempFwBuff[i] = fwBuff[fwBuff_count];
					}

					Char_To_Hex(tempFwBuff, 4);		// конвертируем символы в хекс

					fwIhexData.extended_adress = (uint32_t) (tempFwBuff[0] << 28 | tempFwBuff[1] << 24 | tempFwBuff[2] << 20 | tempFwBuff[3] << 16);//считаем адрес

					calculation_check_sum += tempFwBuff[0] + tempFwBuff[1] + tempFwBuff[2] + tempFwBuff[3];
					calculation_check_sum = ~(calculation_check_sum) + 1;

					for (i = 0; i < 2; i++, fwBuff_count++) { //считываем 2 символа контрольной суммы  во временный буфер
						tempFwBuff[i] = fwBuff[fwBuff_count];
					}

					Char_To_Hex(tempFwBuff, 2);		// конвертируем символы в хекс
					fwIhexData.check_sum = tempFwBuff[1] + 16 * tempFwBuff[0];

					if (calculation_check_sum == fwIhexData.check_sum) {	//если контрольная сумма совпадает ...

						flash_address = fwIhexData.extended_adress; // формируем адрес для записи во флэш

						/*здесь ещё можно проверить что расширенный адрес соответствует
						 адресу нужного сектора(или страницы)флэш памяти
						*/
					}
					else {
						// если контрольная сумма не совпадает ...
						//можно вывести сообщение об ошибке и выйти
					}

					calculation_check_sum = 0;		//обнуляем контрольную сумму

					break;

				default:
					break;
				}
			}

		}


	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Char_To_Hex(uint8_t *buff, uint8_t count) {
	uint8_t i;

	for (i = 0; i < count; i++) {
		if (buff[i] <= '9' && buff[i] >= '0') {
			buff[i] -= 0x30;
		} else {
			buff[i] = buff[i] - 0x41 + 10;
		}
	}
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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
