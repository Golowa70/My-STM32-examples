/* USER CODE BEGIN Header */
/**
  ******************************************************************************
	Test program for loading images in XBM format via UART
	and save to flash memory W25Q.To display uses the ST7565
	display with the U8G2 library.

	Тестовая программа для загрузки через UART картинок в формте XBM
	и сохранение в флэш память W25Q. Для отображения используется дисплей ST7565
	с библиотекой U8G2
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25qxx.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "u8g2.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rxSourceBuff[SOURCE_BUFFER_SIZE];
uint8_t rxdataBuff[DATA_BUFFER_SIZE];
volatile bool flag_uart_data_avalible = false;
volatile bool flag_uart_data_overflow = false;
uint16_t rx_received_len;
uint16_t rx_data_len;
uint16_t flash_sector_address;

u8g2_t u8g2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void UartToFlashWriter(void);
int hex_char_to_bin(char ch);

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8,
		U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
		U8X8_UNUSED void *arg_ptr);
void u8g_port_delay_10us(uint8_t us);
void u8g_port_delay_100ns(uint8_t ns);
void u8g_port_delay_ns(uint8_t ns);

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  W25qxx_Init();
  //W25qxx_EraseChip();
  //W25qxx_EraseBlock(0); // 65536 байт
  //W25qxx_EraseSector(0); // 4096 байт
 // W25qxx_WriteSector(water_20level_bits, 0, 0, 1024);
  HAL_Delay(100);
  //W25qxx_ReadBytes(rxdataBuff, 0, 1024);

  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxSourceBuff, SOURCE_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);


	u8g2_Setup_st7565_nhd_c12864_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8g2_gpio_and_delay_stm32);
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_SetContrast(&u8g2, 250);
	u8g2_ClearDisplay(&u8g2);

	u8g2_SetFont(&u8g2, u8g2_font_courB18_tr);
	u8g2_DrawStr(&u8g2, 20, 30, "Hello!");
	u8g2_SendBuffer(&u8g2);
	HAL_Delay(1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		UartToFlashWriter();

		u8g2_ClearBuffer(&u8g2);
		W25qxx_ReadBytes(rxdataBuff, 0, 1024);
		u8g2_DrawXBMP(&u8g2, 0, 0, 64, 55, rxdataBuff);
		u8g2_SendBuffer(&u8g2);
		HAL_Delay(3000);
		u8g2_ClearBuffer(&u8g2);
		W25qxx_ReadBytes(rxdataBuff, (FLASH_SECTOR_SIZE*1), 1024);
		u8g2_DrawXBMP(&u8g2, 0, 0, 64, 55, rxdataBuff);
		u8g2_SendBuffer(&u8g2);
		HAL_Delay(3000);
		u8g2_ClearBuffer(&u8g2);
		W25qxx_ReadBytes(rxdataBuff, (FLASH_SECTOR_SIZE*2), 1024);
		u8g2_DrawXBMP(&u8g2, 0, 0, 64, 55, rxdataBuff);
		u8g2_SendBuffer(&u8g2);
		HAL_Delay(3000);



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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W25Q_CS_GPIO_Port, W25Q_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : W25Q_CS_Pin */
  GPIO_InitStruct.Pin = W25Q_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W25Q_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RESET_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//колбэк прерывания по завершению приёма данных максимального размера буфера
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
         if(huart == &huart1)


          {
        	 flag_uart_data_overflow = true;

        	 /*
        	  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
        	  rx_received_len = SOURCE_BUFFER_SIZE;
        	  flag_uart_data_avalible = 1;
			  HAL_UART_AbortReceive(&huart1);
			  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
			  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
			  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxSourceBuff, SOURCE_BUFFER_SIZE);
			*/
          }

}
//*************************************************************************

// функция колбэк срабатывания флага свободности UART
//определяет конец приёма при неизвестном размере входных данных
//проверка флага UART_FLAG_IDLE и колбэк добавлены в обработчик прерывания
void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{

	if(huart == &huart1)
	{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		rx_received_len = SOURCE_BUFFER_SIZE - huart->RxXferCount;

		flag_uart_data_avalible = true;
		HAL_UART_AbortReceive(&huart1);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_IT(&huart1, (uint8_t*)rxSourceBuff, SOURCE_BUFFER_SIZE);
	}
}
//**************************************************************************

//функция приёма данных по UART и записи их во флэш W25Q
void UartToFlashWriter(void){

	if (flag_uart_data_avalible) {

		flag_uart_data_avalible = false;

		enum state {
			check_sector_index_prefix,
			read_sector_index,
			check_byte_prefix,
			read_byte,
			write_byte,
			next,
			save_data,
			source_overflow,
			data_overflow,
			error,
			end
		};

		enum state step = check_sector_index_prefix;
		uint16_t i = 0; //incoming char counter
		uint8_t data_byte = 0;
		bool flag_flash_address_ok = false;
		bool flag_end = false;
		char buff[64] = { 0, };

		if (flag_uart_data_overflow) {
			step = source_overflow;
			flag_uart_data_overflow = false;
		} else {
			step = check_sector_index_prefix;
		}

		while (!flag_end) {

			switch (step) {

			case check_sector_index_prefix:
				if (rxSourceBuff[i] == '$' && rxSourceBuff[i + 1] == '0'
						&& rxSourceBuff[i + 2] == 'x'
						&& rxSourceBuff[i + 5] == '$') {
					step = read_sector_index;
					i += 3;
					flag_flash_address_ok = true;
				} else {
					step = next;
				}
				break;

			case read_sector_index:
				flash_sector_address = hex_char_to_bin(rxSourceBuff[i]) * 16;
				flash_sector_address += hex_char_to_bin(rxSourceBuff[i + 1]);

				if ((flash_sector_address >= FLASH_SECTOR_RANGE_MIN)
						|| (flash_sector_address <= FLASH_SECTOR_RANGE_MAX)) {
					i += 3;
					step = check_byte_prefix;
				} else {
					sprintf(buff, "\r\n Flash sector address %d not correct.",
							flash_sector_address);
					HAL_UART_Transmit(&huart1, (uint8_t*) buff,
							strlen((char*) buff), 100);
					step = error;
				}
				break;

			case check_byte_prefix:
				if (rxSourceBuff[i] == '0' && rxSourceBuff[i + 1] == 'x') {
					step = read_byte;
					i += 2;
				} else {
					step = next;
				}
				break;

			case read_byte:
				data_byte = hex_char_to_bin(rxSourceBuff[i]) * 16;
				data_byte += hex_char_to_bin(rxSourceBuff[i + 1]);
				step = write_byte;
				i += 2;
				break;

			case write_byte:
				if (rx_data_len <= (DATA_BUFFER_SIZE)) {
					rxdataBuff[rx_data_len++] = data_byte;
					HAL_UART_Transmit(&huart1, (uint8_t*) &data_byte, 1, 100);
					step = next;
				} else {
					step = data_overflow;
				}
				break;

			case next:
				i++;
				if (i > rx_received_len) {
					if (flag_flash_address_ok == true) {
						step = save_data;
					} else {
						sprintf(buff, "\r\n Incorrect format.");
						HAL_UART_Transmit(&huart1, (uint8_t*) buff,
								strlen((char*) buff), 100);
						step = error;
					}
				} else {
					if (flag_flash_address_ok == true)
						step = check_byte_prefix;
					else
						step = check_sector_index_prefix;
				}
				break;

			case save_data:
				W25qxx_EraseSector(flash_sector_address);
				W25qxx_WriteSector(rxdataBuff, flash_sector_address, 0, rx_data_len);
				memset(rxdataBuff, 0, DATA_BUFFER_SIZE);

				sprintf(buff, "\r\n Successful!");
				HAL_UART_Transmit(&huart1, (uint8_t*) buff,
						strlen((char*) buff), 100);
				sprintf(buff, "\r\n Written %d byte to sector %d ", rx_data_len,
						flash_sector_address);
				HAL_UART_Transmit(&huart1, (uint8_t*) buff,
						strlen((char*) buff), 100);
				sprintf(buff, "\r\n Source data size %d byte", rx_received_len);
				HAL_UART_Transmit(&huart1, (uint8_t*) buff,
						strlen((char*) buff), 100);
				step = end;
				break;

			case source_overflow:
				sprintf(buff,
						"\r\n Source buffer overflow.\r\n Max size %d byte",
						SOURCE_BUFFER_SIZE - 1);
				HAL_UART_Transmit(&huart1, (uint8_t*) buff,
						strlen((char*) buff), 100);
				step = error;
				break;

			case data_overflow:
				sprintf(buff, "\r\n Data buffer overflow.\r\n Max size %d byte",
						DATA_BUFFER_SIZE);
				HAL_UART_Transmit(&huart1, (uint8_t*) buff,
						strlen((char*) buff), 100);
				step = error;
				break;

			case error:
				sprintf(buff, "\r\n Fault.");
				HAL_UART_Transmit(&huart1, (uint8_t*) buff,
						strlen((char*) buff), 100);
				step = end;
				break;

			case end:
				flag_end = true;
				step = check_sector_index_prefix;
				rx_data_len = 0;
				break;

			default:
				step = error;
				break;
			}
		}

		flag_end = false;

	}

}

//**************************************************************************

// функция преобразования символа в число
int hex_char_to_bin(char ch)
{
        if(ch >= '0' && ch <= '9')
        {
                return (ch - '0');
        }
        else if(ch >= 'a' && ch <= 'f')
        {
                return (10 + ch - 'a');
        }
        else if(ch >= 'A' && ch <= 'F')
        {
                return (10 + ch - 'A');
        }
        return -1;

}
//********************************************************************************

//функции задержек для работы библиотеки дисплея
void u8g_port_delay_ns(uint8_t ns) {
	// Core @72 MHZ: 14ns per instruction.
	// __NOP(); is direct "nop;" instruction to cpu.
	// Divide ns / 28 (extra instruction for jump back to beginning of the loop) for loop cycles.
	for (uint8_t i = 0; i < (ns / 28); i++) {
		__NOP();
	}
}

void u8g_port_delay_100ns(uint8_t ns) {
	// Same as in u8g_hw_port_delay_ns function.
	// 100 / 28 = 3.57;
	for (uint16_t i = 0; i < (ns * 3.57); i++) {
		__NOP();
	}
}

void u8g_port_delay_10us(uint8_t us) {
	// Same as in u8g_hw_port_delay_ns function.
	// 3.57 * 100 ? 357;
	for (uint16_t i = 0; i < (us * 357); i++) {
		__NOP();
	}
}
//************************************************************************

// функция обработки задержек и управления gpio для работы библиотеки дисплея
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8,
		U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
		U8X8_UNUSED void *arg_ptr) {

	switch (msg) {

		case U8X8_MSG_GPIO_AND_DELAY_INIT:
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);
		break;

		case U8X8_MSG_DELAY_NANO:
		u8g_port_delay_ns(arg_int);
		break;

		case U8X8_MSG_DELAY_100NANO:
		u8g_port_delay_100ns(arg_int);
		break;

		case U8X8_MSG_DELAY_10MICRO:
		u8g_port_delay_10us(arg_int);
		break;

		case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;

		case U8X8_MSG_GPIO_RESET:
		if (arg_int)
		HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, SET);
		else
		HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, RESET);
		break;
		default:
		return 0;//A message was received which is not implemented, return 0 to indicate an error
	}

	return 1; // command processed successfully.
}
//***************************************************************************************************

// функция для работы библиотеки дисплея по SPI
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {

	switch (msg) {
	case U8X8_MSG_BYTE_SEND:
		HAL_SPI_Transmit(&hspi1, (uint8_t*) arg_ptr, arg_int, 100);
		break;

	case U8X8_MSG_BYTE_INIT:
		break;

	case U8X8_MSG_BYTE_SET_DC:
		 HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, arg_int);
		break;

	case U8X8_MSG_BYTE_START_TRANSFER:
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);
		break;

	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, SET);
		break;

	default:
		return 0;
	}
	return 1;
}
//*****************************************************************************************
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
