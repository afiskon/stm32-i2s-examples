/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/* vim: set ai et ts=4 sw=4: */
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "sdcard.h"
#include "st7735.h"
#include "ff.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

FATFS fs;

volatile bool end_of_file_reached = false;
volatile bool read_next_chunk = false;
volatile uint16_t* signal_play_buff = NULL;
volatile uint16_t* signal_read_buff = NULL;
volatile uint16_t signal_buff1[4096];
volatile uint16_t signal_buff2[4096];

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if(end_of_file_reached)
        return;

    volatile uint16_t* temp = signal_play_buff;
    signal_play_buff = signal_read_buff;
    signal_read_buff = temp;

    int nsamples = sizeof(signal_buff1) / sizeof(signal_buff1[0]);
    HAL_I2S_Transmit_IT(&hi2s2, (uint16_t*)signal_play_buff, nsamples);
    read_next_chunk = true;
}

void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
    va_end(args);
}

int playWavFile(const char* fname) {
    UART_Printf("Openning %s...\r\n", fname);
    FIL file;
    FRESULT res = f_open(&file, fname, FA_READ);
    if(res != FR_OK) {
        UART_Printf("f_open() failed, res = %d\r\n", res);
        return -1;
    }

    UART_Printf("File opened, reading...\r\n");

    unsigned int bytesRead;
    uint8_t header[44];
    res = f_read(&file, header, sizeof(header), &bytesRead);
    if(res != FR_OK) {
        UART_Printf("f_read() failed, res = %d\r\n", res);
        f_close(&file);
        return -2;
    }

    if(memcmp((const char*)header, "RIFF", 4) != 0) {
        UART_Printf("Wrong WAV signature at offset 0: 0x%02X 0x%02X 0x%02X 0x%02X\r\n", header[0], header[1], header[2], header[3]);
        f_close(&file);
        return -3;
    }

    if(memcmp((const char*)header + 8, "WAVEfmt ", 8) != 0) {
        UART_Printf("Wrong WAV signature at offset 8!\r\n");
        f_close(&file);
        return -4;
    }

    if(memcmp((const char*)header + 36, "data", 4) != 0) {
        UART_Printf("Wrong WAV signature at offset 36!\r\n");
        f_close(&file);
        return -5;
    }

    uint32_t fileSize = 8 + (header[4] | (header[5] << 8) | (header[6] << 16) | (header[7] << 24));
    uint32_t headerSizeLeft = header[16] | (header[17] << 8) | (header[18] << 16) | (header[19] << 24);
    uint16_t compression = header[20] | (header[21] << 8);
    uint16_t channelsNum = header[22] | (header[23] << 8);
    uint32_t sampleRate = header[24] | (header[25] << 8) | (header[26] << 16) | (header[27] << 24);
    uint32_t bytesPerSecond = header[28] | (header[29] << 8) | (header[30] << 16) | (header[31] << 24);
    uint16_t bytesPerSample = header[32] | (header[33] << 8);
    uint16_t bitsPerSamplePerChannel = header[34] | (header[35] << 8);
    uint32_t dataSize = header[40] | (header[41] << 8) | (header[42] << 16) | (header[43] << 24);

    UART_Printf(
        "--- WAV header ---\r\n"
        "File size: %lu\r\n"
        "Header size left: %lu\r\n"
        "Compression (1 = no compression): %d\r\n"
        "Channels num: %d\r\n"
        "Sample rate: %ld\r\n"
        "Bytes per second: %ld\r\n"
        "Bytes per sample: %d\r\n"
        "Bits per sample per channel: %d\r\n"
        "Data size: %ld\r\n"
        "------------------\r\n",
        fileSize, headerSizeLeft, compression, channelsNum, sampleRate, bytesPerSecond, bytesPerSample,
        bitsPerSamplePerChannel, dataSize);

    if(headerSizeLeft != 16) {
        UART_Printf("Wrong `headerSizeLeft` value, 16 expected\r\n");
        f_close(&file);
        return -6;
    }

    if(compression != 1) {
        UART_Printf("Wrong `compression` value, 1 expected\r\n");
        f_close(&file);
        return -7;
    }

    if(channelsNum != 2) {
        UART_Printf("Wrong `channelsNum` value, 2 expected\r\n");
        f_close(&file);
        return -8;
    }

    if((sampleRate != 44100) || (bytesPerSample != 4) || (bitsPerSamplePerChannel != 16) || (bytesPerSecond != 44100*2*2)
       || (dataSize < sizeof(signal_buff1) + sizeof(signal_buff2))) {
        UART_Printf("Wrong file format, 16 bit file with sample rate 44100 expected\r\n");
        f_close(&file);
        return -9;
    }

    res = f_read(&file, (uint8_t*)signal_buff1, sizeof(signal_buff1), &bytesRead);
    if(res != FR_OK) {
        UART_Printf("f_read() failed, res = %d\r\n", res);
        f_close(&file);
        return -10;
    }

    res = f_read(&file, (uint8_t*)signal_buff2, sizeof(signal_buff2), &bytesRead);
    if(res != FR_OK) {
        UART_Printf("f_read() failed, res = %d\r\n", res);
        f_close(&file);
        return -11;
    }

    read_next_chunk = false;
    end_of_file_reached = false;
    signal_play_buff = signal_buff1;
    signal_read_buff = signal_buff2;

    HAL_StatusTypeDef hal_res;
    int nsamples = sizeof(signal_buff1) / sizeof(signal_buff1[0]);
    hal_res = HAL_I2S_Transmit_IT(&hi2s2, (uint16_t*)signal_buff1, nsamples);
    if(hal_res != HAL_OK) {
        UART_Printf("I2S - HAL_I2S_Transmit failed, hal_res = %d!\r\n", hal_res);
        f_close(&file);
        return -12;
    }

    while(dataSize >= sizeof(signal_buff1)) {
        if(!read_next_chunk) {
            continue;
        }

        read_next_chunk = false;

        res = f_read(&file, (uint8_t*)signal_read_buff, sizeof(signal_buff1), &bytesRead);
        if(res != FR_OK) {
            UART_Printf("f_read() failed, res = %d\r\n", res);
            f_close(&file);
            return -13;
        }

        dataSize -= sizeof(signal_buff1);
    }

    end_of_file_reached = true;

    res = f_close(&file);
    if(res != FR_OK) {
        UART_Printf("f_close() failed, res = %d\r\n", res);
        return -14;
    }

    return 0;
}

int init() {
    // unselect all SPI devices first
    SDCARD_Unselect();
    ST7735_Unselect();

    // initialize SD-card as fast as possible, it glitches otherwise
    // (this is important only if SPI bus is shared by multiple devices)
    int code = SDCARD_Init();
    if(code < 0) {
        UART_Printf("SDCARD_Init() failed, code = %d\r\n", code);
        return -1;
    }

    ST7735_Init();
    ST7735_FillScreen(ST7735_BLACK);

    // mount the default drive
    FRESULT res = f_mount(&fs, "", 0);
    if(res != FR_OK) {
        UART_Printf("f_mount() failed, res = %d\r\n", res);
        return -2;
    }
    UART_Printf("f_mount() done!\r\n");

    return 0;
}

void loop() {
    playWavFile("test.wav");
    HAL_Delay(5000);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_I2S2_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int code = init();
  if(code < 0) {
    UART_Printf("init() failed with code %d, terminating.\r\n", code);
    return 0;
  }

  while (1)
  {
    loop();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Btn2_Pin Btn3_Pin Btn4_Pin */
  GPIO_InitStruct.Pin = Btn2_Pin|Btn3_Pin|Btn4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Btn1_Pin */
  GPIO_InitStruct.Pin = Btn1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Btn1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
