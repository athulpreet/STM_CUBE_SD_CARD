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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
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

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
char buffer[64];
uint8_t seconds, minutes, hours, day, date, month, year;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);




/* USER CODE BEGIN PFP */
void process_SD_card( void );
bool mcp7940n_init(void);
bool mcp7940n_read_time(void);
bool mcp7940n_set_time(uint8_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t min, uint8_t sec);
bool mcp7940n_write_register(uint8_t reg, uint8_t value);
bool mcp7940n_read_register(uint8_t reg, uint8_t *value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void process_SD_card( void )
{
  FATFS       FatFs;                //Fatfs handle
  FIL         fil;                  //File handle
  FRESULT     fres;                 //Result after operations
  char        buf[100];
  int len=0;
  do
  {
    //Mount the SD Card
    fres = f_mount(&FatFs, "", 1);    //1=mount now
    if (fres != FR_OK)
    {
      //printf("No SD Card found : (%i)\r\n", fres);



      len = sprintf(buffer, "No SD Card found : (%i)\r\n", fres);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

      break;
    }
    //printf("SD Card Mounted Successfully!!!\r\n");
     HAL_UART_Transmit(&huart2, (uint8_t*)"SD Card Mounted Successfully!!!\r\n", 35, HAL_MAX_DELAY);
    //Read the SD Card Total size and Free Size
    FATFS *pfs;
    DWORD fre_clust;
    uint32_t totalSpace, freeSpace;

    f_getfree("", &fre_clust, &pfs);
    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);

   // printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);
    len = sprintf(buffer,"TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    //Open the file
    fres = f_open(&fil, "EmbeTronicX.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(fres != FR_OK)
    {
      //printf("File creation/open Error : (%i)\r\n", fres);
      len = sprintf(buffer,"File creation/open Error : (%i)\r\n", fres);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
      break;
    }

    //printf("Writing data!!!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)"Writing data!!!\r\n", 30, HAL_MAX_DELAY);
    //write the data
    f_puts("Welcome to EmbeTronicX", &fil);

    //close your file
    f_close(&fil);

    //Open the file
    fres = f_open(&fil, "EmbeTronicX.txt", FA_READ);
    if(fres != FR_OK)
    {
      //printf("File opening Error : (%i)\r\n", fres);
      len = sprintf(buffer,"File opening Error : (%i)\r\n", fres);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);


      break;
    }

    //read the data
    f_gets(buf, sizeof(buf), &fil);

   // printf("Read Data : %s\n", buf);
    len =  sprintf(buffer,"Read Data : %s\n", buf);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    //close your file
    f_close(&fil);
   // printf("Closing File!!!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)"Closing File!!!\r\n", 30, HAL_MAX_DELAY);
#if 0
    //Delete the file.
    fres = f_unlink(EmbeTronicX.txt);
    if (fres != FR_OK)
    {
      printf("Cannot able to delete the file\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)"Cannot able to delete the file\n", 30, HAL_MAX_DELAY);
    }
#endif
  } while( false );

  //We're done, so de-mount the drive
  f_mount(NULL, "", 0);
  printf("SD Card Unmounted Successfully!!!\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)"SD Card Unmounted Successfully!!!\r\n", 30, HAL_MAX_DELAY);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  // Change USART2 baud rate to 9600
  huart2.Init.BaudRate = 9600;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  // Send initialization message
  HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing MCP7940N RTC...\r\n", 30, HAL_MAX_DELAY);

  // Initialize MCP7940N RTC
  if (mcp7940n_init())
  {
    HAL_UART_Transmit(&huart2, (uint8_t*)"RTC initialized successfully\r\n", 29, HAL_MAX_DELAY);

    // Set initial time: 2023/04/09 14:30:00
    /*
    if (mcp7940n_set_time(25, 4, 11, 14, 45, 0))
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Time set successfully\r\n", 23, HAL_MAX_DELAY);
    }
    else
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to set time\r\n", 20, HAL_MAX_DELAY);
    }
    */
  }
  else
  {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to initialize RTC\r\n", 26, HAL_MAX_DELAY);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  process_SD_card();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Read current time from RTC
    if (mcp7940n_read_time())
    {
      // Format the date and time string
      sprintf(buffer, "RTC Time: 20%02d/%02d/%02d %02d:%02d:%02d\r\n",
              year, month, date, hours, minutes, seconds);

      // Send to UART2
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
    else
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Error reading RTC\r\n", 19, HAL_MAX_DELAY);
    }

    // Wait one second before next update
    HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// MCP7940N I2C address (7-bit address)
#define MCP7940N_ADDRESS 0x6F

// MCP7940N registers
#define MCP7940N_REG_SECONDS 0x00
#define MCP7940N_REG_MINUTES 0x01
#define MCP7940N_REG_HOURS   0x02
#define MCP7940N_REG_WKDAY   0x03
#define MCP7940N_REG_DATE    0x04
#define MCP7940N_REG_MONTH   0x05
#define MCP7940N_REG_YEAR    0x06
#define MCP7940N_REG_CONTROL 0x07
#define MCP7940N_REG_OSCTRIM 0x08

// Storage for date/time
uint8_t seconds, minutes, hours, day, date, month, year;

// Write a register on the MCP7940N
bool mcp7940n_write_register(uint8_t reg, uint8_t value)
{
  uint8_t data[2];
  data[0] = reg;
  data[1] = value;

  if (HAL_I2C_Master_Transmit(&hi2c1, MCP7940N_ADDRESS << 1, data, 2, HAL_MAX_DELAY) != HAL_OK)
  {
    return false;
  }
  return true;
}

// Read a register from the MCP7940N
bool mcp7940n_read_register(uint8_t reg, uint8_t *value)
{
  if (HAL_I2C_Master_Transmit(&hi2c1, MCP7940N_ADDRESS << 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return false;
  }

  if (HAL_I2C_Master_Receive(&hi2c1, MCP7940N_ADDRESS << 1 | 1, value, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return false;
  }

  return true;
}

// Read time from MCP7940N
bool mcp7940n_read_time()
{
  uint8_t value;

  // Read seconds
  if (!mcp7940n_read_register(MCP7940N_REG_SECONDS, &value)) return false;
  // Strip off the ST bit (bit 7)
  seconds = (value & 0x0F) + ((value & 0x70) >> 4) * 10;

  // Read minutes
  if (!mcp7940n_read_register(MCP7940N_REG_MINUTES, &value)) return false;
  minutes = (value & 0x0F) + ((value & 0x70) >> 4) * 10;

  // Read hours (assuming 24-hour mode)
  if (!mcp7940n_read_register(MCP7940N_REG_HOURS, &value)) return false;
  hours = (value & 0x0F) + ((value & 0x30) >> 4) * 10;

  // Read weekday
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) return false;
  day = value & 0x07;  // Only lower 3 bits represent day of week

  // Read date
  if (!mcp7940n_read_register(MCP7940N_REG_DATE, &value)) return false;
  date = (value & 0x0F) + ((value & 0x30) >> 4) * 10;

  // Read month
  if (!mcp7940n_read_register(MCP7940N_REG_MONTH, &value)) return false;
  month = (value & 0x0F) + ((value & 0x10) >> 4) * 10;

  // Read year
  if (!mcp7940n_read_register(MCP7940N_REG_YEAR, &value)) return false;
  year = (value & 0x0F) + ((value & 0xF0) >> 4) * 10;

  return true;
}

// Set time on MCP7940N
bool mcp7940n_set_time(uint8_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t min, uint8_t sec)
{
  // Convert decimal to BCD
  uint8_t sec_bcd = ((sec / 10) << 4) | (sec % 10);
  uint8_t min_bcd = ((min / 10) << 4) | (min % 10);
  uint8_t hour_bcd = ((h / 10) << 4) | (h % 10);
  uint8_t date_bcd = ((d / 10) << 4) | (d % 10);
  uint8_t month_bcd = ((m / 10) << 4) | (m % 10);
  uint8_t year_bcd = ((y / 10) << 4) | (y % 10);
  uint8_t wkday = 1;  // Default to Monday

  // For seconds, set the ST bit (bit 7) to enable oscillator
  sec_bcd |= 0x80;

  // Write time/date registers
  if (!mcp7940n_write_register(MCP7940N_REG_SECONDS, sec_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_MINUTES, min_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_HOURS, hour_bcd)) return false;
  // Set the VBATEN bit (bit 3) in RTCWKDAY register to enable battery backup
  if (!mcp7940n_write_register(MCP7940N_REG_WKDAY, wkday | 0x08)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_DATE, date_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_MONTH, month_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_YEAR, year_bcd)) return false;

  return true;
}

// Initialize the MCP7940N RTC
bool mcp7940n_init()
{
  // Test communication
  uint8_t value;
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) {
    return false;
  }

  // Enable oscillator by setting ST bit (bit 7) in seconds register
  if (!mcp7940n_read_register(MCP7940N_REG_SECONDS, &value)) {
    return false;
  }
  value |= 0x80;  // Set ST bit
  if (!mcp7940n_write_register(MCP7940N_REG_SECONDS, value)) {
    return false;
  }

  // Enable battery backup by setting VBATEN bit (bit 3) in RTCWKDAY register
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) {
    return false;
  }
  value |= 0x08;  // Set VBATEN bit
  if (!mcp7940n_write_register(MCP7940N_REG_WKDAY, value)) {
    return false;
  }

  // Set control register (basic configuration)
  return mcp7940n_write_register(MCP7940N_REG_CONTROL, 0x00);
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
