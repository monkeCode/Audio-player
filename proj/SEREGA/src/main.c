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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "audio.h"
#include "dma.h"
#include "w25qxx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define    W25_ENABLE_RESET  0x66
#define    W25_RESET  0x99
#define    W25_READ  0x03
#define    W25_GET_JEDEC_ID  0x9f
#define   W25_WRITE_ENABLE  0x06
#define   W25_WRITE_DISABLE  0x04
#define W25_PAGE 0x02


#define mem_pin_set() HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET)
#define mem_pin_reset() HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t low_byte(uint16_t x)
{
  return ((uint8_t)(x%256));
} 

uint8_t high_byte( uint16_t x)
{
  return ((uint8_t)(x / 256));
} 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t dac_addr = 0b01100000; // адрес с учетом A0 к GND

/* USER CODE END 0 */

HAL_StatusTypeDef DAC_write(uint16_t value)
{
    uint8_t buffer[3]; 
    buffer[0] = 0b01000000; // Установка буфера DAC в режиме записи
    buffer[2] = value >> 4;  // Наиболее значимые биты
    buffer[1] = value << 4; // Младшие биты

      return HAL_I2C_Master_Transmit(&hi2c1, dac_addr << 1, buffer, sizeof(buffer), HAL_MAX_DELAY);
}

HAL_StatusTypeDef DAC_write_dma(uint16_t value)
{
  uint8_t buffer[2]; 
    buffer[0] = 0b00000000; 
    buffer[0] += high_byte(value); // старшие биты
    buffer[1] = low_byte(value);  // младшие

      return HAL_I2C_Master_Transmit_DMA(&hi2c1, dac_addr << 1, buffer, sizeof(buffer));
}

HAL_StatusTypeDef DAC_write_fast_mode(uint16_t value)
{
  uint8_t buffer[2]; 
    buffer[0] = 0b00000000; 
    buffer[0] += high_byte(value) & 0b00001111; // старшие биты
    buffer[1] = low_byte(value);  // младшие

      return HAL_I2C_Master_Transmit(&hi2c1, dac_addr << 1, buffer, sizeof(buffer), HAL_MAX_DELAY);
}



HAL_StatusTypeDef read_memory(uint8_t* buffer, uint8_t size)
{
  return HAL_SPI_Receive(&hspi1, buffer, size, 500);
}

HAL_StatusTypeDef write_memory(uint8_t* data, uint8_t size)
{
  return HAL_SPI_Transmit(&hspi1, data, size, 500);
} 

HAL_StatusTypeDef W25_Reset(void)
{
  uint8_t tx_buf[2];
  mem_pin_set();
  tx_buf[0] = W25_ENABLE_RESET;
  tx_buf[1] = W25_RESET;
  HAL_StatusTypeDef res = write_memory(tx_buf, 2);
//  HAL_Delay(500);
  mem_pin_reset();
  return res;
}

uint32_t W25_Read_ID(void)
{
  uint8_t dt[3];
  uint8_t tx_buf[0];
  tx_buf[0] = W25_GET_JEDEC_ID;
  mem_pin_set();
  //HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_buf, dt, 1, 500);
  HAL_StatusTypeDef status =  write_memory(tx_buf, 1);
  if (status == HAL_OK)
  {
   status = read_memory(dt, 3);
  }
  mem_pin_reset();
  if (status == HAL_OK)
    return (dt[0] << 16) | (dt[1] << 8) | dt[2];
  return -1;
}

HAL_StatusTypeDef W25_Write_Enable()
{
  uint8_t tx_buf[1] = {W25_WRITE_ENABLE};
  mem_pin_set();
  HAL_StatusTypeDef res =  write_memory(tx_buf, 1);
  mem_pin_reset();
  return res;
}

HAL_StatusTypeDef W25_Write_Disable()
{
  uint8_t tx_buf[1] = {W25_WRITE_DISABLE};
  mem_pin_set();
  HAL_StatusTypeDef res =  write_memory(tx_buf, 1);
  mem_pin_reset();
  return res;
}

HAL_StatusTypeDef W25_Read_Data(uint32_t addr, uint8_t* data)
{
  uint8_t tx_buf[4];
  mem_pin_set();
  tx_buf[0] = W25_READ;
  tx_buf[1] = (addr >> 16) & 0xFF;
  tx_buf[2] = (addr >> 8) & 0xFF;
  tx_buf[3] = addr & 0xFF;
  HAL_StatusTypeDef res =  write_memory(tx_buf, 4);
  if(res == HAL_OK)
  {
    res = read_memory(data, 1);
  }
  mem_pin_reset();
  return res;
}

HAL_StatusTypeDef W25_Write_Data(uint32_t addr, uint8_t* data, uint8_t sz)
{
  uint8_t tx_buf[4];
  tx_buf[0] = W25_PAGE;
  tx_buf[1] = (addr >> 16) & 0xFF;
  tx_buf[2] = (addr >> 8) & 0xFF;
  tx_buf[3] = addr & 0xFF;
  mem_pin_set();
  write_memory(tx_buf, 4);
  HAL_StatusTypeDef res = write_memory(data, sz);
  mem_pin_reset();
  return res;
}


void write_audio(uint8_t* audio,  uint32_t address_start)
{
  uint32_t addr = address_start;
  for(int i = 0; i < sizeof(audio); i+=256)
  {
    uint8_t size = 255;
    if(sizeof(audio) - i > 256)
      size = sizeof(audio) - i;
    W25_Write_Data(addr, audio[i], size);
    addr += size;
  }
}

void fill_buffer(uint8_t* buffer, uint16_t buffer_size, uint32_t addr_start)
{
  for(int i = 0; i < buffer_size; i++)
  {
    uint8_t rx;
    W25_Read_Data(addr_start, &rx);
    buffer[i] = rx;
    addr_start += 1;
  }
}

uint8_t buffer[256];
uint32_t addr = 0;
uint32_t inx = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
    {
      // if(inx >= sizeof(buffer))
      // {
      //   fill_buffer(buffer, sizeof(buffer), addr);
      //   addr += sizeof(buffer);
      //   addr %= 40000;
      //   inx = 0;
      // }
    /* USER CODE END WHILE */
      // if(inx == sizeof(audio))
      // {
      //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      //   return;
      // }
      uint16_t val = ((uint16_t)audio[inx]) * 15;
      DAC_write_fast_mode(val);
      inx +=3;
      inx %= sizeof(audio);
    }
}


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
  MX_DMA_Init();
  MX_SPI1_Init();

  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //W25_Reset();

//  W25_Write_Enable();
  //write_audio(audio, 0);
//  W25_Write_Disable();

  //fill_buffer(buffer, sizeof(buffer), addr);
  addr+=sizeof(buffer);


  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  // for(int i = 0; i < 100; i++)
  // {
  //   uint8_t rx;
  //   W25_Read_Data(i, &rx);

  //   if(rx == data[i])
  //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  //   else 
  //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  // } 

//  W25qxx_Init();
//  W25qxx_EraseBlock(0);

  //W25qxx_WritePage(data, 0, 0, sizeof(data));

//  for(int i = 0; i < 100; i++)
//  {
//    W25qxx_WriteByte(data[i], i);
//  }

//  for(int i = 0; i < 100; i++)
  // {
  //   uint8_t rx_data[1];
  //   W25qxx_ReadByte(rx_data, i);
  //   if(rx_data[0] == data[i])
  //   {
  //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  //   }
  //   else{
  //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  //   }
  // }

  while (1)
  {
      //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
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
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(500);
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
