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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


uint32_t addrAdc = 0x08004000;
uint32_t addrAdcEnd = 0x0800A400;
uint32_t addrAdc1 = 0x08004000;
uint32_t addrAdc2 = 0x08004000;
uint32_t addrAcc = 0x0800A400;
uint32_t addrAccEnd = 0x0800D600;
uint32_t addrAcc1 = 0x0800A400;
uint32_t addrAcc2 = 0x0800A400;
uint32_t addrEra = 0x08004000;
uint8_t adcBuf[3]=0;
uint8_t uartmid[4]={0x12,0x34,0x56,0x78};
uint8_t acc[4]=0;
uint8_t adcUart[3]=0;
uint8_t accUart[4]=0;
uint8_t comFlag1=0;
uint8_t comFlag2=0;
uint8_t comFlag3=0;
uint8_t timFlag1=0;
uint8_t timFlag2=0;

static void ADXL345_Select() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
static void ADXL345_Unselect() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void ADXL345_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t ui8address,	uint8_t ui8Data) 
{
	uint8_t data[2];
	data[0] = ((ui8address ) ); /* Combine write register address and Write command */
	data[1] = ui8Data;
	ADXL345_Select(); /* Select accelerometer */
	HAL_SPI_Transmit(&hspi1, data, 2,0xffff);
	ADXL345_Unselect(); /* Deselect accelerometer */
}
void ADXL345_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t ui8address,uint8_t* acc) 
{
	uint8_t ui24Result[6];
	//uint32_t ui32Result;
	uint8_t ui8writeAddress;
	ui8writeAddress = (ui8address|0x80); /* Combine read register address and READ command */
	ADXL345_Select(); /* Select accelerometer */
	HAL_SPI_Transmit(&hspi1, &ui8writeAddress, 1,0xffff); /* Send register address */
	HAL_SPI_Receive(&hspi1, acc, 1,0xffff);
	/* Combine 3Bit register into one uint32 */
	
	//ui32Result = ((ui24Result[0] << 16) | (ui24Result[1] << 8) | ui24Result[2]);

	ADXL345_Unselect(); /* Deselect accelerometer */
}
void ADXL_init()
{
    ADXL345_WriteRegister(&hspi1,0x2c,0x0F);
    ADXL345_WriteRegister(&hspi1,0x2D,0x08);
    ADXL345_WriteRegister(&hspi1,0x31,0x0B);
}
HAL_StatusTypeDef eraseFlash(uint32_t addr){
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = addr;
	f.NbPages = 1;
	uint32_t PageError = 0;
	if(HAL_FLASHEx_Erase(&f, &PageError)==HAL_OK){
		HAL_FLASH_Lock();
		if(PageError != 0xFFFFFFFF) return HAL_ERROR;
		return HAL_OK;	
	}
	HAL_FLASH_Lock();
	return HAL_ERROR;
}
void writeFlashTest(uint32_t addr,uint32_t writeFlashData)
{
  HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, writeFlashData);
  HAL_FLASH_Lock();
}

void readAdcFlash(uint32_t addr)
{
  uint32_t temp = *(__IO uint32_t*)(addr);
	adcUart[2]=temp&0xff;
	adcUart[1]=(temp>>8)&0xff;
	adcUart[0]=(temp>>16)&0xff;
}
void readAccFlash(uint32_t addr)
{
  uint32_t temp = *(__IO uint32_t*)(addr);
	accUart[2]=temp&0xff;
	accUart[3]=(temp>>8)&0xff;
	accUart[0]=(temp>>16)&0xff;
	accUart[1]=(temp>>24)&0xff;
}


void SystemClock_Config(void);

int main(void)
{
  
  HAL_Init();

  
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  while(addrEra<addrAccEnd){
		eraseFlash(addrEra);
		addrEra += 0x0400;
	}
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcBuf,3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	
  while (1)
  {
    if(comFlag1==1){
			comFlag1=2;
			comFlag3=1;
			HAL_TIM_Base_Stop_IT(&htim3);
		}
		if(comFlag3==1){
			while(addrAdc2<addrAdcEnd){
				readAdcFlash(addrAdc2);
				HAL_UART_Transmit(&huart1, (uint8_t *)adcUart, sizeof(adcUart),0xffff);
				addrAdc2+=0x04;
			}
			HAL_UART_Transmit(&huart1, (uint8_t *)uartmid, sizeof(uartmid),0xffff);
			while(addrAcc2<addrAccEnd){
				readAccFlash(addrAcc2);
				HAL_UART_Transmit(&huart1, (uint8_t *)accUart, sizeof(accUart),0xffff);
				addrAcc2+=0x04;
			}
			break;
		}
  }
  /* USER CODE END 3 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim2))
    {
			
    }
		if (htim == (&htim3))
    {
			if(timFlag1==0){
				if(addrAdc1<addrAdcEnd){
					uint32_t temp2 = (adcBuf[0]<<16)|(adcBuf[1]<<8)|adcBuf[2];
					writeFlashTest(addrAdc1,temp2);
					addrAdc1+=0x04;
				}else{
					comFlag1=1;
				}
				timFlag1=1;
			}
			if(timFlag1==1){
				if(addrAdc1<addrAdcEnd){
					ADXL345_ReadRegister(&hspi1,0x32,acc);
					ADXL345_ReadRegister(&hspi1,0x33,acc+1);
					ADXL345_ReadRegister(&hspi1,0x34,acc+2);
					ADXL345_ReadRegister(&hspi1,0x35,acc+3);
					uint32_t temp3 = (acc[0]<<24)|(acc[1]<<16)|(acc[2]<<8)|(acc[3]);
					uint32_t temp2 = (adcBuf[0]<<16)|(adcBuf[1]<<8)|adcBuf[2];
					writeFlashTest(addrAcc1,temp3);
					addrAcc1+=0x04;
					writeFlashTest(addrAdc1,temp2);
					addrAdc1+=0x04;
				}else{
					comFlag1=1;
				}
				timFlag1=0;
			}
    }
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
