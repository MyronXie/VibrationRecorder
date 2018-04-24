/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "common.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS fs;                 // Work area (file system object) for logical drive
FIL fil;                  // file objects
char fileName[20]="default.txt";
uint16_t fileNo=0;
uint32_t byteswritten;                /* File write counts */

char wtext[100] = "FatFs on STM32F765."; /* File write buffer */
uint32_t seqStamp=0;
uint32_t tmStamp=0,tmStampLst=0;
uint8_t errCnt=0;
uint8_t exitCnt=0;

#define startAddr 0X08018000
FLASH_EraseInitTypeDef fl_erase;
uint32_t PageError = 0;

SensorType MPU6050_0xD0={0x00,0xD0,"MPU6050[0xD0]",0,0,{0,0,0}};
SensorType MPU6050_0xD2={0x01,0xD2,"MPU6050[0xD2]",0,0,{0,0,0}};
SensorType ADXL345_0xA6={0x10,0xA6,"ADXL345[0xA6]",0,0,{0,0,0}};
SensorType ADXL345_0x3A={0x11,0x3A,"ADXL345[0x3A]",0,0,{0,0,0}};
SensorType H3LIS100DL_0x30={0x20,0x30,"H3LIS100DL[0x30]",0,0,{0,0,0}};
SensorType H3LIS100DL_0x32={0x21,0x32,"H3LIS100DL[0x32]",0,0,{0,0,0}};

// Select enabled sensor
SensorType *sensorList[4]={&ADXL345_0xA6,&ADXL345_0x3A,&H3LIS100DL_0x30,&H3LIS100DL_0x32};

uint8_t status=0;
uint8_t i=0,j=0;

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern uint32_t fatTime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t SensorInit(SensorType* sensor);
uint8_t SensorGetData(SensorType* sensor);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SDMMC1_SD_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
    printf("\r\n ****** Vibration Logger (bulid 180424) ******\r\n");
  
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    
    printf("\r\n Time:%02d-%02d-%02d %02d:%02d:%02d",sDate.Year,sDate.Month,sDate.Date,sTime.Hours,sTime.Minutes,sTime.Seconds);
    fatTime = ((sDate.Year+20)<<25)|(sDate.Month<<21)|(sDate.Date<<16)|(sTime.Hours<<11)|(sTime.Minutes<<5)|(sTime.Seconds>>1);
  
	fileNo = *(__IO uint32_t*)(startAddr);
	fileNo++;
  
	HAL_FLASH_Unlock();
	fl_erase.TypeErase=FLASH_TYPEERASE_SECTORS;
	fl_erase.Sector=FLASH_SECTOR_3;
	fl_erase.NbSectors=1;
	fl_erase.VoltageRange=FLASH_VOLTAGE_RANGE_3;
	
	HAL_FLASHEx_Erase(&fl_erase, &PageError);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, startAddr, (uint32_t)fileNo);
	
	HAL_FLASH_Lock();
	
	fileNo = *(__IO uint32_t*)(startAddr);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/*##-1- Register the file system object to the FatFs module ##############*/
        printf("\r\n Mount: ");
		retSD = f_mount(&fs, "0:", 1);
		if(retSD)   printf("error: %d",retSD);
		else        printf("success!");
		 
		/*##-2- Create and Open new text file objects with write access ######*/
        printf("\r\n Open : ");
		sprintf(fileName,"#%d.txt",fileNo);
		retSD = f_open(&fil, fileName, FA_CREATE_ALWAYS | FA_WRITE);
		if(retSD)   printf("error: %d",retSD);
		else        printf("success!: %s",fileName);
		
		errCnt=0;
		
		 /* MEMS Initial process */
		
        for(int i=0;i<4;i++)
        {
            printf("\r\n #%d-%s:",i,sensorList[i]->name);
            status = SensorInit(sensorList[i]);

            if(!status)
            {
                printf("Init success!");
                sensorList[i]->enabled=1;
            }
            else
            {
                printf("Init error!");
                sensorList[i]->enabled=0;
            }
        }

		sprintf(wtext,"SeqSmp, TmSmp,Acc_1x,Acc_1y,Acc_1z,Acc_2x,Acc_2y,Acc_2z,Acc_3x,Acc_3y,Acc_3z,Acc_4x,Acc_4y,Acc_4z");
		retSD = f_write(&fil, wtext, sizeof(wtext), (void *)&byteswritten);
        printf("\r\n%s",wtext);
		
		while(1)
		{
			seqStamp++;
            tmStamp = HAL_GetTick();
            
            for(int i=0;i<4;i++)
            {
                if(sensorList[i]->enabled)
                {
                    status = SensorGetData(sensorList[i]);
                    if(status)
                    {
                        if(++sensorList[i]->err>=10)
                        {
                            sensorList[i]->enabled = 0;
                            sensorList[i]->rawData[0]=0; sensorList[i]->rawData[1]=0; sensorList[i]->rawData[2]=0;
                            printf("\r\n%6d: %s offboard!",seqStamp,sensorList[i]->name);
                        }
                    }
                    else sensorList[i]->err=0;
                }

            }
            
            sprintf(wtext,"\r\n%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d,%6d",seqStamp%1000000,tmStamp%1000000,
                        sensorList[0]->rawData[0],sensorList[0]->rawData[1],sensorList[0]->rawData[2],
                        sensorList[1]->rawData[0],sensorList[1]->rawData[1],sensorList[1]->rawData[2],
                        sensorList[2]->rawData[0],sensorList[2]->rawData[1],sensorList[2]->rawData[2],
                        sensorList[3]->rawData[0],sensorList[3]->rawData[1],sensorList[3]->rawData[2]);
            
            retSD = f_write(&fil, wtext, sizeof(wtext), (void *)&byteswritten);          
    
            if(tmStamp-tmStampLst>=1000)
            {
                tmStampLst = tmStamp;
                printf("%s",wtext);
            }
            				
			if(seqStamp%256==0)
			{
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);	
			}
            
            if(seqStamp%256==0)
            {
                for(int i=0;i<4;i++)
                {
                    if(!sensorList[i]->enabled)
                    {
                        status = SensorInit(sensorList[i]);
                        
                        if(!status)
                        {
                            sensorList[i]->enabled=1;
                            printf("\r\n%6d: %s onboard!",seqStamp,sensorList[i]->name);
                        }
                        else        sensorList[i]->enabled=0;
                    }
                }
            }
	
			if(seqStamp%256==0)	
			{
				retSD = f_sync(&fil);
				if(retSD)
                {
                    errCnt++;
                    printf(",Sync:%d",errCnt);
                }
                else        errCnt=0;
                
			}
			
            if(errCnt>5)    break;

		}
		
		/*##-4- Close the open text files ################################*/
		retSD = f_close(&fil);
		if(retSD)   printf(" Close error: %d\r\n",retSD);
		else        printf(" Close success!\r\n");
        
        if(errCnt>=5)	
		{
			printf("Fatal Error! Reset system...");
			NVIC_SystemReset();
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
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

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)      // TIM2: System Management (1Hz)
    {
    }
}

uint8_t SensorInit(SensorType* sensor)
{
    uint8_t status = 0;
    switch((sensor->id)>>4)
    {
        case TYPE_MPU6050:
            status+=I2C_WriteByte(sensor->addr,0x6B,0x00);
            status+=I2C_WriteByte(sensor->addr,0x19,0x07);
            status+=I2C_WriteByte(sensor->addr,0x1A,0x07);
            status+=I2C_WriteByte(sensor->addr,0x1B,0x18);
            status+=I2C_WriteByte(sensor->addr,0x1C,0x18); 
            break;
        
        case TYPE_ADXL345:
            status+=I2C_WriteByte(sensor->addr,0x31,0x0B);
            status+=I2C_WriteByte(sensor->addr,0x2C,0x0F);
            status+=I2C_WriteByte(sensor->addr,0x2D,0x08);
            status+=I2C_WriteByte(sensor->addr,0x2E,0x80);
            status+=I2C_WriteByte(sensor->addr,0x1E,0x00);
            status+=I2C_WriteByte(sensor->addr,0x1F,0x00);
            status+=I2C_WriteByte(sensor->addr,0x20,0x05);
            break;
        
        case TYPE_H3LIS100DL:
            status+=I2C_WriteByte(sensor->addr,0x20,0x37);
            status+=I2C_WriteByte(sensor->addr,0x21,0x00);
            status+=I2C_WriteByte(sensor->addr,0x22,0x00);
            status+=I2C_WriteByte(sensor->addr,0x23,0x00);
            status+=I2C_WriteByte(sensor->addr,0x24,0x00);
            break;
    }
    
    return status;

}

uint8_t SensorGetData(SensorType* sensor)
{
    uint8_t status = 0;
    switch((sensor->id)>>4)
    {
        case TYPE_MPU6050: status = MPU6050_RawData(sensor->addr,0x3B,sensor->rawData); break;
            
        case TYPE_ADXL345: status = ADXL345_RawData(sensor->addr,0x32,sensor->rawData); break;
            
        case TYPE_H3LIS100DL: status = H3LIS100DL_RawData(sensor); break;
    }

    return status;
}

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
	printf("[Error]File:%s, Line:%d",file,line);
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
