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
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "dma2d.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "sai.h"
#include "sdmmc.h"
#include "gpio.h"
#include "fmc.h"

/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_audio.h"
#include "rekordbox.h"
#include "mp3dec.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TS_StateTypeDef ts_State;
uint8_t touch;
uint8_t *ch;
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
DIR root;
char SDPath[4]; /* SD card logical drive path */
char wave;
uint32_t wavecolor;
uint32_t bytesread;
uint32_t i = 0, k = 0;
uint16_t file_pos = 0;
uint32_t file_pos_wide = 0;
uint8_t lowp_wavebuffer[400];
uint8_t wavebuffer[60000];

extern RekordboxTypeDef rekordbox;

// MP3 Variables
MP3FrameInfo mp3FrameInfo;
HMP3Decoder	hMP3Decoder;

uint8_t g_Mp3InBuffer[MP3_INBUF_SIZE];
static uint16_t g_pMp3OutBuffer[MAX_NCHAN * MAX_NGRAN * MAX_NSAMP];
static uint16_t* g_pMp3OutBufferPtr = NULL;
uint16_t g_pMp3DmaBuffer[MP3_DMA_BUFFER_SIZE];
uint16_t* g_pMp3DmaBufferPtr = NULL;
UINT bEof = 0;
UINT bOutOfData = 0;
int nResult = 0;
BYTE* pInData = g_Mp3InBuffer;
UINT unInDataLeft = 0;
uint32_t unDmaBufMode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static UINT Mp3FillReadBuffer(BYTE* pInData, UINT unInDataLeft, FIL* pInFile);
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist, uint32_t unArtistSize, char* pszTitle, uint32_t unTitleSize);
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
  MX_SAI2_Init();
  MX_SDMMC1_SD_Init();
  MX_FMC_Init();
  MX_FATFS_Init();
  MX_LTDC_Init();
  MX_I2C3_Init();
  MX_DMA2D_Init();

  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS_0);
  //BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS_1);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  //BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  //BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  //BSP_LCD_SetFont(&Font12);

  //BSP_TS_Init(480, 272);
  /*while(ts_State.touchDetected == 0) {
 	  BSP_TS_GetState(&ts_State);
 	  touch = ts_State.touchDetected;
 	 }
   ch[0] = touch + '0';
   ch[1] = 0;

   HAL_Delay(2000);*/

  g_pMp3DmaBufferPtr = g_pMp3DmaBuffer;
  int nDecodeRes = ERR_MP3_NONE;

  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 60, AUDIO_FREQUENCY_22K);
  hMP3Decoder = MP3InitDecoder();
  HAL_Delay(500);

  if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
  {
	  DecodeRekordboxFiles("a");
 	  if(f_open(&MyFile, rekordbox.filename, FA_READ) == FR_OK)
 	  {
 		  while(f_read(&MyFile, &g_Mp3InBuffer, 4096, (void *)&bytesread) != FR_OK);
 		  BSP_AUDIO_OUT_Play((uint16_t*)&g_Mp3InBuffer, 4096);
 	  }
 	  /*if(f_open(&MyFile, "1.mp3", FA_READ) == FR_OK)
 	  {
 		  // Read ID3v2 Tag
 		  char szArtist[120];
 		  char szTitle[120];
 		  Mp3ReadId3V2Tag(&MyFile, szArtist, sizeof(szArtist), szTitle, sizeof(szTitle));
 		  UINT unFramesDecoded = 0;
 		  do
 		  {
 			  if(unInDataLeft < (2 * MAINBUF_SIZE) && (!bEof))
 			  {
 				  UINT unRead = Mp3FillReadBuffer(pInData, unInDataLeft, &MyFile);
 				  unInDataLeft += unRead;
 				  pInData = g_Mp3InBuffer;
 				  if(unRead == 0)
 				  {
 					  bEof = 1;
 				  }
 			  }

 			  // find start of next MP3 frame - assume EOF if no sync found
 			  int nOffset = MP3FindSyncWord(pInData, unInDataLeft);
 			  if(nOffset < 0)
 			  {
 				  bOutOfData = 1;
 				  break;
 			  }
 			  pInData += nOffset;
 			  unInDataLeft -= nOffset;

 			  // decode one MP3 frame - if offset < 0 then bytesLeft was less than a full frame
 			  nDecodeRes = MP3Decode(hMP3Decoder, &pInData, (int*)&unInDataLeft, (short*)g_pMp3OutBuffer, 0);
 			  switch(nDecodeRes)
 			  {
 		      	  case ERR_MP3_NONE:
 		      	  {
 		      		  MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
 		      		  if(unFramesDecoded == 0)
 		      		  {
 		      			  if((mp3FrameInfo.samprate > 48000) || (mp3FrameInfo.bitsPerSample != 16) || (mp3FrameInfo.nChans != 2))
 		      			  {
 		      				  nResult = -5;
 		      				  break;
 		      			  }
 		      		  }
 		      		  if((unFramesDecoded) % 100 == 0)
 		      		  {
 		      		  }
 		      		  unFramesDecoded++;
 		      		  g_pMp3OutBufferPtr = g_pMp3OutBuffer;

 		      		  uint32_t unOutBufferAvail= mp3FrameInfo.outputSamps;
 		      		  while(unOutBufferAvail > 0)
 		      		  {
 		      			  // fill up the whole dma buffer
 		      			  uint32_t unDmaBufferSpace = 0;
 		      			  if(unDmaBufMode == 0)
 		      			  {
 		      				  // fill the whole buffer
 		      				  // dma buf ptr was reset to beginning of the buffer
 		      				  unDmaBufferSpace = g_pMp3DmaBuffer + MP3_DMA_BUFFER_SIZE - g_pMp3DmaBufferPtr;
 		      			  }
 		      			  else if(unDmaBufMode == 1)
 		      			  {
 		      				  // fill the first half of the buffer
 		      				  // dma buf ptr was reset to beginning of the buffer
 		      				  unDmaBufferSpace = g_pMp3DmaBuffer + (MP3_DMA_BUFFER_SIZE / 2) - g_pMp3DmaBufferPtr;
 		      			  }
 		      			  else
 		      			  {
 		      				  // fill the last half of the buffer
 		      				  // dma buf ptr was reset to middle of the buffer
 		      				  unDmaBufferSpace = g_pMp3DmaBuffer + MP3_DMA_BUFFER_SIZE - g_pMp3DmaBufferPtr;
 		      			  }
 		      			  uint32_t unCopy = unDmaBufferSpace > unOutBufferAvail ? unOutBufferAvail : unDmaBufferSpace;
 		      			  if(unCopy > 0)
 		      			  {
 		      				  memcpy(g_pMp3DmaBufferPtr, g_pMp3OutBufferPtr, unCopy * sizeof(uint16_t));
 		      				  unOutBufferAvail -= unCopy;
 		      				  g_pMp3OutBufferPtr += unCopy;
 		      				  unDmaBufferSpace -= unCopy;
 		      				  g_pMp3DmaBufferPtr += unCopy;
 		      			  }
 		      			  if(unDmaBufferSpace == 0)
 		      			  {
 		      				  // dma buffer full
 		      				  // see if this was the first run
 		      				  if(unDmaBufMode == 0)
 		      				  {
 		      					  // on the first buffer fill up,
 		      					  // start the dma transfer
 		      					  BSP_AUDIO_OUT_Play(g_pMp3DmaBuffer, MP3_DMA_BUFFER_SIZE * sizeof(uint16_t));
 		      				  }
 		      				  // we must wait for the dma stream tx interrupt here
 		      				  while(unDmaBufMode == 0);
 		      			  }
 		      		  }
 		      		  break;
 		      	  }
 		      	  case ERR_MP3_MAINDATA_UNDERFLOW:
 		      	  {
 		      		  // do nothing - next call to decode will provide more mainData
 		      		  break;
 		      	  }
 		      	  case ERR_MP3_FREE_BITRATE_SYNC:
 		      	  {
 		      		  break;
 		      	  }
 		      	  case ERR_MP3_INDATA_UNDERFLOW:
 		      	  {
 		      		  bOutOfData = 1;
 		      		  break;
 		      	  }
 		      	  default:
 		      	  {
 		      		  bOutOfData = 1;
 		      		  break;
 		      	  }
 			  }
 		  }
 		  while((!bOutOfData) && (nResult == 0));
 		  MP3FreeDecoder(hMP3Decoder);
 		  BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
 		  f_close(&MyFile);
 	  }*/
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(file_pos > rekordbox.spectrum_size) while(1);

	  BSP_LCD_DrawSpectra(40, 250, wavebuffer);
	  BSP_LCD_SetTextColor(LCD_COLOR_RED);
	  BSP_LCD_DrawVLine(239, 60, 80);
	  BSP_LCD_DrawVLine(240, 60, 80);
	  BSP_LCD_DrawVLine(241, 60, 80);
	  BSP_LCD_DrawVLine(file_pos+40, 220, 40);
	  BSP_LCD_DrawVLine(file_pos+41, 220, 40);

	  HAL_Delay(20);

	  BSP_LCD_Clear(LCD_COLOR_BLACK);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 429;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLI2SDivQ = 19;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
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

/*
 * Taken from
 * http://www.mikrocontroller.net/topic/252319
 */
static uint32_t Mp3ReadId3V2Text(FIL* pInFile, uint32_t unDataLen, char* pszBuffer, uint32_t unBufferSize)
{
	UINT unRead = 0;
	BYTE byEncoding = 0;
	if((f_read(pInFile, &byEncoding, 1, &unRead) == FR_OK) && (unRead == 1))
	{
		unDataLen--;
		if(unDataLen <= (unBufferSize - 1))
		{
			if((f_read(pInFile, pszBuffer, unDataLen, &unRead) == FR_OK) ||
					(unRead == unDataLen))
			{
				if(byEncoding == 0)
				{
					// ISO-8859-1 multibyte
					// just add a terminating zero
					pszBuffer[unDataLen] = 0;
				}
				else if(byEncoding == 1)
				{
					// UTF16LE unicode
					uint32_t r = 0;
					uint32_t w = 0;
					if((unDataLen > 2) && (pszBuffer[0] == 0xFF) && (pszBuffer[1] == 0xFE))
					{
						// ignore BOM, assume LE
						r = 2;
					}
					for(; r < unDataLen; r += 2, w += 1)
					{
						// should be acceptable for 7 bit ascii
						pszBuffer[w] = pszBuffer[r];
					}
					pszBuffer[w] = 0;
				}
			}
			else
			{
				return 1;
			}
		}
		else
		{
			// we won't read a partial text
			if(f_lseek(pInFile, f_tell(pInFile) + unDataLen) != FR_OK)
			{
				return 1;
			}
		}
	}
	else
	{
		return 1;
	}
	return 0;
}

/*
 * Taken from
 * http://www.mikrocontroller.net/topic/252319
 */
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist, uint32_t unArtistSize, char* pszTitle, uint32_t unTitleSize)
{
	pszArtist[0] = 0;
	pszTitle[0] = 0;

	BYTE id3hd[10];
	UINT unRead = 0;
	if((f_read(pInFile, id3hd, 10, &unRead) != FR_OK) || (unRead != 10))
	{
		return 1;
	}
	else
	{
		uint32_t unSkip = 0;
		if((unRead == 10) &&
				(id3hd[0] == 'I') &&
				(id3hd[1] == 'D') &&
				(id3hd[2] == '3'))
		{
			unSkip += 10;
			unSkip = ((id3hd[6] & 0x7f) << 21) | ((id3hd[7] & 0x7f) << 14) | ((id3hd[8] & 0x7f) << 7) | (id3hd[9] & 0x7f);

			// try to get some information from the tag
			// skip the extended header, if present
			uint8_t unVersion = id3hd[3];
			if(id3hd[5] & 0x40)
			{
				BYTE exhd[4];
				f_read(pInFile, exhd, 4, &unRead);
				size_t unExHdrSkip = ((exhd[0] & 0x7f) << 21) | ((exhd[1] & 0x7f) << 14) | ((exhd[2] & 0x7f) << 7) | (exhd[3] & 0x7f);
				unExHdrSkip -= 4;
				if(f_lseek(pInFile, f_tell(pInFile) + unExHdrSkip) != FR_OK)
				{
					return 1;
				}
			}
			uint32_t nFramesToRead = 2;
			while(nFramesToRead > 0)
			{
				char frhd[10];
				if((f_read(pInFile, frhd, 10, &unRead) != FR_OK) || (unRead != 10))
				{
					return 1;
				}
				if((frhd[0] == 0) || (strncmp(frhd, "3DI", 3) == 0))
				{
					break;
				}
				char szFrameId[5] = {0, 0, 0, 0, 0};
				memcpy(szFrameId, frhd, 4);
				uint32_t unFrameSize = 0;
				uint32_t i = 0;
				for(; i < 4; i++)
				{
					if(unVersion == 3)
					{
						// ID3v2.3
						unFrameSize <<= 8;
						unFrameSize += frhd[i + 4];
					}
					if(unVersion == 4)
					{
						// ID3v2.4
						unFrameSize <<= 7;
						unFrameSize += frhd[i + 4] & 0x7F;
					}
				}

				if(strcmp(szFrameId, "TPE1") == 0)
				{
					// artist
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszArtist, unArtistSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else if(strcmp(szFrameId, "TIT2") == 0)
				{
					// title
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszTitle, unTitleSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else
				{
					if(f_lseek(pInFile, f_tell(pInFile) + unFrameSize) != FR_OK)
					{
						return 1;
					}
				}
			}
		}
		if(f_lseek(pInFile, unSkip) != FR_OK)
		{
			return 1;
		}
	}

	return 0;
}

static UINT Mp3FillReadBuffer(BYTE* pInData, UINT unInDataLeft, FIL* pInFile)
{

  // move last, small chunk from end of buffer to start, then fill with new data
  memmove(g_Mp3InBuffer, pInData, unInDataLeft);

  UINT unSpaceLeft = MP3_INBUF_SIZE - unInDataLeft;
  UINT unRead = 0;
  FRESULT fr = f_read(pInFile, g_Mp3InBuffer + unInDataLeft, unSpaceLeft, &unRead);
  if(fr != FR_OK)
  {
    unRead = 0;
  }
  if(unRead < unSpaceLeft)
  {
    // zero-pad to avoid finding false sync word after last frame (from old data in readBuf)
    memset(g_Mp3InBuffer + unInDataLeft + unRead, unSpaceLeft - unRead, 0);
  }

  return unRead;
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
