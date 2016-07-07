/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int dumb=0;
float distance;
int time=0;

//start imu stuff
int imuDisable = 0;
uint8_t trtx[] = {(0xA0|0xC0),0xA1,0xA2};
uint8_t trrx[3];
uint8_t mag_powerup[] = {0x22,0x00};
uint8_t mag_powerup_rd[] = {0xA2,0x00};

uint8_t data_mag[] = {(0x27|0xC0)};
uint8_t data_acc[] = 0xA7;//{(0x27|0x80)};
uint8_t data_gyr[] = {(0x17|0x80)};

uint8_t data_magback[10];
uint8_t data_accback[10];
uint8_t data_gyrback[10];

uint8_t gyroODRBWaddrW[] = {0x10,0xC0};
uint8_t gyroODRBWdata[] = {0xC0};
uint8_t gyroENaddrW[] = {0x1E,0x38};
uint8_t gyroENdata[] = {0x38};

uint8_t acceODRBWaddrW[] = {0x20,0xC0};
uint8_t acceODRBWdata[] = {0xC0};
uint8_t acceENaddrW[] = {0x1F,0x38};
uint8_t acceENdata[] = {0x38};

float magnometer_x,magnometer_y,magnometer_z;
float acceleration_x,acceleration_y,acceleration_z;
float angular_rate_x,angular_rate_y,angular_rate_z;
uint8_t gyro_status,acce_status,magn_status;
int accel_range= 5;
int gyro_range = 490;
int magn_range = 8;
float anglex=0;
//end imu stuff
//gps stuff
char str[130],str2[130],str3[130],lonss_s[10],latts_s[10],lonss_s_2[10],latts_s_2[10],lonss_s_3[10],latts_s_3[10],id_s1[10];
int gpsDisable=1;
uint8_t gps_string[100],gps_char,gps_start_str[6];
uint8_t GPS_START[] = {'$','G','P','G','G','A'};
uint8_t lat_inc,lon_inc,gpscheck;
int latint=0,lonint=0;
double latitude=39.948999,lattemp=0,lontemp=0,longitude=-76.730270,latintbuild=0,latintdec=0,lonintbuild=0,lonintdec=0;
//end gpsstuff
int i=0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */
  //mag init
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1);
   
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
    asm("nop");
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
    asm("nop"); 
    HAL_SPI_Transmit(&hspi1,&mag_powerup[0],2,100);
    asm("nop");
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
    asm("nop");
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
    asm("nop"); 
    HAL_SPI_Transmit(&hspi1,&trtx[0],1,100);
    asm("nop");
    HAL_SPI_Receive(&hspi1,&trrx[0],3,100);
    asm("nop");    
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
    asm("nop");
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
    asm("nop"); 
    HAL_SPI_Transmit(&hspi1,&mag_powerup_rd[0],1,100);
    asm("nop");
    HAL_SPI_Receive(&hspi1,&trrx[2],1,100);
    asm("nop");
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
    asm("nop");
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
    asm("nop"); 
    HAL_SPI_Transmit(&hspi1,&trtx[2],1,100);
    asm("nop");
    HAL_SPI_Receive(&hspi1,&trrx[2],1,100);
    
    //accel and gyro init
    HAL_GPIO_WritePin(GPIOA, CS_M_Pin, 1);
    
    HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,1);
    asm("nop");
    HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,0);
    asm("nop");
    HAL_SPI_Transmit(&hspi1,&gyroODRBWaddrW[0],2,100);
   // HAL_SPI_Transmit(&hspi1,&gyroODRBWdata[0],1,100);
    asm("nop");
    
    HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,1);
    asm("nop");
    HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,0);
    asm("nop");
    HAL_SPI_Transmit(&hspi1,&gyroENaddrW[0],2,100);
   // HAL_SPI_Transmit(&hspi1,&gyroENdata[0],1,100);
    asm("nop");
    
    HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,1);
    asm("nop");
    HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,0);
    asm("nop");
    HAL_SPI_Transmit(&hspi1,&acceODRBWaddrW[0],2,100);
   // HAL_SPI_Transmit(&hspi1,&acceODRBWdata[0],1,100);
    asm("nop");
    
    HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,1);
    asm("nop");
    HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,0);
    asm("nop");
    HAL_SPI_Transmit(&hspi1,&acceENaddrW[0],2,100);
   // HAL_SPI_Transmit(&hspi1,&acceENdata[0],1,100);
    asm("nop");
    
    /*
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    TIM1->PSC  = 0x0001;
    TIM1->ARR  = 0xFFFF;
    HAL_Delay(100);
    TIM1->CCR1 = 0x5000;
    //HAL_Delay(100);
    //TIM1->CCR1 = ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   
    
    
    
    if(!imuDisable){
      HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,1);
      asm("nop"); 
      
      HAL_GPIO_WritePin(GPIOA,CS_M_Pin,1);
      asm("nop"); 
      HAL_GPIO_WritePin(GPIOA,CS_M_Pin,0);
      asm("nop"); 
      HAL_SPI_Transmit(&hspi1,&data_mag[0],1,100);
      asm("nop");
      HAL_SPI_Receive(&hspi1,&data_magback[i],9,100);
      asm("nop");   
      
      magn_status  = data_magback[2];
      magnometer_x = ((float)((int16_t)(data_magback[4]<<8) + data_magback[3]))*magn_range/0xFFFF;
      magnometer_y = ((float)((int16_t)(data_magback[6]<<8) + data_magback[5]))*magn_range/0xFFFF;
      magnometer_z = ((float)((int16_t)(data_magback[8]<<8) + data_magback[7]))*magn_range/0xFFFF;
      
      HAL_GPIO_WritePin(GPIOA,CS_M_Pin,1);
      asm("nop"); 
      
      HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,1);
      asm("nop"); 
      HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,0);
      asm("nop"); 
      HAL_SPI_Transmit(&hspi1,&data_acc[0],1,100);
      asm("nop");
      HAL_SPI_Receive(&hspi1,&data_accback[0],9,100);
      asm("nop");
      
      acce_status    = data_accback[2];
      acceleration_x = ((float)((int16_t)(data_accback[4]<<8) + data_accback[3]))*accel_range/0xFFFF;
      acceleration_y = ((float)((int16_t)(data_accback[6]<<8) + data_accback[5]))*accel_range/0xFFFF;
      acceleration_z = ((float)((int16_t)(data_accback[8]<<8) + data_accback[7]))*accel_range/0xFFFF;
      
      HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,1);
      asm("nop"); 
      HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,0);
      asm("nop"); 
      HAL_SPI_Transmit(&hspi1,&data_gyr[0],1,100);
      asm("nop");
      HAL_SPI_Receive(&hspi1,&data_gyrback[0],9,100);
      asm("nop");
      
      gyro_status    = data_gyrback[2];
      angular_rate_x = ((float)((int16_t)(data_gyrback[4]<<8) + data_gyrback[3]))*gyro_range/0xFFFF;
      angular_rate_y = ((float)((int16_t)(data_gyrback[6]<<8) + data_gyrback[5]))*gyro_range/0xFFFF;
      angular_rate_z = ((float)((int16_t)(data_gyrback[8]<<8) + data_gyrback[7]))*gyro_range/0xFFFF;
    
      anglex = getAngle(anglex,angular_rate_x,0.1);
    }   
    //HAL_Delay(100);
    /*if(TIM1->CCR1 <=(0xAFFA)){TIM1->CCR1 += 0x00FF;}
    else{TIM1->CCR1 = 0x6000;}
    */
    //gps
    gpscheck = 0; 
    while((gpsDisable) && (gpscheck == 0)){
      HAL_UART_Receive(&huart1, &gps_char, 1, 1);
      if(gps_char == '\n'){
        HAL_UART_Receive(&huart1, &gps_char, 1, 1);
        if(gps_char == '$'){
          gps_string[0] = gps_char;
          HAL_UART_Receive(&huart1, &gps_char, 1, 1);
          if(gps_char == 'G'){
            gps_string[1] = gps_char;
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
          if(gps_char == 'P'){
            gps_string[2] = gps_char;
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
          if(gps_char == 'G'){
            gps_string[3] = gps_char;
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
          if(gps_char == 'G'){
            gps_string[4] = gps_char;
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
          if(gps_char == 'A'){
            gps_string[5] = gps_char;
            
            while(gps_char != ','){HAL_UART_Receive(&huart1, &gps_char, 1, 1);}
            gps_char = 0x00;
            
            //get rid of time
            while(gps_char != ','){HAL_UART_Receive(&huart1, &gps_char, 1, 1);}
            gps_char = 0x00;
            
            //now grab lat
            
            //grab first 2 of lat
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[6] = gps_char;
            latint = (gps_char-0x30);  
            
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[7] = gps_char;
            latint = ((latint*10) + (gps_char-0x30));
            
            //grab next 2 for calc
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[8] = gps_char;
            latintbuild = (gps_char-0x30); 
            
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[9] = gps_char;
            latintbuild = ((latintbuild*10) + (gps_char-0x30));
            
            //get rid of '.'
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            
            //grab decimal for calc
            latintdec=0;
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            latintdec = (gps_char-0x30); 
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            latintdec = (latintdec*10) + (gps_char-0x30); 
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            latintdec = (latintdec*10) + (gps_char-0x30); 
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            latintdec = (latintdec*10) + (gps_char-0x30); 
            lattemp = ( ((double)latintbuild +( (double)latintdec/10000.00 )) /60.00 );
            
            latitude = latint + lattemp;
            
             
            ///////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////
            
            //get to end
            while(gps_char != ','){HAL_UART_Receive(&huart1, &gps_char, 1, 1);}
            gps_char = 0x00;
            
            //get rid of n/s
            while(gps_char != ','){HAL_UART_Receive(&huart1, &gps_char, 1, 1);}
            gps_char = 0x00;
            /*//get to end
            while(gps_char != ','){HAL_UART_Receive(&huart6, &gps_char, 1, 1);}
            gps_char = 0x00;*/
            
            //now grab lon
            
            //grab first 3 of lon
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[15] = gps_char;
            //lonint = (gps_char-0x30);  
            lonint = 0.1;
            
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[16] = gps_char;
            lonint = ((lonint*10) + (gps_char-0x30));
            
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[17] = gps_char;
            lonint = ((lonint*10) + (gps_char-0x30));
            
            //grab next 2 for calc
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[18] = gps_char;
            lonintbuild = (gps_char-0x30); 
            
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            gps_string[19] = gps_char;
            lonintbuild = ((lonintbuild*10) + (gps_char-0x30));
            
            //get rid of '.'
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            
            //grab decimal for calc
            latintdec=0;
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            lonintdec = (gps_char-0x30); 
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            lonintdec = (lonintdec*10) + (gps_char-0x30); 
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            lonintdec = (lonintdec*10) + (gps_char-0x30); 
            HAL_UART_Receive(&huart1, &gps_char, 1, 1);
            //lonintdec = (lonintdec*10) + (gps_char-0x30); 
            lontemp = ( ((double)lonintbuild +( (double)lonintdec/1000.00 )) /60.00 );
            
            longitude = (lonint + lontemp)*(-1);
            gpscheck=1;
            ///////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////
            
            ////////////}
            
            /*
            gps_char = 0x00;
            //while(gps_char != ','){HAL_UART_Receive(&huart6, &gps_char, 1, 1);}
            lon_inc=0;
            while(gps_char != ','){
              HAL_UART_Receive(&huart6, &gps_char, 1, 1);
              gps_string[6+lat_inc+lon_inc] = gps_char;
              lon_inc++;
            }*/
            //HAL_UART_Receive(&huart6, &gps_string[10], 10, 100);
            //while(gps_char != 'N' || gps_char != 'S' ){HAL_UART_Receive(&huart6, &gps_char, 1, 1);}
            /*
            while(gps_char != ','){HAL_UART_Receive(&huart6, &gps_char, 1, 1);}
            HAL_UART_Receive(&huart6, &gps_string[20], 10, 100);
            //while(gps_char != 'N' || gps_char != 'S' ){HAL_UART_Receive(&huart6, &gps_char, 1, 1);}
            HAL_UART_Receive(&huart6, &gps_char, 1, 1);
            HAL_UART_Receive(&huart6, &gps_string[30], 17, 100);
            while(gps_char != ','){HAL_UART_Receive(&huart6, &gps_char, 1, 1);}
            //while(gps_char != 'W' || gps_char != 'E' ){HAL_UART_Receive(&huart6, &gps_char, 1, 1);}
            */
            
            gps_char='P';
        
        /*if(gps_char == '$'){
          HAL_UART_Receive(&huart6, &gps_char, 1, 1);
          if(gps_char == 'G'){*/
            
                  }
                }
              }
            }
          }
        }               
      }  
    }//endgps
  
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void filter(){
	uint8_t counter=0;
	float runAvg=0;
	float runSum=0;
	
	if(counter<255){
		counter++;
		runSum += getNewVal();
		runAvg = runSum/counter;
	}else{
		counter=1;
		runSum = runAvg;
	}
	
	//this is a change
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
