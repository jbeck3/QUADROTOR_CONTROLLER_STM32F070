/*
**
**FILE: INERTIAL MEASUREMENT INTERFACE
**AUTHOR: JBECKIII
**DATE:6/17/2016
**PURPOSE: PROVIDE INTERFACE FOR AN INERTIAL MEASUREMENT UNIT WITH HARDWARE
**LEVEL INTERFACE AND RELEVANT FILTER IMPLEMENTATIONS
**
*/
#include "spi.h"

/* 
void readMagnometer()
param: none
returns: none
function: to read magnometer by sending spi read command, and then reading data
*/
void readMagnometer(){
  /*
      HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,1);
      HAL_GPIO_WritePin(GPIOA,CS_M_Pin,1);
      HAL_GPIO_WritePin(GPIOA,CS_M_Pin,0);
      HAL_SPI_Transmit(&hspi1,&data_mag[0],1,100);
      HAL_SPI_Receive(&hspi1,&data_back[i],7,100);*/
      
}

/*
void writeMagnometer()
param: none
returns: none
function: write data to magnometer over spi
*/
void writeMagnometer(){

}

/*
void initMagnometer()
param: none
returns: none
function: initialize the magnometer for normal operation
*/
void initMagnometer(){
  /*
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
    */
}






/*
**
**REVISION INFORMATION:
**6/17/2016: INITIAL FILE CREATION
**
*/