/*
 **
 **FILE: INERTIAL MEASUREMENT INTERFACE
 **AUTHOR: JBECKIII
 **DATE:6/17/2016
 **PURPOSE: PROVIDE INTERFACE FOR AN INERTIAL MEASUREMENT UNIT WITH HARDWARE
 **LEVEL INTERFACE AND RELEVANT FILTER IMPLEMENTATIONS
 **
 */

/*Includes*/
#include "IMU.h"
#include "spi.h"
#include "math.h"

/* 
void initIMU()
param: none
returns: none
function: initialize IMU registers
 */
void initIMU(){
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
	asm("nop");

	HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,1);
	asm("nop");
	HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,0);
	asm("nop");
	HAL_SPI_Transmit(&hspi1,&gyroENaddrW[0],2,100);
	asm("nop");

	HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,1);
	asm("nop");
	HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,0);
	asm("nop");
	HAL_SPI_Transmit(&hspi1,&acceODRBWaddrW[0],2,100);
	asm("nop");

	HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,1);
	asm("nop");
	HAL_GPIO_WritePin(GPIOA, CS_AG_Pin,0);
	asm("nop");
	HAL_SPI_Transmit(&hspi1,&acceENaddrW[0],2,100);
	asm("nop");
}

/* 
void readMagnometer()
param: none
returns: none
function: to read magnometer by sending spi read command, and then reading data
 */
void readMagnometer(){
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
	magnometer_x = ((float)((int16_t)(data_magback[4]<<8) + data_magback[3]))*magn_range/0x7FFF;
	magnometer_y = ((float)((int16_t)(data_magback[6]<<8) + data_magback[5]))*magn_range/0x7FFF;
	magnometer_z = ((float)((int16_t)(data_magback[8]<<8) + data_magback[7]))*magn_range/0x7FFF;
	if(counter<AVERAGE_MAX){
		counter++;
		runSumMx += magnometer_x;
		runSumMy += magnometer_y;
		runSumMz += magnometer_z;
		runAvgMx = runSumMx/counter;
		runAvgMy = runSumMy/counter;
		runAvgMz = runSumMz/counter;
	}else{        
		counter=1;
		runSumMx = runAvgMx;
		runSumMy = runAvgMy;
		runSumMz = runAvgMz;
	}

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
void getAccel()
param: none
returns: none
function: get accelerations
 */
void getAccel(){            
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
  acceleration_x = ((float)((int16_t)(data_accback[4]<<8) + data_accback[3]))*accel_range/0x7FFF;
  acceleration_y = ((float)((int16_t)(data_accback[6]<<8) + data_accback[5]))*accel_range/0x7FFF;
  acceleration_z = ((float)((int16_t)(data_accback[8]<<8) + data_accback[7]))*accel_range/0x7FFF;
  
  
  if(counter<AVERAGE_MAX){
          counter++;                
          runSumAx += acceleration_x;
          runSumAy += acceleration_y;
          runSumAz += acceleration_z;    
  
          runAvgAx = runSumAx/counter;
          runAvgAy = runSumAy/counter;
          runAvgAz = runSumAz/counter;
  
  }else{        
          counter=1;                
          runSumAx = runAvgAx;
          runSumAy = runAvgAy;
          runSumAz = runAvgAz;
  }
}

/*
void getGyro()
param: none
returns: none
function: get angular rates
 */
void getGyro(){   
	HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,1);
	asm("nop"); 
	HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,0);
	asm("nop"); 
	HAL_SPI_Transmit(&hspi1,&data_gyr[0],1,100);
	asm("nop");
	HAL_SPI_Receive(&hspi1,&data_gyrback[0],9,100);
	asm("nop");

	gyro_status    = data_gyrback[2];
	angular_rate_x = ((float)((int16_t)(data_gyrback[4]<<8) + data_gyrback[3]))*gyro_range/0x7FFF;
	angular_rate_y = ((float)((int16_t)(data_gyrback[6]<<8) + data_gyrback[5]))*gyro_range/0x7FFF;
	angular_rate_z = ((float)((int16_t)(data_gyrback[8]<<8) + data_gyrback[7]))*gyro_range/0x7FFF;   

	if(counter<AVERAGE_MAX){
		counter++;                
		runSumGx += angular_rate_x;
		runSumGy += angular_rate_y;
		runSumGz += angular_rate_z;                
		runAvgGx = runSumGx/counter;
		runAvgGy = runSumGy/counter;
		runAvgGz = runSumGz/counter;
	}else{        
		counter=1;
		runSumGx = runAvgGx;
		runSumGy = runAvgGy;
		runSumGz = runAvgGz;
	}
}
/*
void getAllVals()
param: none
returns: none
function: 
 */
void getAllVals(){      
	//readMagnometer();
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
	acceleration_x = ((float)((int16_t)(data_accback[4]<<8) + data_accback[3]))*accel_range/0x7FFF;
	acceleration_y = ((float)((int16_t)(data_accback[6]<<8) + data_accback[5]))*accel_range/0x7FFF;
	acceleration_z = ((float)((int16_t)(data_accback[8]<<8) + data_accback[7]))*accel_range/0x7FFF;
/*
	HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,1);
	asm("nop"); 
	HAL_GPIO_WritePin(GPIOA,CS_AG_Pin,0);
	asm("nop"); 
	HAL_SPI_Transmit(&hspi1,&data_gyr[0],1,100);
	asm("nop");
	HAL_SPI_Receive(&hspi1,&data_gyrback[0],9,100);
	asm("nop");

	gyro_status    = data_gyrback[2];
	angular_rate_x = ((float)((int16_t)(data_gyrback[4]<<8) + data_gyrback[3]))*gyro_range/0x7FFF;
	angular_rate_y = ((float)((int16_t)(data_gyrback[6]<<8) + data_gyrback[5]))*gyro_range/0x7FFF;
	angular_rate_z = ((float)((int16_t)(data_gyrback[8]<<8) + data_gyrback[7]))*gyro_range/0x7FFF;
*/

	if(counter<AVERAGE_MAX){
		counter++;

		runSumAx += acceleration_x;
		runSumAy += acceleration_y;
		runSumAz += acceleration_z;
		runSumMx += magnometer_x;
		runSumMy += magnometer_y;
		runSumMz += magnometer_z;

		runAvgAx = runSumAx/counter;
		runAvgAy = runSumAy/counter;
		runAvgAz = runSumAz/counter;
		runAvgMx = runSumMx/counter;
		runAvgMy = runSumMy/counter;
		runAvgMz = runSumMz/counter;
	}else{        
		counter=1;

		runSumAx = runAvgAx;
		runSumAy = runAvgAy;
		runSumAz = runAvgAz;
		runSumMx = runAvgMx;
		runSumMy = runAvgMy;
		runSumMz = runAvgMz;
	}

	pitch =  atan2f(runAvgAx,sqrt(runAvgAy*runAvgAy + runAvgAz*runAvgAz))*180/PI;
	roll  =  atan2f(runAvgAy,(runAvgAz))*180/PI;
	YH = runAvgMy*cosf(roll)+runAvgMz*sinf(roll);
	XH = (-runAvgMx*cosf(pitch))+(runAvgMy*sinf(pitch)*sinf(roll))+(runAvgMz*sinf(pitch)*cosf(roll));
	yaw = atan2f(-YH,XH)*180/PI;
}

float *getRoll(){
	return &roll;
}
float *getPitch(){
	return &pitch;
}
float *getYaw(){
	return &yaw;
}

void filter(){


	if(counter<255){
		counter++;
		//runSum += getNewVal();
		//runAvg = runSum/counter;
	}else{
		counter=1;
		//runSum = runAvg;
	}	
	//this is a change	
}



/*
 **
 **REVISION INFORMATION:
 **6/17/2016: INITIAL FILE CREATION
 **7/13/2016: Add of seperate functions
 */