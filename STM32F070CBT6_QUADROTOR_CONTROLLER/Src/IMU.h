#include "stm32f0xx_hal.h"
/*
*Variables
*/
//start imu stuff
int i=0;
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
int accel_range= 2;
int gyro_range = 245;
int magn_range = 4;
float anglex=0;
//end imu stuff
float pitch,roll,YH,XH,yaw;
float PI = 3.1415;
uint16_t counter=0;
float runAvgAx=0,runAvgAy=0,runAvgAz=0;
float runSumAx=0,runSumAy=0,runSumAz=0;
float runAvgMx=0,runAvgMy=0,runAvgMz=0;
float runSumMx=0,runSumMy=0,runSumMz=0;
float derp45=0,derp0=0,derpn45=0;

/*
function prototypes
*/

void initIMU();
void readMagnometer();
void getAllVals();
float getAx();
float getAy();
float getAz();
