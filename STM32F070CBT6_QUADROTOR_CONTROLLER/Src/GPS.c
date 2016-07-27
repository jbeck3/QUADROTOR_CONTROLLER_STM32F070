/*
 **
 **FILE: GPS INTERFACE.c
 **AUTHOR: JBECKIII
 **DATE:7/7/2016
 **PURPOSE: 
 **LEVEL 
 **
 */

#include "usart.h"
#include "GPS.h"
/*
 *
 *void getLocation()
 *
 */
void getLocation(){
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