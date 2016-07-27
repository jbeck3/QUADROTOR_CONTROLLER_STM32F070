/*
 **
 **FILE: GPS INTERFACE.h
 **AUTHOR: JBECKIII
 **DATE:7/7/2016
 **PURPOSE: 
 **LEVEL 
 **
 */
//gps stuff
char str[130],str2[130],str3[130],lonss_s[10],latts_s[10],lonss_s_2[10],latts_s_2[10],lonss_s_3[10],latts_s_3[10],id_s1[10];
int gpsDisable=1;
uint8_t gps_string[100],gps_char,gps_start_str[6];
uint8_t GPS_START[] = {'$','G','P','G','G','A'};
uint8_t lat_inc,lon_inc,gpscheck;
int latint=0,lonint=0;
double latitude=39.948999,lattemp=0,lontemp=0,longitude=-76.730270,latintbuild=0,latintdec=0,lonintbuild=0,lonintdec=0;
//end gpsstuff