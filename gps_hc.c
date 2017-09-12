#include "incl/gps_hc.h"
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void gpsConfig(int fd) {
	int i=0;
	
	char rate_buffer[]={0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x00,0x00};
	char CK_A=0, CK_B=0;
	for (i=0; i<10; i++) {
		CK_A = CK_A + rate_buffer[i];
 		CK_B = CK_B + CK_A;
	}
	char rate_config[]={0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x00,0x00,CK_A,CK_B,'\0'};
	write(fd, rate_config,sizeof(rate_config));
	delay(500);
	
	char gsv_buffer[]={0x06,0x01,0x03,0x00,0xF0,0x03,0xFF,'\0'};
	CK_A=0; CK_B=0;
	for (i=0; i<7; i++) {
		CK_A = CK_A + gsv_buffer[i];
 		CK_B = CK_B + CK_A;
	}
	char gsv_config[]={0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0xFF,CK_A,CK_B,'\0'};
	write(fd, gsv_config,sizeof(gsv_config));
	delay(500);
	
	char gsa_buffer[]={0x06,0x01,0x03,0x00,0xF0,0x02,0xFF,'\0'};
	CK_A=0; CK_B=0;
	for (i=0; i<7; i++) {
		CK_A = CK_A + gsa_buffer[i];
 		CK_B = CK_B + CK_A;
	}
	char gsa_config[]={0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0xFF,CK_A,CK_B,'\0'};
	write(fd, gsa_config,sizeof(gsa_config));
	delay(500);
	
	char gga_buffer[]={0x06,0x01,0x03,0x00,0xF0,0x00,0xFF,'\0'};
	CK_A=0; CK_B=0;
	for (i=0; i<7; i++) {
		CK_A = CK_A + gga_buffer[i];
 		CK_B = CK_B + CK_A;
	}
	char gga_config[]={0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0xFF,CK_A,CK_B,'\0'};
	write(fd, gga_config,sizeof(gga_config));
	delay(500);
	
	char vtg_buffer[]={0x06,0x01,0x03,0x00,0xF0,0x05,0xFF,'\0'};
	CK_A=0; CK_B=0;
	for (i=0; i<7; i++) {
		CK_A = CK_A + vtg_buffer[i];
 		CK_B = CK_B + CK_A;
	}
	char vtg_config[]={0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0xFF,CK_A,CK_B,'\0'};
	write(fd, vtg_config,sizeof(vtg_config));
	delay(500);
	
	char rmc_buffer[]={0x06,0x01,0x03,0x00,0xF0,0x04,0xFF,'\0'};
	CK_A=0; CK_B=0;
	for (i=0; i<7; i++) {
		CK_A = CK_A + rmc_buffer[i];
 		CK_B = CK_B + CK_A;
	}
	char rmc_config[]={0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0xFF,CK_A,CK_B,'\0'};
	write(fd, rmc_config,sizeof(rmc_config));
	delay(500);
	
	char gll_buffer[]={0x06,0x01,0x03,0x00,0xF0,0x01,0xFF,'\0'};
	CK_A=0; CK_B=0;
	for (i=0; i<7; i++) {
		CK_A = CK_A + gll_buffer[i];
 		CK_B = CK_B + CK_A;
	}
	char gll_config[]={0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0xFF,CK_A,CK_B,'\0'};
	write(fd, gll_config,sizeof(gll_config));
	delay(500);
	
}

void gpsPrint(const struct GPS_Data* d) {
	printf("\nTime= %f \nLatitude= d:%d - m:%f %c \nLongitude= d:%d - m:%f %c \nAltitude= %f m",d->time,d->latitude.degrees,d->latitude.minutes,d->latitude.direction,d->longitude.degrees,d->longitude.minutes,d->longitude.direction,d->altitude);
	printf("\nNavigation status= %s \nHorizontal accuracy= %f m \nVertical accuracy= %f m \nSpeed over ground= %f Km/h \nCourse over ground= %f Degrees",d->navStat,d->hAcc,d->vAcc,d->SOG,d->COG);
	printf("\nVertical velocity= %f m/s \nHorizontal diluition of precision= %f \nVertical diluition of precision= %f \nTime diluition of precision= %f \nGPS satellites used= %d \n",d->vVel,d->HDOP,d->VDOP,d->TDOP,d->nSat);
}

void scompatta(const char* s, struct GPS_Data* d) {
	int i=7, j=0;
 int elem=0;
 char tmp[50],tmp1[5];
 
 //Time
 for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->time = atof(tmp);
  
  //Latitude
  tmp[0]=s[i]; i++;
  tmp[1]=s[i]; i++;
  tmp[2]='\0';
  d->latitude.degrees = atoi(tmp);
  
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
  j=0; i++;
  d->latitude.minutes = atof(tmp);
  d->latitude.direction = s[i]; i++; i++;
  
  //Longitude
  tmp[0]=s[i]; i++;
  tmp[1]=s[i]; i++;
  tmp[2]='\0';
  d->longitude.degrees = atoi(tmp);
  
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
  j=0; i++;
  d->longitude.minutes = atof(tmp);
  d->longitude.direction = s[i]; i++; i++;
  
  //Altitude m
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->altitude = atof(tmp);
  
  //Navigation status
  d->navStat[0]=s[i]; i++;
  d->navStat[1]=s[i]; i++; i++;
  d->navStat[2]='\0';
  
  //Horizontal accuracy estimate m
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->hAcc = atof(tmp);
  
  //Vertical accuracy estimate m
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->vAcc = atof(tmp);
  
  //Speed over ground Km/h
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->SOG = atof(tmp);
  
  //Course over ground Degrees
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->COG = atof(tmp);
  
  //Vertical velocity m/s 
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->vVel = atof(tmp);
  
  //Age of most recent DGPS corrections (Not used)
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++; 
	
	//Horizontal Diluition of Precision
	for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->HDOP = atof(tmp);
  
  //Vertical Diluition of Precision
	for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->VDOP = atof(tmp);
  
  //Horizontal Diluition of Precision
	for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->TDOP = atof(tmp);
  
  //Number of satellites used
  for (; s[i]!=','; i++, j++)
    tmp[j]=s[i];
    
  tmp[j]='\0';
	j=0; i++;    
  d->nSat = atoi(tmp);
  
}

int waitforpin(int pin, int level, int timeout)
{
   struct timeval now, start;
   int done;
   long micros;

   gettimeofday(&start, NULL);

   micros = 0;

   done=0;

   while (!done)
   {
      gettimeofday(&now, NULL);
      if (now.tv_sec > start.tv_sec) micros = 1000000L; else micros = 0;
      micros = micros + (now.tv_usec - start.tv_usec);

      if (micros > timeout) done=1;

      if ((level!=TIMEOUT) && (digitalRead(pin) == level)) done = 1;
   }
   return micros;
}

float distanza_sensore(int echo, int trig, float pre_dist, int* err)
{
	  float distance=pre_dist;
	  int pulsewidth;
	  int i;
	 /* trigger reading */

      digitalWrite(trig, HIGH);
      waitforpin(echo, TIMEOUT, 10); /* wait 10 microseconds */
      digitalWrite(trig, LOW);

      /* wait for reading to start */
      waitforpin(echo, HIGH, 10000); /* 10 ms timeout */

      if (digitalRead(echo)  == HIGH)
      {
         pulsewidth = waitforpin(echo, LOW, 60000L); /* 60 ms timeout */

         if (digitalRead(echo) == LOW)
         {
            /* valid reading code */
	        distance = pulsewidth / 58;
         //DEBUG: printf("%s : echo at %d micros -- Distance: %d\n", sensore, pulsewidth, distance);
         }
         else
         {
            /* no object detected code */
            printf("echo tiimed out\n");
            *err=1;
         }
      }
      else
      {
         /* sensor not firing code */
         printf("sensor didn't fire\n");
				 *err=1;		 
      }
	  //DEBUG: delay(2000);
	  if (distance > 400) distance = 400;
	  return distance;
}
