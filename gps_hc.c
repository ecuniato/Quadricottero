#include "incl/gps_hc.h"
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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

      if (digitalRead(pin) == level) done = 1;
   }
   return micros;
}

float distanza_sensore(int echo, int trig)
{
	  float distance;
	  int pulsewidth;
	  int i;
	 /* trigger reading */

      digitalWrite(trig, HIGH);
      waitforpin(echo, TIMEOUT, 10); /* wait 10 microseconds */
      digitalWrite(trig, LOW);

      /* wait for reading to start */
      waitforpin(echo, HIGH, 5000); /* 5 ms timeout */

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
         }
      }
      else
      {
         /* sensor not firing code */
         printf("sensor didn't fire\n");		 
      }
	  //DEBUG: delay(2000);
	  if (distance > 400) distance = 400;
	  return distance;
}
