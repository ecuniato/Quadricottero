#ifndef GPS_HC
#define GPS_HC

#define TRIG 23
#define ECHO 24
#define TIMEOUT 999 /* any value other than LOW or HIGH */

int waitforpin(int pin, int level, int timeout);
float distanza_sensore(int echo, int trig);


#endif
