#ifndef GPS_HC
#define GPS_HC

#define TRIG 23
#define ECHO 24
#define TIMEOUT 999 /* any value other than LOW or HIGH */

struct latLong {
	int degrees;
	float minutes;
	char direction;
};

struct GPS_Data {
  float time;
  struct latLong latitude;
  struct latLong longitude;
  float altitude;
  char navStat[3];
  float hAcc;
  float vAcc;
  float SOG;
  float COG;
  float vVel;
  float HDOP;
  float VDOP;
  float TDOP;
  int nSat;
};

int waitforpin(int pin, int level, int timeout);
float distanza_sensore(int echo, int trig, float pre_dist, int* err);
void gpsConfig(int fd);
void gpsPrint(const struct GPS_Data* d);
void scompatta(const char* s, struct GPS_Data* d);

#endif
