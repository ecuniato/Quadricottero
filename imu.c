#include "incl/imu.h"
#include "incl/MadgwickAHRS.h"
#include <stdint.h>
#define SWAP_2BYTES(x) (((x & 0xFFFF) >> 8) | ((x & 0xFF) << 8))
int cc=41,cc1=5;
double sensors_offsets[20];
int sensors_raw_values[12];
double factory_cal[3];
float dest1[3]={0,0,0}, dest2[3]={1,1,1};
int fd, fd_ak, fd_bmp;

// BMP280 compensation parameters
int  dig_T1, dig_P1;
int dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void init_imu()
{
  //MPU CONFIGURATION
  
	// Resetta tutti i registri ai valori iniziali
	wiringPiI2CWriteReg8 ( fd,  MPU9250_PWR_MGMT_1,  0x80);
	delay(200);

	// Sveglia l'MPU e imposta il clock sulla sorgente migliore
	wiringPiI2CWriteReg8 ( fd,  MPU9250_PWR_MGMT_1,  0x01);
  delay(10);
	// Abilita gyro e acc
	wiringPiI2CWriteReg8 (fd, MPU9250_PWR_MGMT_2, 0x00);
  delay(10);
	// Filtro passa-basso a 92Hz - 3.0ms sul gyro
 // this limits the sample rate to 1000 Hz for gyro and temp
	wiringPiI2CWriteReg8 (fd, MPU9250_CONFIG, 0x02);
  delay(10);	
	// Imposta la frequenza di misurazione a 1KHz
	wiringPiI2CWriteReg8 (fd, MPU9250_SMPLRT_DIV, 0x00);
  delay(10);	
	// Imposta la scala del gyro a 250°/s e attiva il filtro passa-basso
	wiringPiI2CWriteReg8 (fd, MPU9250_GYRO_CONFIG, 0x00);
  delay(10);
	// Imposta la scala dell'accelerometro a 2g
	wiringPiI2CWriteReg8 (fd, MPU9250_ACCEL_CONFIG, 0x00);
  delay(10);	
 	// Filtro passa-basso a 460Hz - 1.94ms sull'acc
	wiringPiI2CWriteReg8 (fd, MPU9250_ACCEL_CONFIG2, 0x00);
  delay(10);
	// Imposta il pin INT a mantenersi HIGH finche' l'interrupt non scompare 
	// e mi permette di leggere direttamente il magnetometro
	int e=wiringPiI2CWriteReg8 (fd, MPU9250_INT_PIN_CFG, 0x22);
	if(e!=0) printf("error\n");
  delay(10);
	// Imposta Data Ready come interrupt
	wiringPiI2CWriteReg8 (fd, MPU9250_INT_ENABLE, 0x01);
  delay(10);
	// Non memorizza i valori dei registri dei sensori nella memoria FIFO
	wiringPiI2CWriteReg8 (fd, MPU9250_FIFO_EN, 0x00);
  delay(10);
	// Modalità a singolo master e non aspetta per un Data Ready esterno
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_MST_CTRL, 0x00);
  delay(10);
  // Disabilita FIFO, AUX I2C e I2C reset.
	wiringPiI2CWriteReg8 (fd, MPU9250_USER_CTRL, 0x00);
  delay(10);
	// Setta i dispositivi AUX I2C
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV0_ADDR, 0x00);
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV0_REG, 0x00);
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV0_CTRL, 0x00);
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV1_ADDR, 0x00);
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV1_REG, 0x00);
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV1_CTRL, 0x00);
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV2_ADDR, 0x00);
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV2_REG, 0x00);
   wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV2_CTRL, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV3_ADDR, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV3_REG, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV3_CTRL, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV4_ADDR, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV4_REG, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV4_DO, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV4_CTRL, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV4_DI, 0x00);
  delay(10);
	// Setta i dati in uscita verso i dispositivi AUX I2C
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV0_DO, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV1_DO, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV2_DO, 0x00);
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_SLV3_DO, 0x00);
  delay(10);
	// Altra configurazione per i dispositivi AUX I2C.
	wiringPiI2CWriteReg8 (fd, MPU9250_I2C_MST_DELAY_CTRL, 0x00);
  delay(10);
	// Reset dei sensori disattivato
	wiringPiI2CWriteReg8 (fd, MPU9250_SIGNAL_PATH_RESET, 0x00);
  delay(10);
	// Configurazione della Motion Detection
	wiringPiI2CWriteReg8 (fd, MPU9250_MOT_DETECT_CTRL, 0x00);
  delay(10);	
	// Disabilita il trasferimento dati dal FIFO
	wiringPiI2CWriteReg8 (fd, MPU9250_FIFO_R_W, 0x00);
  delay(10);	
	
	
	// AK COMPASS CONFIGURATION
	
	//Fuse ROM mode
	wiringPiI2CWriteReg8 (fd_ak, AK8963_CNTL1, 0x1F);
	delay(10);
	
	factory_cal[0]=wiringPiI2CReadReg8(fd_ak, AK8963_ASAX);
	factory_cal[1]=wiringPiI2CReadReg8(fd_ak, AK8963_ASAY);
	factory_cal[2]=wiringPiI2CReadReg8(fd_ak, AK8963_ASAZ);
	factory_cal[0] =  (double)(factory_cal[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  factory_cal[1] =  (double)(factory_cal[1] - 128)/256. + 1.;  
  factory_cal[2] =  (double)(factory_cal[2] - 128)/256. + 1.; 
  
	//Exit Fuse ROM mode
	wiringPiI2CWriteReg8 (fd_ak, AK8963_CNTL1, 0x00);
	delay(10);
	
	// Setta misurazione a 100Hz e 16-bit output
	wiringPiI2CWriteReg8 (fd_ak, AK8963_CNTL1, 0x16);
  delay(10);
  
  
  // BMP CONFIGURATION - 23Hz refresh rate
  wiringPiI2CWriteReg8(fd_bmp, BMP280_CTRL_MEAS, 0x3F); //temp x2, press x16, normal mode
  delay(10);
  wiringPiI2CWriteReg8(fd_bmp, BMP280_CONFIG, 0x1C); //sleep 0.5ms, IIR 16, i2c
  delay(10);

	dig_T1 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_T1);
  dig_T2 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_T2);
  dig_T3 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_T3)-65536;
  dig_P1 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P1);
  dig_P2 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P2)-65536;
  dig_P3 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P3);
  dig_P4 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P4);
  dig_P5 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P5);
  dig_P6 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P6)-65536;
  dig_P7 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P7);
  dig_P8 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P8)-65536;
  dig_P9 = wiringPiI2CReadReg16(fd_bmp, BMP280_DIG_P9);
  
	calc_oriz_offsets(sensors_offsets);
	
	return;
}

void read_raw_acc_gyro(int* data) {
	char d[16];
	
	const int addr1 = MPU9250_ACCEL_XOUT_H;
	int i;

	write(fd,&addr1,1);
	read(fd,d,14);
	
	// Lettura accelerometro asse x
	data[AX] = d[1] | (d[0] << 8);

	// Lettura accelerometro asse y
	data[AY] = d[3] | (d[2] << 8);

	// Lettura accelerometro asse z
	data[AZ] = d[5] | (d[4] << 8);
	
	// Lettura temperatura
	data[TMP] = d[7] | (d[6] << 8);

	// Lettura giroscopio asse x
	data[GX] = d[9] | (d[8] << 8);

	// Lettura giroscopio asse y
	data[GY] = d[11] | (d[10] << 8);

	// Lettura giroscopio asse z
	data[GZ] = d[13] | (d[12] << 8);
	
	// Trsaforma tutti i dati contenuti nell'array dal complemento a 2 al decimale
	for(i = 0; i<=TMP; i++)
	{
		if (data[i] & 0x8000)
		 	data[i] -= 65536;
	}
	
}

void read_raw_mag(int* data) {
	char d[8];
	const int addr2 = AK8963_XOUT_L;
	const int addr3 = AK8963_ST2;
	int i;
	
	write(fd_ak,&addr2,1);
	read(fd_ak,d,6);
		
	// Lettura magnetometro asse x
	data[MX] = d[0] | (d[1] << 8);
	
	// Lettura magnetometro asse y
	data[MY] = d[2] | (d[3] << 8);
	
	// Lettura magnetometro asse z
	data[MZ] = d[4] | (d[5] << 8);
		
	write(fd_ak,&addr3,1);
	read(fd_ak,d,1);
	// Controlla un overflow del mag, anche necessario per aggiornare le letture
	if(0x08 & d[0]) printf("mag overflow\n");
	
	for(i = MX; i<=MZ; i++)
	{
		if (data[i] & 0x8000)
			//data[i] = (signed int)((-1)*(signed int)((~data[i])+1)); 
		 	data[i] -= 65536;
	}
	
}

void read_raw_press(int* data) {
	char d[8];
	const int addr4 = BMP280_PRESS_MSB;
	int i;
	
	write(fd_bmp, &addr4,1);
	read(fd_bmp,d,6);
			
	// Lettura tempertura barometro
	data[PS_TMP] = ((((d[3] << 8) | d[4]) << 8) | d[5]) >> 4;
		  
	//Lettura pressione barometro
	data[PS] = ((((d[0] << 8) | d[1]) << 8) | d[2]) >> 4;
}

void calc_oriz_offsets(double* offs)
{
	int i,jj;
	int n, mag_cal=0;
	int mag_max[3], mag_min[3];
	int mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	
	// Inizializza l'array sensor_offsets
	for(i = 0; i < 12; i++)
	{
		offs[i] = 0;
	}
	
	// Scarta le prime 100 letture
	for(i = 0; i < 100; i++)
	{
		read_raw_acc_gyro(sensors_raw_values);
		read_raw_mag(sensors_raw_values);
		read_raw_press(sensors_raw_values);
  }
  
  printf("\nInizio calibrazione sensori\n");
 
	// Calcola i valori da dare agli offsets
	for(i = 0; i < CALIBRATION_ITERATIONS; i++)
	{
		read_raw_acc_gyro(sensors_raw_values);

		// Assegna i valori offsets
		for(n = 0; n <= PS_TMP; n++)
		{
			offs[n] += sensors_raw_values[n];
		}

		delay(20);
	}
	
	if(mag_cal) {
		printf("\nInizio calibrazione magnetometro\n");
		delay(2000);
		for(i = 0; i < CALIBRATION_ITERATIONS*2; i++)
		{
			read_raw_mag(sensors_raw_values);
				
			for (jj = 0; jj < 3; jj++) {
				if(i==0) {
					mag_max[jj] = sensors_raw_values[jj+MX];
					mag_min[jj] = sensors_raw_values[jj+MX];
				}
				else {
			 		if(sensors_raw_values[jj+MX] > mag_max[jj]) mag_max[jj] = sensors_raw_values[jj+MX];
		  		if(sensors_raw_values[jj+MX] < mag_min[jj]) mag_min[jj] = sensors_raw_values[jj+MX];
		  	}
	 		}
	 			
	 	delay(60);
		}

		// Get hard iron correction
		mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	 	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	 	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
	
		dest1[0] = (float) mag_bias[0]* 1.4993894994 * factory_cal[0];  // save mag biases in G for main program
	 	dest1[1] = (float) mag_bias[1]* 1.4993894994 * factory_cal[1];   
	 	dest1[2] = (float) mag_bias[2]* 1.4993894994 * factory_cal[2];	
		 
		// Get soft iron correction estimate
	 	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	 	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	 	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
	
		float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
		avg_rad /= 3.0;
		
		dest2[0] = avg_rad/((float)mag_scale[0]);
		dest2[1] = avg_rad/((float)mag_scale[1]);
		dest2[2] = avg_rad/((float)mag_scale[2]);	
		
		printf("\ndest1: %f - %f - %f\ndest2: %f - %f - %f\n",dest1[0],dest1[1],dest1[2],dest2[0],dest2[1],dest2[2]);
		delay(2000);
	}
	
	for(n = 0; n <= 11; n++)
		{
			offs[n] /= CALIBRATION_ITERATIONS;
		}
		
	printf("\nFine calibrazione\n");
}

void get_angle(struct angle* angolo)
{
  // Legge questo registro per riportare l'interrupt allo status LOW
	//wiringPiI2CReadReg8(fd, MPU9250_INT_STATUS);
	
	// Legge i dati attuali
	read_raw_acc_gyro(sensors_raw_values);
	
	angolo->acc_x = (double)sensors_raw_values[AX] / 16384;
	angolo->acc_y = (double)sensors_raw_values[AY] / 16384;
	angolo->acc_z = (double)sensors_raw_values[AZ] / 16384;
	angolo->gyro_x = (double)(sensors_raw_values[GX] - sensors_offsets[GX])/131.072; //At 250°/s scale
  angolo->gyro_y = (double)(sensors_raw_values[GY] - sensors_offsets[GY])/131.072;
	angolo->gyro_z = (double)(sensors_raw_values[GZ] - sensors_offsets[GZ])/131.072;
	//sensors_values[TMP] = (double)sensors_raw_values[TMP] / 340.0 + 36.53;
	
	if (cc1>4) {
		read_raw_mag(sensors_raw_values);
		angolo->mag_x = (double) (((sensors_raw_values[MX] ) * 1.4993894994 * factory_cal[0]) - dest1[0])*dest2[0];
		angolo->mag_y = (double) (((sensors_raw_values[MY] ) * 1.4993894994 * factory_cal[1]) - dest1[1])*dest2[1];
		angolo->mag_z = (double) (((sensors_raw_values[MZ] ) * 1.4993894994 * factory_cal[2]) - dest1[2])*dest2[2];
		cc1=0;
	}
	cc1++;

	if (cc>40) {
		read_raw_press(sensors_raw_values);

		int t1 = (((sensors_raw_values[PS_TMP] >> 3) - (dig_T1 << 1)) * (dig_T2)) >> 11;
	  signed int t2 = (((((sensors_raw_values[PS_TMP] >> 4) - (dig_T1)) * ((sensors_raw_values[PS_TMP] >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14;
	  signed int t_fine = t1+t2;
	  angolo->press_tmp = (double)((t_fine * 5 + 128) >> 8)/100;

		long long signed int pippo1, pippo2, pluto;
	  pippo1 = ((long long signed int)t_fine) - 128000;
	  pippo2 = pippo1 * pippo1 * (long long signed int)dig_P6;
	  pippo2 = pippo2 + ((pippo1*(long long signed int)dig_P5)<<17);
	  pippo2 = pippo2 + (((long long signed int)dig_P4)<<35);
	  pippo1 = ((pippo1 * pippo1 * (long long signed int)dig_P3)>>8) + ((pippo1 * (long long signed int)dig_P2)<<12);
		pippo1 = (((((long long signed int)1)<<47)+pippo1))*((long long signed int)dig_P1)>>33;

	   if(pippo1 == 0)
	  {
	    return;
	    // avoid exception caused by division by zero
	  }
	  
	  pluto = 1048576 - (long long signed int)sensors_raw_values[PS];
	  pluto = (double)(((pluto<<31) - pippo2)*3125)/pippo1;
	  pippo1 = (((long long signed int)dig_P9) * (pluto>>13) * (pluto>>13)) >> 25;
	  pippo2 = (((long long signed int)dig_P8) * pluto)>> 19;
	  pluto = ((pluto + pippo1 + pippo2) >> 8) + (((long long signed int)dig_P7)<<4);
		angolo->press=((double)pluto) / 256;
	
		cc=0;
	}  
	cc++;

//  printf("\nX: %.3f - Y: %.3f - Z: %.3f - Alt: %.3f - TMP: %.3f\n",angolo->x,angolo->y,angolo->z,44330.0*(1-pow((sensors_values[PS]/101325),(1/5.255))),sensors_values[PS_TMP]);
  //delay(200);
  
 	return;
}
