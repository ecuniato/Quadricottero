#include "incl/imu.h"
#include "incl/pwm.h"
#include "incl/pid.h"
#include "incl/MadgwickAHRS.h"
#include "incl/tinyekf_config.h"
#include "incl/tiny_ekf.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <errno.h>
//COmmento

#define M1 4//26 //X+
#define M2 17//13 //X-
#define M3 27//19 //Y+
#define M4 22//6  //Y-
#define PORT 4567

extern int fd, fd_ak, fd_bmp;
struct PID_parameters roll_angle_pid, pitch_angle_pid, yaw_angle_pid, roll_rate_pid, pitch_rate_pid, yaw_rate_pid;
int channel = 1;
int imuReady=0, motoReady=0;
FILE* file;
extern float dest1[3], dest2[3];
float stepTime, stepTime1;
ekf_t ekf;
 double height, zacc=0;

struct dataStruct
  {
   int throttle;
	 float roll_rate, pitch_rate, yaw_rate;
	 float r_kp, r_ki, r_kd;
	 float a_kp, a_ki, a_kd;
	 float ay_kp, ay_ki, ay_kd;
	 float y_kp, y_ki, y_kd;
	 int roll_corr, pitch_corr, yaw_corr;
	 struct angle angolo;
	 int roll_trim, pitch_trim, yaw_trim;
	 char quit, acro;
  } data;


void init_data()
{
  data.throttle = 0;
  data.roll_rate = 0.0; data.pitch_rate = 0.0; data.yaw_rate = 0.0;
  fscanf(file, "%f %f %f", &data.r_kp, &data.r_ki, &data.r_kd);
	fscanf(file, "%f %f %f", &data.a_kp, &data.a_ki, &data.a_kd);
	fscanf(file, "%f %f %f", &data.ay_kp, &data.ay_ki, &data.ay_kd);
	fscanf(file, "%f %f %f", &data.y_kp, &data.y_ki, &data.y_kd);
	fscanf(file, "%d %d %d", &data.roll_trim, &data.pitch_trim, &data.yaw_trim);
	fscanf(file, "%f %f %f", &dest1[0], &dest1[1], &dest1[2]);
	fscanf(file, "%f %f %f", &dest2[0], &dest2[1], &dest2[2]);
  data.roll_corr = 0; data.pitch_corr = 0; data.yaw_corr = 0;
  data.quit = 0, data.acro = 1;  
  printf("\ndest1: %f - %f - %f\ndest2: %f - %f - %f\n",dest1[0],dest1[1],dest1[2],dest2[0],dest2[1],dest2[2]);
}

static void init(ekf_t * ekf)
{
    // Set Q
    ekf->Q[0][0]=0.3;
    ekf->Q[1][1]=0.5;

    // initial covariances of state noise, measurement noise
    ekf->P[0][0] = 0.1;
    ekf->P[1][1] = 0.1;
    ekf->P[2][2] = 1000;
    
    ekf->R[0][0] = 1;

    // initial states
    ekf->x[0] = 0;
    ekf->x[1] = 0;
    ekf->x[2] = 100;
}

static void model(ekf_t * ekf, double acc)
{ 
	
	ekf->fx[0] = ekf->x[0] + stepTime * ekf->x[1] + pow(stepTime,2)*0.5*acc;
	ekf->fx[1] = ekf->x[1] + stepTime*acc;
	ekf->fx[2] = ekf->x[2];
	
  ekf->F[0][0] = 1;
  ekf->F[1][1] = 1;
  ekf->F[2][2] = 1;
  ekf->F[0][1] = stepTime;

  ekf->hx[0] = ekf->x[0] + ekf->x[2];
  
  ekf->H[0][0]  = 1;
  ekf->H[0][1]  = 0;
  ekf->H[0][2]  = 1;
}


PI_THREAD (sensorFusion)
{
	unsigned int time=0, time1;
	while(!imuReady) delay(500);
	
	while (1) {
		time1 = micros();
	  stepTime1 = time1 - time;
		time = time1;
	//	if(stepTime1<100)
  //    	delayMicroseconds(90);
	  stepTime1 /= 1000000; 
		
	//	MadgwickAHRSupdate(data.angolo.gyro_x * PI / 180, -data.angolo.gyro_y * PI / 180, -data.angolo.gyro_z * PI / 180, -data.angolo.acc_x, data.angolo.acc_y, data.angolo.acc_z, data.angolo.mag_y, -data.angolo.mag_x, data.angolo.mag_z, 1/stepTime1);
		MadgwickAHRSupdate(data.angolo.gyro_x * PI / 180, data.angolo.gyro_y * PI / 180, data.angolo.gyro_z * PI / 180, data.angolo.acc_x, data.angolo.acc_y, data.angolo.acc_z, data.angolo.mag_y, data.angolo.mag_x, data.angolo.mag_z, 1/stepTime1);
	// 	MadgwickAHRSupdateIMU(data.angolo.gyro_x * PI / 180, -data.angolo.gyro_y * PI / 180, -data.angolo.gyro_z * PI / 180, -data.angolo.acc_x, data.angolo.acc_y, data.angolo.acc_z, 1/stepTime1);
		data.angolo.z=atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1)*180/PI;
	  data.angolo.x=asin(2*q1*q3+2*q0*q2)*180/PI;
	  data.angolo.y=atan2(2*q2*q3-2*q0*q1,2*q0*q0+2*q3*q3-1)*180/PI;
	  zacc=data.angolo.acc_z - (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
	  
	  //TODO: Controllare convergenza sensor fusion
       
	}
}


PI_THREAD (imuData)
{
  unsigned int time=0, time1;
 
  
  int roll_corr=0, pitch_corr=0, yaw_corr=0;
  int roll_rate_target=0, pitch_rate_target=0, yaw_rate_target=0;
  int M1corr=0, M2corr=0, M3corr=0, M4corr=0;
 
  
  // Do generic EKF initialization
 	ekf_init(&ekf, Nsta, Mobs);
 			    
	// Do local initialization
	init(&ekf);
	
  // Configura l'IMU
	init_imu();
  
  imuReady=1;
  printf("Imu ready \n");
  
 //  while( !motoReady ) 
    delay(5000); 
  
  init_pid_param (&roll_angle_pid, data.a_kp, data.a_ki, data.a_kd, 0, 600, 0);
  init_pid_param (&pitch_angle_pid, data.a_kp, data.a_ki, data.a_kd, 0, 600, 0);
  init_pid_param (&yaw_angle_pid, data.ay_kp, data.ay_ki, data.ay_kd, 0, 600, 0);
  init_pid_param (&roll_rate_pid, data.r_kp, data.r_ki, data.r_kd, 0, 600, 0);
  init_pid_param (&pitch_rate_pid, data.r_kp, data.r_ki, data.r_kd, 0, 600, 0);
  init_pid_param (&yaw_rate_pid, data.y_kp, data.y_ki, data.y_kd, 0, 600, 0);
  
  
  while(1)
  {
    if (time==0)
    {
      time = micros();
      delay(5);
    }
    else
    {
    	time1 = micros();
      stepTime = time1 - time;
      time = time1;
     // if(stepTime<5000)
      //	delayMicroseconds(5000-stepTime);
      stepTime /= 1000000;      
      
      get_angle(&(data.angolo));
      
			model(&ekf, zacc);
			height=44330.0*(1-pow((data.angolo.press/101325),(1/5.255)));
 			ekf_step(&ekf, &height);
  
  
      
      if (data.acro==0)
      {
//      	roll_rate_target = round( pid(data.roll_rate, data.angolo.y, stepTime, &roll_angle_pid) );
// 	      roll_rate_target /= stepTime;
//      	roll_corr = round( pid(roll_rate_target, data.angolo.gyro_x, stepTime, &roll_rate_pid) );
      	
      	pitch_rate_target = round( pid(data.pitch_rate, data.angolo.x, stepTime, &pitch_angle_pid) );
// 	      pitch_rate_target /= stepTime;
      	pitch_corr = round( pid(pitch_rate_target, data.angolo.gyro_y, stepTime, &pitch_rate_pid) );
      	
 //     	yaw_rate_target = round( pid(data.yaw_rate, data.angolo.z, stepTime, &yaw_angle_pid) );
// 	      yaw_rate_target /= stepTime;
//      	yaw_corr = round( pid(yaw_rate_target, data.angolo.gyro_z, stepTime, &yaw_rate_pid) );      	
			}
			else
			{
//				roll_corr = round( pid(data.roll_rate, data.angolo.gyro_x, stepTime, &roll_rate_pid) );

      	pitch_corr = round( pid(data.pitch_rate, data.angolo.gyro_y, stepTime, &pitch_rate_pid) );
      
  //    	yaw_corr = round( pid(data.yaw_rate, data.angolo.gyro_z, stepTime, &yaw_rate_pid) );
      }
      
      
      M1corr = data.throttle-pitch_corr+yaw_corr-data.pitch_trim-data.yaw_trim;
      if (M1corr < 0)
        M1corr = 0;
      else if (M1corr > 1000)
        M1corr = 1000;
      add_channel_pulse(channel, M1, 0, (1000+M1corr));
      
      M2corr = data.throttle+pitch_corr+yaw_corr+data.pitch_trim-data.yaw_trim;
      if (M2corr < 0)
        M2corr = 0;
      else if (M2corr > 1000)
        M2corr = 1000;
      add_channel_pulse(channel, M2, 0, (1000+M2corr));
      
      M3corr = data.throttle+roll_corr-yaw_corr+data.roll_trim+data.yaw_trim;
      if (M3corr < 0)
        M3corr = 0;
      else if (M3corr > 1000)
        M3corr = 1000;
      add_channel_pulse(channel, M3, 0, (1000+M3corr));
      
      M4corr = data.throttle-roll_corr-yaw_corr-data.roll_trim+data.yaw_trim;
      if (M4corr < 0)
        M4corr = 0;
      else if (M4corr > 1000)
        M4corr = 1000;
      add_channel_pulse(channel, M4, 0, (1000+M4corr));	      
        
      data.roll_corr = roll_corr;
      data.pitch_corr = pitch_corr;
      data.yaw_corr = yaw_corr;   
//			printf("StepTime: %f\n", stepTime);   
    }
  }
  return;  
}


PI_THREAD (debug)
{
   while( !imuReady ) {
    delay(500); }
  while (1)
  {
   //printf("StepSensors: %f - StepFusion: %f\n", stepTime, stepTime1);   
   printf("X0: %f - X1: %f - X2: %f - H: %f\n", ekf.x[0], ekf.x[1], ekf.x[2], height);   
    fflush(NULL);
    delay(100);
  }
  return;
}


int main()
{  
  char c;
  int i, e;
  int sockfd, new_sockfd=-1, yes=1;
  struct sockaddr_in host_inf, client_inf;
  socklen_t struct_size;
  data.angolo.x=0;
  data.angolo.y=0;
  data.angolo.z=0;
  
  file = fopen("/home/pi/tmp/quad/save.txt","r");
  if (file == NULL)
  {
    printf("Error opening file");
    exit(0);
  }
  init_data();
  fclose(file);
  /*
  sockfd=socket(PF_INET,SOCK_STREAM,0);
  if (sockfd==-1)
    printf("Socket Errno=%d", errno);
    
  fcntl(sockfd, F_SETFL, O_NONBLOCK);
  host_inf.sin_family=AF_INET;
  host_inf.sin_port=htons(PORT);
  host_inf.sin_addr.s_addr=INADDR_ANY;
  memset(&(host_inf.sin_zero),'\0',8);
    
  e = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
  if (e == -1)
    printf("Setsockopt Errno=%d", errno);
    
  e = bind(sockfd,(struct sockaddr *)&host_inf,sizeof(struct sockaddr));
  if (e == -1)
    printf("Bind Errno=%d", errno);
    
  e = listen(sockfd,1);
  if (e == -1)
    printf("Listen Errno=%d", errno);
     */
  
  // Inizializza i pin GPIO
	if( wiringPiSetupGpio() == -1)
	{
		printf("Inizializzazione gpio fallita!");
		return 0;
	}

	// Apre l'interfaccia I2C
   fd = wiringPiI2CSetup(MPU_ADDRESS);
   if( fd == -1 )
   {
      printf("Inizializzazione I2C MPU fallita!");
      return 0;
   }

	fd_ak = wiringPiI2CSetup(AK8963_ADDRESS);
   if( fd_ak == -1 )
   {
      printf("Inizializzazione I2C HMC fallita!");
      return 0;
   }
   
   fd_bmp = wiringPiI2CSetup(BMP280_ADDRESS);
   if( fd_bmp == -1 )
   {
      printf("Inizializzazione I2C BMP fallita!");
      return 0;
   }

	
	// Avvia il thread
	e=piThreadCreate (imuData);
	if (e!=0)
	  printf("Thread non partito");
  e=piThreadCreate (debug);
	if (e!=0)
	  printf("Thread non partito"); 
	e=piThreadCreate (sensorFusion);
	if (e!=0)
	  printf("Thread non partito"); 
	  
	setup(PULSE_WIDTH_INCREMENT_GRANULARITY_US_DEFAULT, DELAY_VIA_PWM);
	
	// Setup channel
  init_channel(channel, SUBCYCLE_TIME_US_DEFAULT);
 
  add_channel_pulse(channel, M1, 0, 1000);
	add_channel_pulse(channel, M2, 0, 1000);
	add_channel_pulse(channel, M3, 0, 1000);
	add_channel_pulse(channel, M4, 0, 1000);
  printf("Throttle al minimo.\n");
  while(1) {delay(1000);
	}

  while (1)
  { 
    delay(30);
    
    if (new_sockfd == -1)
    {
      struct_size = sizeof(struct sockaddr_in);
      new_sockfd = accept(sockfd,(struct sockaddr *)&client_inf,&struct_size);
      if (new_sockfd != -1)
      {
      	motoReady=1;
        printf("\n Connessione ottenuta con %s tramite la porta %d \n", inet_ntoa(client_inf.sin_addr), ntohs(host_inf.sin_port));
        fcntl(new_sockfd, F_SETFL, O_NONBLOCK);
        e = send(new_sockfd,&data,sizeof(struct dataStruct),0); 
        if (e == -1)
          printf(" byte=%d errno=%d" , e, errno);   
      }
      else
        continue;
    }
    
    do
    {
      e = recv(new_sockfd,&data,sizeof(struct dataStruct),0);
	   	if (e != -1)
		  {
        if (data.quit)
        {
          file = fopen("/home/pi/tmp/quad/save.txt","w");
          if (file != NULL)
          {
            fprintf(file,"%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n%d %d %d\n%f %f %f\n%f %f %f", data.r_kp, data.r_ki, data.r_kd, data.a_kp, data.a_ki, data.a_kd, data.ay_kp, data.ay_ki, data.ay_kd, data.y_kp, data.y_ki, data.y_kd, data.roll_trim, data.pitch_trim, data.yaw_trim,dest1[0],dest1[1],dest1[2],dest2[0],dest2[1],dest2[2]);
            fclose(file);
          }
          else  
            printf("Error opening file");  
          
          close (new_sockfd);
          new_sockfd=-1;
          close (sockfd);          
          
          data.throttle=-10000;
          delay(1000);
  
          for (i=0; i<5; i++)
          {
            add_channel_pulse(channel, M1, 0, 1000);
	          add_channel_pulse(channel, M2, 0, 1000);
	          add_channel_pulse(channel, M3, 0, 1000);
	          add_channel_pulse(channel, M4, 0, 1000);
            delay(200);
          }
    
          clear_channel_gpio(channel, M1);
          clear_channel_gpio(channel, M2);
          clear_channel_gpio(channel, M3);
          clear_channel_gpio(channel, M4);
  
          // All done
          shutdown_pwm();
          
//          system("sudo shutdown -h now");    
          
          exit(0);
        }
        init_pid_kvalues (&roll_rate_pid, data.r_kp, data.r_ki, data.r_kd);
        init_pid_kvalues (&pitch_rate_pid, data.r_kp, data.r_ki, data.r_kd);
        init_pid_kvalues (&yaw_rate_pid, data.y_kp, data.y_ki, data.y_kd);
        init_pid_kvalues (&roll_angle_pid, data.a_kp, data.a_ki, data.a_kd);
        init_pid_kvalues (&pitch_angle_pid, data.a_kp, data.a_ki, data.a_kd);
        init_pid_kvalues (&yaw_angle_pid, data.ay_kp, data.ay_ki, data.ay_kd);        
      }
      delay(1);
    } while (e == -1);
    
    
    e = send(new_sockfd,&data,sizeof(struct dataStruct),0); 
    if (e == -1)
    {
		  printf(" byte=%d errno=%d" , e, errno);
//		  new_sockfd = -1;
    }
    
        
  }	  
	return 0;
}
