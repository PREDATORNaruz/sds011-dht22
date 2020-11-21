/*
 *
 *
 *   @authors : Houssem & Fabio
 *
 *   @date:   11.11.2020
 *
 *   @project: Embedded Systems 2 , Reading and logging from
 *    Sensors
 *
 *   @copyright : MIT Lincence
 *
 *   Sensor :
 *   SDS011 - FD - ttyUSB0
 *   DHT22 - Onewire serial Communication
 *
 *   Logging : CSV with Timestamp
 */

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <inttypes.h>
#include <termio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>
#define DIRECT_COMPILE

#ifdef DIRECT_COMPILE
#include <wiringPi.h>
#define DHT_PIN 3 
#endif
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef float float32;
typedef double float64;

#define FAIL -1
#define SUCCESS 0

//#define SENS_DEBUG

#define USER_PATH
//#define SELF_BAUDRATE

#if defined(SELF_BAUDRATE)
#define BAUDRATE 9600
#endif

#ifdef USER_PATH
#ifdef SYSFS_WAY
#define GPIO_FILE_EDGE_PATH "/sys/class/gpio/gpio22/edge"
#define GPIO_FILE_VALUE_PATH "/sys/class/gpio/gpio22/value"
#define GPIO_FILE_DIRECTION_PATH "/sys/class/gpio/gpio22/direction"
#define GPIO_FILE_PATH "/sys/class/gpio"
#endif
#define USART_FILE_PATH "/dev/ttyUSB0"
#define CSV_FILE_PATH "/home/pi/Documents/embedded_sys_2/data.csv"
#endif

#define MAX_TIMINGS 85

#define ERROR(x) { fprintf(stderr, "ERROR in %s: %s", __func__, (x)); }

#define CHECK(syscall, msg) do {                    \
    if ((syscall) == -1) {                          \
      perror(msg);                                  \
    }                                               \
  } while(0)

// -----------------------------------------------
struct termios tty_back;
i32 fd;
typedef struct {
	float32 pm25;
	float32 pm10;
	float32 acc_pm25 ; 
	float32 acc_pm10 ; 
	float32 avg_pm25 ; 
	float32 avg_pm10 ; 
} sds011_t;
// -----------------------------------------------
#ifdef SYS_WAY
char pin_string[3]= "22" ;
#endif
typedef struct {
	float32 temp;
	float32 humi;
	float32 acc_temp ; 
	float32 acc_humi ;
	float32 avg_tmp ; 
	float32 avg_hum ; 
} dht_t;
int data[5] = { 0 } ;
i8 offset = 0 ;
i32 dht_size = 0 ; 
i32 sds_size = 0  ;
// -----------------------------------------------
void finit(FILE* f) {
	f = fopen(CSV_FILE_PATH, "w");
	if (f == NULL) {
		ERROR("file is NULL\n");
	}
	fprintf(f, "Time , PM2.5 , PM10, TEMP , HUMI\n");
	
	fclose(f);
}

void fwrite_data(FILE* f, sds011_t * data , dht_t * params) {
//   struct timespec ts ;
//
//   timespec_get(&ts , TIME_UTC) ;
//   char time_buf[15]= {0} ;
//   char date_buf[20] = {0} ;
//   strftime(date_buf , sizeof(date_buf) , "%F" , gmtime(&ts.tv_sec));
	char buffer_data[200];
	char pm_string[12];
	char pm10_string[5];
	char pm25_string[5];
	// char temp_string[4] ;
	// char humi_string[4] ;
	time_t raw_time;
//	struct tm * timeinfo ;
//	time(&raw_time) ;
//	timeinfo = localtime(&raw_time) ;
//	printf("Current local time and date : %s" , asctime(timeinfo)) ;
//
//

	raw_time = time(NULL);
	char * time_str = ctime(&raw_time);
	time_str[strlen(time_str) - 1] = '\0';
	f = fopen(CSV_FILE_PATH, "a");
	if (f == NULL) {
		ERROR("file writer is NULL");
	}
	//sprintf(pm10_string, ", %0.2f ,", data->pm10);
//	sprintf(pm25_string, ", %0.2f ,", data->pm25);
	//strcat(pm_string, pm25_string);
//	strcat(pm_string, pm10_string);
	//strcat(buffer, asctime(timeinfo)) ;
//	printf("%s", pm_string);
//	strcat(buffer_data, time_str);
//	strcat(buffer_data, pm_string);
//	strcat(buffer_data, "0");
//	strcat(buffer_data, ",");
//	strcat(buffer_data, "0");
//	strcat(buffer_data, "\n");
//	fprintf(f, buffer_data);
//   if(offset){
strcat(time_str,",") ; 
   fprintf(f,time_str); 
   fprintf(f,"%0.2f ," ,data->avg_pm10) ; 
   fprintf(f,"%0.2f ," ,data->avg_pm25) ; 

   fprintf(f,"%0.2f ," ,params->avg_tmp) ; 
   fprintf(f,"%0.2f \n" ,params->avg_hum) ; 
 //  }
 //  else
 //  {
//	offset = 1 ; 
//   }
   

	fclose(f);

}

// -----------------------------------------------
#ifdef SYSFS_WAY
i8 gpio_setup(i8 mode) {

//    ssize_t ret ;
	i32 gpio_fd ;
//	char buffer_gpio_setup[50] ;
	if(export==0){
	gpio_fd = open(GPIO_FILE_PATH"/export" , O_WRONLY);
	if(gpio_fd <0){
		ERROR("gpio_fd invalid\n") ;
	}
    write(gpio_fd , pin_string , sizeof(pin_string)) ;
//   if (ret < 0) {
//	   ERROR("invalid write GPIO\n") ;
//   }
    export = 1  ;
    close(fd) ;
	}
//    strcat(buffer_gpio_setup,GPIO_FILE_PATH"/gpio");
//    strcat(buffer_gpio_setup,pin_string);
//    strcat(buffer_gpio_setup,"/direction") ;
    gpio_fd = open(GPIO_FILE_DIRECTION_PATH,O_WRONLY) ;
     if (gpio_fd<0) {
    	 ERROR("setting gpio direction!\n") ;
     }
     if(mode == 0) {
    	 write(gpio_fd,"in" , 3) ;
    	 close(gpio_fd);
     }
     else {
    	 write(fd,"out",4) ;
    	 close(gpio_fd) ;
     }
//     memset(buffer_gpio_setup, '\0', sizeof(buffer_gpio_setup));
//     strcat(buffer_gpio_setup,GPIO_FILE_PATH"/gpio");
//     strcat(buffer_gpio_setup,pin_string);
//     strcat(buffer_gpio_setup,"/edge") ;
     if(export==0) {
     gpio_fd = open(GPIO_FILE_EDGE_PATH , O_WRONLY) ;
     if (gpio_fd<0) {
    	 ERROR("setting gpio edge!\n") ;
     }
      write(fd , "none" , 4) ;
      close(fd) ;
     }
     return SUCCESS ;
}
i8 set_gpio_value(i8 value) {

//	char buffer_value[50] ;
	i32 gpio_fd ;
//    strcat(buffer_value,GPIO_FILE_PATH"/gpio");
//    strcat(buffer_value,pin_string);
//    strcat(buffer_value,"/value") ;
    gpio_fd = open(GPIO_FILE_VALUE_PATH , O_WRONLY) ;
    if(fd < 0 ) {
      ERROR("writing gpio value\n") ;
    }
    if(value==0) {
    write(gpio_fd,"0",2) ;
    }
    else {
    write(gpio_fd,"1",2) ;
    }
    close(gpio_fd) ;
    return SUCCESS ;
}
i8 get_gpio_value(){
	i8 val[2] ;
//	char buf[50]="/sys/class/gpio/gpio22/value" ;
  //  memset(buffer, '\0', sizeof(buffer));
	i32 gpio_fd ;
//    strcat(buf,GPIO_FILE_PATH"/gpio");
//    strcat(buf,pin_string);
//    strcat(buf,"/value") ;

    gpio_fd = open(GPIO_FILE_VALUE_PATH, O_RDONLY) ;
    if(fd < 0 ) {
      ERROR("writing gpio value\n") ;
    }
    read(gpio_fd , &val , sizeof(val)) ;
    if(val[0] == 0x30){
    	close(gpio_fd) ;
    	return 0 ;
    }
   close(gpio_fd) ;
    return 1  ;
}

//void read_dht_data()
//{
//	u8 laststate	= 1;
//	u8 counter		= 0;
//	u8 j			= 0, i;
//
//	data_dht[0] = data_dht[1] = data_dht[2] = data_dht[3] = data_dht[4] = 0;
//
//
//	gpio_setup(1) ;
//	set_gpio_value(0) ;
//	usleep(180) ;
//
//
//	gpio_setup(0) ;
//
//
//	for ( i = 0; i < MAX_TIMINGS; i++ )
//	{
//		counter = 0;
//		while ( get_gpio_value() == laststate )
//		{
//			counter++;
//			usleep( 1 );
//			if ( counter == 255 )
//			{
//				break;
//			}
//		}
//		laststate = get_gpio_value();
//
//		if ( counter == 255 )
//			break;
//
//		if ( (i >= 4) && (i % 2 == 0) )
//		{
//			data_dht[j / 8] <<= 1;
//			if ( counter > 16 )
//				data_dht[j / 8] |= 1;
//			j++;
//		}
//	}
//
//
//	if ( (j >= 40) &&
//	     (data_dht[4] == ( (data_dht[0] + data_dht[1] + data_dht[2] + data_dht[3]) & 0xFF) ) )
//	{
//		float h = (float)((data_dht[0] << 8) + data_dht[1]) / 10;
//		if ( h > 100 )
//		{
//			h = data_dht[0];	// for DHT11
//		}
//		float c = (float)(((data_dht[2] & 0x7F) << 8) + data_dht[3]) / 10;
//		if ( c > 125 )
//		{
//			c = data_dht[2];	// for DHT11
//		}
//		if ( data_dht[2] & 0x80 )
//		{
//			c = -c;
//		}
//		float f = c * 1.8f + 32;
//		printf( "Humidity = %.1f %% Temperature = %.1f *C (%.1f *F)\n", h, c, f );
//	}
//	else  {
//		printf( "Data not good, skip\n" );
//	}
//}
#endif
#ifdef DIRECT_COMPILE

void read_dht_data(dht_t * params)
{
	uint8_t laststate	= HIGH;
	uint8_t counter		= 0;
	uint8_t j			= 0, i;

	data[0] = data[1] = data[2] = data[3] = data[4] = 0;

	
	pinMode( DHT_PIN, OUTPUT );
	digitalWrite( DHT_PIN, LOW );
	delay( 18 );


	pinMode( DHT_PIN, INPUT );


	for ( i = 0; i < MAX_TIMINGS; i++ )
	{
		counter = 0;
		while ( digitalRead( DHT_PIN ) == laststate )
		{
			counter++;
			delayMicroseconds( 1 );
			if ( counter == 255 )
			{
				break;
			}
		}
		laststate = digitalRead( DHT_PIN );

		if ( counter == 255 )
			break;

		
		if ( (i >= 4) && (i % 2 == 0) )
		{
			
			data[j / 8] <<= 1;
			if ( counter > 16 )
				data[j / 8] |= 1;
			j++;
		}
	}


	if ( (j >= 40) &&
	     (data[4] == ( (data[0] + data[1] + data[2] + data[3]) & 0xFF) ) )
	{
		float h = (float)((data[0] << 8) + data[1]) / 10;
		if ( h > 100 )
		{
			h = data[0];	
		}
		float c = (float)(((data[2] & 0x7F) << 8) + data[3]) / 10;
		if ( c > 125 )
		{
			c = data[2];	
		}
		if ( data[2] & 0x80 )
		{
			c = -c;
		}
		float f = c * 1.8f + 32;
		
	//	printf( "Humidity = %.1f %% Temperature = %.1f *C (%.1f *F)\n", h, c, f );
	    params->humi = h ; 
		params->temp = c ; 
		params->acc_humi = params->acc_humi + params->humi ;
		params->acc_temp = params->acc_temp + params->temp ;
		dht_size++ ; 
	}else  {
		printf( "Data not good, skip\n" );
	}
}
#endif



// -----------------------------------------------
void calc_avg(sds011_t * data , dht_t * params) { 
	data->avg_pm10 = data->acc_pm10/sds_size ; 
	data->avg_pm25 = data->acc_pm25/sds_size ; 
	params->avg_hum = params->acc_humi/dht_size ; 
	params->avg_tmp = params->acc_temp/dht_size ; 
}
void init_data(sds011_t * data, dht_t * params) {
	data->pm10 = 0.0;
	data->pm25 = 0.0;
	data->acc_pm10 = 0.0 ; 
	data->acc_pm25 = 0.0 ; 
	data->avg_pm10 = 0.0 ; 
	data->avg_pm25 = 0.0 ; 
	params->humi = 0.0;
	params->temp = 0.0;
	params->acc_humi = 0.0 ; 
	params->acc_temp = 0.0 ; 
	params->avg_tmp = 0.0 ; 
	params->avg_hum = 0.0 ;
}
void init_avgparams(sds011_t * data , dht_t * params) {
	data->acc_pm10 = 0.0 ; 
	data->acc_pm25 = 0.0 ; 
	data->avg_pm10 = 0.0 ; 
	data->avg_pm25 = 0.0 ; 
	params->acc_humi = 0.0 ; 
	params->acc_temp = 0.0 ; 
	params->avg_tmp = 0.0 ; 
	params->avg_hum = 0.0 ;
	dht_size = 0 ; 
	sds_size = 0 ; 
}
i8 checkcrc(u8* packet, u8 len) {
	uint8_t checksum = 0, counter;

	for (counter = 0; counter < len; counter++) {
		checksum += packet[counter];
	}

	return checksum;
}
void set_blocking(int fd, int mcount) {
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		perror("tcgetattr");
		exit(1);
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;

	if (tcsetattr(fd, TCSANOW, &tty) < 0) {
		perror("tcsetattr");
		exit(1);
	}
}

u8 data_control(u8* packet, u8 len, sds011_t * data) {

	if (packet[1] == 0xC0) {
		data->pm25 = (float) (((packet[3] * 256) + packet[2]) / 10.0);
		data->pm10 = (float) (((packet[5] * 256) + packet[4]) / 10.0);
		data->acc_pm10 = data->acc_pm10 + data->pm10 ; 
		data->acc_pm25 = data->acc_pm25 + data->pm25 ; 
		return SUCCESS;
	} else
		return FAIL;

}

void restore_ser(int fd) {
	CHECK(tcsetattr(fd, TCSANOW, &tty_back), "reset tcsetattr");
}

i8 setup_usart() {
	struct termios tty;

	fd = open(USART_FILE_PATH, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if (fd < SUCCESS) {
		ERROR("open()\n");
		return FAIL;
	}

	if (tcgetattr(fd, &tty_back) < SUCCESS) {
		ERROR("openning ttyaddr");
		close(fd);
		return FAIL;
	}
	if (tcgetattr(fd, &tty) < SUCCESS) {
		ERROR("openning ttyaddr");
		close(fd);
		return FAIL;
	}
	cfmakeraw(&tty);
	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	CHECK(tcsetattr(fd , TCSANOW , &tty) , "tcsetattr()");

	return SUCCESS;
}

ssize_t read_data(sds011_t * rx_buf) {
	ssize_t nbytes = 0;
	u8 buf[20];

	if (rx_buf == NULL) {
		return FAIL;
	}
	sleep(2);
	nbytes = read(fd, buf, sizeof(buf));
	CHECK(nbytes , "read()");

	data_control(buf, (u8) nbytes, rx_buf);
    sds_size++ ; 
	close(fd);

	return nbytes;
}

int main() {
	sds011_t data;
	dht_t dht;
	ssize_t nbytes_ret = 0;
	FILE* fp;
    i32 cnt = 0 ; 
	init_data(&data, &dht);
	finit(fp);
      #ifdef DIRECT_COMPILE
        	if ( wiringPiSetup() == -1 )
		exit( 1 );
     #endif
//	gpio_setup(0) ;
//    if(get_gpio_value() == 0) {
//    	printf("ok") ;
//    }
	usleep(10000);
	system("modprobe usbserial");
	system("modprobe ch341");
	printf("Starting Programm . . . \n");
	printf("Launching USB Setup . . . \n");
while (1) {
		CHECK(setup_usart(), "setup_usart()");
		set_blocking(fd, 0);
		usleep(10000);
		tcflush(fd, TCIOFLUSH);
		usleep(10000);
		nbytes_ret = read_data(&data);
		if (nbytes_ret < SUCCESS) {
			ERROR("read_data \n");
		}
#ifdef DIRECT_COMPILE
		read_dht_data(&dht) ;
		sleep(1) ;
#endif
		printf("Done Reading . . . \n");
		printf("number of measurement SDS : %d \n" , sds_size  ) ; 
		printf("number of measurement DHT : %d \n" , dht_size); 
		printf("Pm10 : %0.2f \n", data.pm10);
		printf("Pm2.5 : %0.2f \n", data.pm25);
		printf("Temeratur : %0.2f \n", dht.temp);
		printf("Humidity : %0.2f \n", dht.humi);
		printf("acc Pm10 : %0.2f \n", data.acc_pm10);
		printf("acc Pm2.5 : %0.2f \n", data.acc_pm25);
		printf("acc Temeratur : %0.2f \n", dht.acc_temp);
		printf("acc Humidity : %0.2f \n", dht.acc_humi);
		cnt++;
		if(cnt==60){

			printf("Logging . . . \n ");
			calc_avg(&data , &dht) ; 
			printf("avg Pm10 : %0.2f \n", data.avg_pm10);
		    printf("avg Pm2.5 : %0.2f \n", data.avg_pm25);
		    printf("avg Temeratur : %0.2f \n", dht.avg_tmp);
		    printf("avg Humidity : %0.2f \n", dht.avg_hum);
			fwrite_data(fp, &data , &dht);
			init_avgparams(&data , &dht ) ; 
			cnt=0 ;
			#ifdef SENS_DEBUG
			goto end ; 
			#endif
		}

	}
	#ifdef SENS_DEBUG
	end:
	asm("nop"); 
		asm("nop"); 
			asm("nop"); 
				asm("nop"); 


	printf("CRITICAL ; ENDING THE PROGRAMM . . .\n ")  ;
			printf("number of measurement SDS : %d \n" , sds_size  ) ; 
		printf("number of measurement DHT : %d \n" , dht_size); 
			printf("acc Pm10 : %0.2f \n", data.acc_pm10);
		printf("acc Pm2.5 : %0.2f \n", data.acc_pm25);
		printf("acc Temeratur : %0.2f \n", dht.acc_temp);
		printf("acc Humidity : %0.2f \n", dht.acc_humi);
		printf("avg Pm10 : %0.2f \n", data.avg_pm10);
		printf("avg Pm2.5 : %0.2f \n", data.avg_pm25);
		printf("avg Temeratur : %0.2f \n", dht.avg_tmp);
		printf("avg Humidity : %0.2f \n", dht.avg_hum);
		printf("CRITICAL ; GOOD BYE . . . \n ")  ;	
		#endif
//	restore_ser(fd);
	return 0;

}





