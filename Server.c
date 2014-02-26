#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stddef.h>
#include <sys/un.h>
#include <signal.h>
#include <linux/joystick.h>


#define STARTBYTE 0x0F
#define I2CADDRESS 0x07
#define I2CBUS 1
#define DELAY 50000



int sv[6]={0,0,0,0,0,0};                 // servo positions: 0 = Not Used
int sd[6]={5,10,-5,-15,20,-20};                      // servo sweep speed/direction
int lmspeed=0,rmspeed=0;                                 // left and right motor speed from -255 to +255 (negative value = reverse)
int ldir=10;                                          // how much to change left  motor speed each loop (use for motor testing)
int rdir=10;                                          // how much to change right motor speed each loop (use for motor testing)
int both_dir=4;					//change both motor speeds
unsigned int lmbrake,rmbrake;                                // left and right motor brake (non zero value = brake)
unsigned int pwmf = 5;
unsigned int devibrate=50;                                   // time delay after impact to prevent false re-triggering due to chassis vibration
int sensitivity=50;                                  // threshold of acceleration / deceleration required to register as an impact
int lowbat=700;                                      // adjust to suit your battery: 550 = 5.50V
unsigned int i2caddr=7;                                      // default I2C address of T'REX is 7. If this is changed, the T'REX will automatically store new address in EEPROM
unsigned int i2cfreq=0;                                      // I2C clock frequency. Default is 0=100kHz. Set to 1 for 400kHz






struct js_event xbox;
int fd; 
int sockfd, newsockfd,portno,clilen;
struct sockaddr_in serv_addr, cli_addr;

void setup(){
	//setupjs
	fd = open ("/dev/input/js1", O_RDONLY | O_NONBLOCK);
	if(fd<0){
		printf("Failed to open");
		return;
	}
	printf("opened file\n");
	int axes=0, buttons=0;
  	char name[128];
	ioctl(fd, JSIOCGAXES, &axes);
  	ioctl(fd, JSIOCGBUTTONS, &buttons);
  	ioctl(fd, JSIOCGNAME(sizeof(name)), &name);
 	 printf("%s\n  %d Axes %d Buttons\n", name, axes, buttons);

	//setup socket
	sockfd=socket(AF_INET,SOCK_STREAM,0);
	if(sockfd<0){
		error("ERROR opening socket\n");
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));
	portno =8080;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY; //could be IP of beaglebone
	serv_addr.sin_port = htons(portno);
	if(bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))<0){
		error("ERROR on binding.\n");
		exit(1);
	}
	listen(sockfd,5);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if(newsockfd<0){
		error("ERROR on accept\n");
	}
	//bzero(buffer,256);
}

void MasterSend(unsigned int sbyte, unsigned int pfreq, int lspeed, unsigned int lbrake, int rspeed, unsigned int rbrake, int sv0, int sv1, int sv2, int sv3, int sv4, int sv5, unsigned int dev,int sens,int lowbat, unsigned int i2caddr,unsigned int i2cfreq){
	int n;
	__u8 send_data[27]={sbyte,pfreq,((lspeed>>8)&0xFF), (lspeed&0xFF), lbrake, ((rspeed>>8)&0xFF), (rspeed&0xFF), rbrake, ((sv0>>8)&0xFF), (sv0&0xFF), ((sv1>>8)&0xFF), (sv1&0xFF),((sv2>>8)&0xFF), (sv2&0xFF),((sv3>>8)&0xFF), (sv3&0xFF),((sv4>>8)&0xFF), (sv4&0xFF),((sv5>>8)&0xFF), (sv5&0xFF),dev,((sens>>8)&0xFF), (sens&0xFF),((lowbat>>8)&0xFF), (lowbat&0xFF),i2caddr,i2cfreq};

	n = write(newsockfd,send_data,27);
	if(n<0) error("ERROR reading from socket\n");
	return;
}

void MasterReceive(){
	int n;
	int i;
	int count=0;
	__u8 D[24] = {0};
	__u8 d[24] = {0};
	int index = 0;
	if(n = read(newsockfd,d,24)<0){
		error("ERROR writing to socket\n");
	}
  return;
}


void main(){
	//int turn = 0;
	//int rightspeed = 0;
	//int leftspeed = 0;
	setup();
	
	

	while(1){
		if(read(fd,&xbox,sizeof(xbox))<0){
			
		}
		else{
			
			if(xbox.type==2 && xbox.number==5){
				if(lmspeed>=0){
					lmspeed = 0.003815*(xbox.value)+125;
				}
				else{
					lmspeed = -(0.003815*(xbox.value)+125);
				}
				if(rmspeed>=0){
					rmspeed = 0.003815*(xbox.value)+125;
				}
				else{
					rmspeed = -(0.003815*(xbox.value)+125);
				}
			}
			if(xbox.type==2 && xbox.number==2){
				if(lmspeed>=0){
					lmspeed = -(0.003815*(xbox.value)+125);
				}
				else{
					lmspeed = (0.003815*(xbox.value)+125);
				}
				if(rmspeed>=0){
					rmspeed = -(0.003815*(xbox.value)+125);
				}
				else{
					rmspeed = (0.003815*(xbox.value)+125);
				}
			}
			if(xbox.type==2 && xbox.number==0 && xbox.value<-10000){
				//turn = xbox.value;
				printf("Turning left\n");
				float b = 1.878464*rmspeed;
				float m = (8.7846e-5)*(float)rmspeed;
				lmspeed = m*xbox.value+b;
				
				
			}
			if(xbox.type==2 && xbox.number==0 && xbox.value>4000){
				printf("Turning right\n");
				//turn = xbox.value;
				float b = 1.278*lmspeed;
				float m = -(6.95241e-5)*(float)lmspeed;
				rmspeed = m*xbox.value+b;
			}
			printf("lmspeed = %i ",lmspeed);
			printf("rmspeed = %i\n", rmspeed);
		}
		
		MasterSend(STARTBYTE,pwmf,lmspeed,lmbrake,rmspeed,rmbrake,sv[0],sv[1],sv[2],sv[3],sv[4],sv[5],devibrate,sensitivity,lowbat,i2caddr,i2cfreq);
		//usleep(DELAY);
		MasterReceive();
		//usleep(DELAY);
	}	
}
