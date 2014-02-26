#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <netdb.h>
#include "i2c-dev.h"
#include "i2cbusses.h"

//delay 50ms for communication
#define DELAY 50000
#define h_addr h_addr_list[0]

//define portno as the port number that the host will set up
int portno = 8080;
//define hostname as the IP address of the host computer
char hostname[] = "192.168.2.4";

int sockfd, n, file;
struct sockaddr_in serv_addr;
struct hostent *server;
char *filename = "/dev/i2c-1";

void setup(){
	//create socket for network communication
	if((sockfd=socket(AF_INET,SOCK_STREAM,0))<0){
		printf("Failed to create socket.\n");
	}
	/*
	if(setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST, 1,1)<0){
		printf("Failed to set socket options\n");
	}
	*/
	server = gethostbyname(hostname);
	if(server==NULL){
		printf("ERROR, no such host\n");
		exit(0);
	}
	bzero((char*) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,server->h_length);
	serv_addr.sin_port = htons(portno);
	if(connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0){
		printf("Failed to connect\n");
		return;
	}

	int adr = parse_i2c_address("0x07");
	if ((file = open(filename,O_RDWR)) < 0) {
        	printf("Failed to open the bus.\n");
        	exit(1);
    	}
	if (ioctl(file, I2C_SLAVE, adr) < 0) {
    		printf("Failed to acquire bus access and/or talk to slave.\n");
    		exit(1);
	}
}

void loop(){

	//Send socket server data to motor controller
		MasterSend();
		//send motor controller data to socket server
		MasterReceive();
}

void MasterReceive()
{
	__u8 buffer[24];
  	
  	if(read(file, buffer, sizeof(buffer)<0)){
			//printf("Failed to read from I2C\n");
	}
	
  	//write Motor Controller data to socket server
  	if(write(sockfd,buffer,sizeof(buffer))<0){
  			printf("Failed to write to socket\n");
  	}
	
	
   
}

void MasterSend(){
	int a;
	//printf("Ready to receive info\n");
	__u8 buffer[27];
	//read socket server data
  	if(a=read(sockfd, buffer, sizeof(buffer))<0){
  		printf("Failed to read from socket\n");
  	}
	//printf("%i packets received from server\n",a);
	
	printf("Data from server = ");
	for(int i = 0;i<sizeof(buffer);i++){
		printf("%i ",buffer[i]);
	}
	printf("\n");

	if(write(file,buffer, sizeof(buffer))<0){
  		printf("Failed to write to I2C\n");
  	}
 
}


void main(){
	//set up I2C and socket communication
	setup();
	printf("Setup complete\n");
	
	
	while(1){
		loop();
	}
}
