/*
 * robotd.c
 * This code runs the robot server on the beaglebone black
 * It creates a daemon that listens on the passed port and 
 * waits for a controller to connect.  If the connection is broken, 
 * then it safes the robot and goes back to listening 
 *  
 * Written by: 
 *   Mike Anderson
 *   robot_maker12@verizon.net
 *  
 * Based on code by: 
 *   Nathan Catt
 *   https://github.com/cattnb/Xbox-Controller-to-Wild-Thumper
 *   Thanks, Nathan!
 *  
 */

#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <errno.h>
#include <error.h>
#include <strings.h>
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
#include <syslog.h>
#include <poll.h>
#include "i2c-dev.h"
#include "i2cbusses.h"

#define FALSE  0                  // Define FALSE
#define TRUE   1                  // Define TRUE
#define OK     0                  // Define OK (Success)
#define ERROR -1                  // Define ERROR (Failure)
#define TREX_I2C_ADDR "0x7"       // Address of the TREX Controller
#define TREX_MSG_SIZE 27          // Size of a valid driver station message
#define STATUS_MSG_SIZE 24        // Size of a TREX return status message
#define CONTROLLER_HEADER 0x0F    // First byte of driver station to TREX message
#define STATUS_HEADER 0xF0        // First byte of TREX status message
#define I2CBUS 1                  // Which I2C Bus are we using
#define DEBUG_CONTROLLER FALSE    // Define to enable TREX debugging traffic
#define STARTBYTE 0x0F
#define I2CBUS 1
#define DELAY 50000
#define LOWBAT 700                // adjust to suit your battery: 550 = 5.50V
#define I2CADDRESS 0x07           // default I2C address of T'REX is 7.
#define I2CFREQ  0                // I2C clock frequency. Default is 0=100kHz. Set to 1 for 400kHz

/****************************************************************
 * Externs used by getopt
 ****************************************************************/
extern char *optarg; 
extern int optind, opterr, optopt; 

/****************************************************************
 * Global variables
 ****************************************************************/
int keepgoing = 1;      // Set to 0 when ctrl-c is pressed

//define portno as the port number that the host will set up as a default
int portno = 1929;
int i2cbus = 2;        // default for the BBB pins 19, 20
int bDaemon = FALSE;    // default to running from command line
int sockfd, n, i2cfile, fd, newsockfd=-1, client_len;
struct sockaddr_in server_addr, client_addr;
char i2c_filename[255];    // i2c filename
const char *pidfilename = "/var/run/robotd.pid";
const char *configfilename = "/etc/robotd.conf";

/****************************************************************
 * Prototypes
 ****************************************************************/
int  setup_i2c(void);
int  setup_socket(void);
int  relay_to_driver_station(void);
int  relay_to_trex(void);
int safe_robot(void);
void ctrl_c_sig_handler(int sig);
void sighup_sig_handler(int sig);
void sigalrm_sig_handler(int sig);
void daemonize_me(void);
void print_or_log(char *);
void print_usage(char *);

/* 
 * This code was lifted from a stackoverflow message showing the basic steps to
 * make an application a daemon.  The answer came from Pascal Werkl -- thanks to Pascal
 * whoever he is... 
 */
void daemonize_me() {
    pid_t pid;
    int indx;
    FILE *pid_file;

    /* Fork off the parent process */
    pid = fork();

    /* An error occurred */
    if (pid < 0) exit(EXIT_FAILURE);

    /* Success: Let the parent terminate */
    if (pid > 0) exit(EXIT_SUCCESS);

    /* On success: The child process becomes session leader */
    if (setsid() < 0) exit(EXIT_FAILURE);

    /* Ignore SIGCHILD if we get it */
    signal(SIGCHLD, SIG_IGN);

    /* Fork off for the second time*/
    pid = fork();

    /* An error occurred */
    if (pid < 0) exit(EXIT_FAILURE);

    /* Success: Let the parent terminate */
    if (pid > 0) exit(EXIT_SUCCESS);

    /* Set new file permissions */
    umask(0);

    /* Change the working directory to the root directory */
    /* or another appropriated directory */
    chdir("/");

    /* Close all open file descriptors */
    for (indx = sysconf(_SC_OPEN_MAX); indx > 0; indx--) {
        close(indx);
    }

    // Save the PID so we can kill it without doing a ps
    if ((pid_file = fopen(pidfilename, "w")) != NULL) {
        fprintf(pid_file, "%d", getpid());
        fclose(pid_file);
    }

    /* redirect stdio */
    open ("/dev/null",O_RDWR);  // STDIN
    (void) dup(0);              // STDOUT
    (void) dup(0);              // STDERR

    /* Open the log file */
    openlog("robotd", LOG_PID, LOG_DAEMON);
}


/****************************************************************
 * signal handlers
 ****************************************************************/
// Callback called when SIGINT is sent to the process (Ctrl-C)
void ctrl_c_sig_handler(int sig) {
    printf("Ctrl-C pressed, cleaning up and exiting..\n");
    kill(getpid(), SIGIO);  // Abort the blocking read
    keepgoing = 0;
}

// Callback called when SIGHUP (1) is sent to the process
void sigalrm_sig_handler(int sig) {
    printf("Got SIGALRM...\n");
    kill(getpid(), SIGHUP);
    // We want to restart everything here
}

// Callback called when SIGHUP (1) is sent to the process
void sighup_sig_handler(int sig) {
    printf("Got SIGHUP...\n");
    safe_robot();
    // We want to restart everything here
}

/*
 * Initialize the I2C 
 *  
 */
int setup_i2c(void) {
    char log_msg[255];   

    sprintf(i2c_filename, "/dev/i2c-%d", i2cbus);
    // Open the I2C bus
    if ((i2cfile = open(i2c_filename, O_RDWR)) < 0) {
        sprintf(log_msg, "Failed to open the bus.\n");
        print_or_log(log_msg);
        return(ERROR);
    }

    // Set the TREX up as a I2C slave
    int adr = parse_i2c_address(TREX_I2C_ADDR);
    if (ioctl(i2cfile, I2C_SLAVE, adr) < 0) {
        sprintf(log_msg, "Failed to acquire bus access and/or talk to slave.\n");
        print_or_log(log_msg);
        return(ERROR);
    }

    // Put the robot into a known state
    if (safe_robot() != OK) {
        return(ERROR);
    }

    return(OK);
}

/*
 * Initialize the socket 
 *  
 */
int setup_socket(void) {

   int optval = 1;
   socklen_t optlen = sizeof(optval);
   char log_msg[255];   

   // If the socket was previously open, shut it down
   if (newsockfd >= 0) {
       shutdown(newsockfd, SHUT_RDWR);
       close(newsockfd);
   } else {
       //setup listening socket
       sockfd = socket(AF_INET, SOCK_STREAM, 0);
       if (sockfd < 0) {
           sprintf(log_msg, "ERROR opening socket\n");
           print_or_log(log_msg);
           return(ERROR);
       }

       // bind the listening server socket
       bzero((char *)&server_addr, sizeof(server_addr));
       server_addr.sin_family = AF_INET;
       server_addr.sin_addr.s_addr = INADDR_ANY; //Accept from any connector
       server_addr.sin_port = htons(portno);
       if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
           sprintf(log_msg, "ERROR on binding.\n");
           print_or_log(log_msg);
           return(ERROR);
       }

       // Mark for listening
       listen(sockfd, 5);
       client_len = sizeof(client_addr);
   }

    sprintf(log_msg, "Waiting for connection from the driver station\n");
    print_or_log(log_msg);
    // blocking call to wait for a connection request from the driver station
    newsockfd = accept(sockfd, (struct sockaddr *)&client_addr, &client_len);

    if (newsockfd < 0) {
        sprintf(log_msg, "ERROR on accept\n");
        print_or_log(log_msg);
        return(ERROR);
    } else {
        // Set some socket options
        sprintf(log_msg, "Successfull connection!\n");
        print_or_log(log_msg);
        /* We'll leave the Nagle algorithm in effect to reduce packet count */
        /*
         * enable SO_REUSEADDR so we can restart the server immediately
         */
        if (setsockopt(newsockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&optval, optlen) < 0) {
            sprintf(log_msg, "setsockopt SO_REUSEADDR failed\n");
            print_or_log(log_msg);
            close(newsockfd);
            return(ERROR);
        }
        /*
         *  Here we turn on keepalives to give us some modicum of safety
         *  We'll start keepalives after the first second and then send
         *  1 per second.  If we miss 2 in a row, declare the link down,
         *  safe the robot and clean up
         */
        // Turn on keepalive acks
        // I tried this on Ubuntu 13.04, but it doesn't seem to work
        // Nonetheless, it doesn't seem to break anything so I'll leave it
        // here for future reference
        optval = 1;
        if (setsockopt(newsockfd, SOL_SOCKET, SO_KEEPALIVE, (char *)&optval, optlen) < 0) {
            sprintf(log_msg, "setsockopt SO_KEEPALIVE failed\n");
            print_or_log(log_msg);
            close(newsockfd);
            return(ERROR);
        }
        // allow 2 dropped acks before declaring the link down
        optval = 2;
        if (setsockopt(newsockfd, SOL_TCP, TCP_KEEPCNT, (char *)&optval, optlen) < 0) {
            sprintf(log_msg, "setsockopt TCP_KEEPCNT failed\n");
            print_or_log(log_msg);
            close(newsockfd);
            return(ERROR);
        }
        // Wait no more than 1 second before sending the keepalive probes
        optval = 1;
        if (setsockopt(newsockfd, SOL_TCP, TCP_KEEPIDLE, (char *)&optval, optlen) < 0) {
            sprintf(log_msg, "setsockopt TCP_KEEPIDLE failed\n");
            print_or_log(log_msg);
            close(newsockfd);
            return(ERROR);
        }
        // Send keepalive probes every second
        optval = 1;
        if (setsockopt(newsockfd, SOL_TCP, TCP_KEEPINTVL, (char *)&optval, optlen) < 0) {
            sprintf(log_msg, "setsockopt TCP_KEEPINTVL failed\n");
            print_or_log(log_msg);
            close(newsockfd);
            return(ERROR);
        }
    }
    return(OK);
}

int relay_to_driver_station(void) {
    __u8 buffer[STATUS_MSG_SIZE];
    char log_msg[255];   

    // read from the TREX driver station
    if (read(i2cfile, buffer, sizeof(buffer) < 0)) {
        sprintf(log_msg, "Failed to read from I2C\n");
        print_or_log(log_msg);
        return(ERROR);
    }

    if (buffer[0] != STATUS_HEADER) {
        sprintf(log_msg, "Return status Header Error!\n");
        print_or_log(log_msg);
        return(ERROR);
    }

    if (buffer[1] != 0) {
        // We have an error
        sprintf(log_msg, "Error return code = 0x%x\n", (int)buffer[1]);
        print_or_log(log_msg);
        return(ERROR);
    }

#if DEBUG_CONTROLLER
    // We probably don't want to do this because it increases the network traffic too much
    // write Motor Controller data to connected driver station socket
    if (write(newsockfd, buffer, sizeof(buffer)) < 0) {
        sprintf(log_msg, "Failed to write to socket\n");
        print_or_log(log_msg);
    }
#endif
    return(OK);
}

// Put the robot into a known safe condition
int safe_robot() {
    unsigned int sbyte  = STARTBYTE;
    unsigned int pfreq  = 6;    // default PWM Frequency 122Hz
    int lspeed          = 0;    // Neutral for motor driver station
    unsigned int lbrake = 0;    // disable electronic braking
    int rspeed          = 0;
    unsigned int rbrake = 0;
    int sv0             = 0;    // Set the servos to "off"
    int sv1             = 0;
    int sv2             = 0;
    int sv3             = 0;
    int sv4             = 0;
    int sv5             = 0;
    unsigned int dev    = 50;  // default de-vibrate value from TRex manual
    int sens            = 50;  // impact sensitivity value from TRex manual
    char log_msg[255];   

    __u8 send_data[TREX_MSG_SIZE] = {
        sbyte,
        pfreq,
        ((lspeed >> 8) & 0xFF),
        (lspeed & 0xFF),
        lbrake,
        ((rspeed >> 8) & 0xFF),
        (rspeed & 0xFF),
        rbrake,
        ((sv0 >> 8) & 0xFF),
        (sv0 & 0xFF),
        ((sv1 >> 8) & 0xFF),
        (sv1 & 0xFF),
        ((sv2 >> 8) & 0xFF),
        (sv2 & 0xFF),
        ((sv3 >> 8) & 0xFF),
        (sv3 & 0xFF),
        ((sv4 >> 8) & 0xFF),
        (sv4 & 0xFF),
        ((sv5 >> 8) & 0xFF),
        (sv5 & 0xFF),
        dev,
        ((sens >> 8) & 0xFF),
        (sens & 0xFF),
        ((LOWBAT >> 8) & 0xFF),
        (LOWBAT & 0xFF),
        I2CADDRESS,
        I2CFREQ };

    // Write to the TREX
    // THis is a blocking call, so we won't return until everything's queued
    if (write(i2cfile, send_data, sizeof(send_data)) < 0) {
        sprintf(log_msg, "Failed to write to I2C\n");
        print_or_log(log_msg);
        return(ERROR);
    }
    return(OK);
}

int relay_to_trex(void) {
    int num_bytes;
    int buffer_index = 0;
    int num_left = TREX_MSG_SIZE;
    __u8 buffer[TREX_MSG_SIZE];
    char log_msg[255];   
    struct pollfd pfd;

    pfd.fd = newsockfd;
    pfd.events = POLLIN | POLLHUP | POLLRDNORM;
    pfd.revents = 0;

    // Look for events so we can determine if the connection has dropped
    while (pfd.revents == 0) {
        // wait for 200ms for data
        if (poll(&pfd, 1, 200) > 0) {
            // if the result > 0, it means we either have data or the socket is closed
            if (recv(newsockfd, buffer, TREX_MSG_SIZE, MSG_PEEK | MSG_DONTWAIT) == 0) {
                // if we get 0, that means the connection was closed
                shutdown(newsockfd, SHUT_RDWR);
                close(newsockfd);
                return(ERROR);
            }

            // Read user input from driver station socket
            // This is a blocking socket, but the protocol stack is free to deliver
            // the data as it sees fit
            // So, make sure we get TREX_MSG_SIZE bytes before continuing
            while (buffer_index < TREX_MSG_SIZE) {
                num_bytes = read(newsockfd, &buffer[buffer_index], num_left);
                if (num_bytes < 0) {
                    sprintf(log_msg, "Failed to read from socket\n");
                    print_or_log(log_msg);
                    return(ERROR);
                    break;
                } else {
                    // Handle the bookkeeping
                    buffer_index += num_bytes;
                    num_left -= num_bytes;
                }
            }
        }
    }

#if DEBUG_CONTROLLER
    sprintf(log_msg, "Data from server = ");
    print_or_log(log_msg);
    // create the dump message
    for (int indx = 0; indx < (int)sizeof(buffer); indx++) {
        sprintf(log_msg[indx*3], "%02x ", buffer[indx]);
    }
    // place a \n at the end of the buffer
    sprintf(log_msg[(TREX_MSG_SIZE*3 + 1)], "\n");
    print_or_log(log_msg);
#endif

    // Write to the TREX
    // THis is a blocking call, so we won't return until everything's queued
    if (write(i2cfile, buffer, sizeof(buffer)) < 0) {
        sprintf(log_msg, "Failed to write to I2C\n");
        print_or_log(log_msg);
    }
    return(OK);
}

void print_usage(char *program) {
   fprintf(stderr, "Usage: %s [options]\n%s\n%s\n%s\n%s\n", program,
           "-h This help",
           "-d Make the application a daemon",
           "-i The I2C bus you want to use (/dev/i2c-?)",
           "-p The port number you want to listen on");
}

void print_or_log(char *log_msg) {
    if (bDaemon) {
      syslog(LOG_NOTICE, "%s", log_msg);
    } else {
      printf("%s\n", log_msg);
    }
}

void main(int argc, char **argv, char **envp) {

    char log_msg[255];
    int c, index;  /* required for the getopt calls */

    // Set the signal handler for Ctrl-C
    signal(SIGINT, ctrl_c_sig_handler);
    // Set the signal handler for SIGHUP
    signal(SIGHUP, sighup_sig_handler);

    while ((c = getopt(argc, argv, "dhi:p:")) != -1) switch (c) {
        case 'd':
            bDaemon = TRUE;     /* Make us a daemon */
            break;
        case 'h':
        case '?':
            print_usage(argv[0]);
            exit(0);
            break;
        case 'i':
            i2cbus = atoi(optarg);
            sprintf(i2c_filename, "/dev/i2c-%d", i2cbus);
            break;
        case 'p':
            portno = atoi(optarg);
            break;
        case ':':
            if (isprint(optopt)) fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            else fprintf(stderr,
                         "Unknown option character `\\x%x'.\n",
                         optopt);
            exit(1);
        default:
            abort();
        }

    // The use wants us to be a daemon, so be it...
    if (bDaemon) {
        daemonize_me();
    }

    sprintf(log_msg, "port # = %d, I2C bus = %d\n", portno, i2cbus);
    print_or_log(log_msg);

    //set up I2C
    if (setup_i2c() == ERROR) {
        sprintf(log_msg, "I2C bus set up failure!");
        print_or_log(log_msg);
        exit(1);
    }

    //set up driver station socket
    if (setup_socket() == ERROR) {
        sprintf(log_msg, "Failed to set up driver station socket!");
        print_or_log(log_msg);
        exit(1);
    }

    while (keepgoing) {
        //Send socket server data to motor driver station
        if (relay_to_trex() != OK) {
            safe_robot();
            // OK, we got an error reading the socket
            // set up driver station socket again
            if (setup_socket() == ERROR) {
                sprintf(log_msg, "Failed to set up driver station socket in main loop!");
                print_or_log(log_msg);
                exit(1);
            }
        }

#if DEBUG_CONTROLLER
        // Read return status from TREX
        // If we're debugging, send motor controller data to driver station
        if (relay_to_driver_station() == ERROR) {
            //set up I2C again
            if (setup_i2c() == ERROR) {
                sprintf(log_msg, "I2C bus set up failure!");
                print_or_log(log_msg);
                exit(1);
            }
            sprintf(log_msg, "Read status from TRex failed!");
            print_or_log(log_msg);
            // reset the server
            if (setup_socket() == ERROR) {
                sprintf(log_msg, "Failed to set up driver station socket in main loop!");
                print_or_log(log_msg);
                exit(1);
            }            
        }
#endif
    }

}

