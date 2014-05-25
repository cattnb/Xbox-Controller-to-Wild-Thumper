#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <errno.h>
#include <error.h>
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
#include <pthread.h>
#include <arpa/inet.h>
#include <time.h>
#include <sys/resource.h>

// Generic defines
#define TRUE   1                  // Define TRUE
#define FALSE  0                  // Define FALSE
#define OK     0                  // Define OK (Success)
#define ERROR -1                  // Define ERROR (Failure)
#define JOYSTICK_TH_PRIORITY 2    // Potential for running the joystick thread at a higher priority
#define READ_STATUS_TH_PRIORITY 1 // Potential for running the read status thread at a higher priority
#define DEBUG_CONTROLLER FALSE    // Set to TRUE to enable TREX debugging traffic

// TRex-specific defines
#define STARTBYTE 0x0F
#define I2CBUS 1
#define DELAY 50000
#define LOWBAT 700                // adjust to suit your battery: 550 = 5.50V
// If this is changed, the T'REX will automatically store new address in EEPROM
#define I2CADDRESS 0x07           // default I2C address of T'REX is 7.
#define I2CFREQ  0                // I2C clock frequency. Default is 0=100kHz. Set to 1 for 400kHz
#define TREX_MSG_SIZE 27          // Size of a valid controller message
#define STATUS_MSG_SIZE 24        // Size of a TREX return status message
#define CONTROLLER_HEADER 0x0F    // First byte of controller to TREX message
#define STATUS_HEADER 0xF0        // First byte of TREX status message

// XBox or Logitech F-310 settings
// Axis Settings
#define LEFT_JS_X 0              
#define LEFT_JS_Y 4               // I had to switch these because of how I had the robot wired
#define LEFT_TRIGGER 2
#define RIGHT_JS_X 3
#define RIGHT_JS_Y 1              // I had to switch these because of how I had the robot wired
#define RIGHT_TRIGGER 5
#define DPAD_X 6
#define DPAD_Y 7

// Button Settings
#define BTN_A_GREEN 0
#define BTN_B_RED 1
#define BTN_X_BLUE 2
#define BTN_Y_YELLOW 3
#define LEFT_BTN 4
#define RIGHT_BTN 5
#define BACK_BTN 6
#define START_BTN 7
#define MODE_BTN 8
#define LEFT_STICK_BTN 9
#define RIGHT_STICK_BTN 10
#define DEAD_LOW  -140             // Define the dead band for the Joystick
#define DEAD_HIGH  140
#define NEUTRAL 128

// Scaling the power output to keep from tripping the breakers on 6-wd Wild Thumper
#define FULL_POWER 0.007781
#define THREE_QTR_POWER 0.006103
#define HALF_POWER 0.003890

/****************************************************************
 * Global variables
 ****************************************************************/
int keepgoing = TRUE;                     // Set to FALSE when ctrl-c is pressed

int sv[6] = { 0, 0, 0, 0, 0, 0 };         // servo positions: 0 = Not Used
int sd[6] = { 5, 10, -5, -15, 20, -20 };  // servo sweep speed/direction
int lmspeed = 128, rmspeed = 128;         // left and right motor speed from -255 to +255 (negative value = reverse)
int old_lmspeed = 128, old_rmspeed = 128; // left and right motor speed from -255 to +255 (negative value = reverse)
int ldir = 10;                            // how much to change left  motor speed each loop (use for motor testing)
int rdir = 10;                            // how much to change right motor speed each loop (use for motor testing)
int both_dir = 4;                         // change both motor speeds
unsigned int lmbrake = 0,rmbrake = 0;     // left and right motor brake (non zero value = brake)
unsigned int pwmf = 6;                    // Default PWM freq -- 122Hz
unsigned int devibrate = 50;              // time delay after impact to prevent false re-triggering due to chassis vibration
int sensitivity = 50;                     // threshold of acceleration / deceleration required to register as an impact
pthread_t senderThId, readstatusThId;     // pthread IDs from the pthread_create calls
int  *axis = NULL;                        // Pointer to calloced axis array
char *button = NULL;                      // Pointer to calloced button array
pthread_mutex_t data_mutex;               // protect the motor value critical region
int bUsePriorities = FALSE;
int bConnected = FALSE;

struct js_event xbox;                     // The joystick -- Using Logitech F-310 controller 
int joystick_fd;                          // Access the joystick
int sockfd = -1;                          // Socket-related stuff   
int client_len;                           // Socket-related stuff   
struct sockaddr_in robot_addr, client_addr;

// Default port number
int portno = 1929;
// Define default hostname as the IP address of the robot
char hostname[] = "192.168.4.2";

/****************************************************************
 * Externs used by getopt
 ****************************************************************/
extern char *optarg; 
extern int optind, opterr, optopt; 

// Prototypes
void setup(void);
void rcv_from_robot(void);
void send_to_robot(unsigned int sbyte,
                   unsigned int pfreq,
                   int lspeed,
                   unsigned int lbrake,
                   int rspeed,
                   unsigned int rbrake,
                   int sv0,
                   int sv1,
                   int sv2,
                   int sv3,
                   int sv4,
                   int sv5,
                   unsigned int dev,
                   int sens,
                   int lowbat,
                   unsigned int i2caddr,
                   unsigned int i2cfreq);
void  signal_handler(int sig);
void* sender_thread(void *arg);
void* read_status_thread(void *arg);
void  print_usage(char *);

/****************************************************************
 * signal_handler
 ****************************************************************/
// Callback called when SIGINT is sent to the process (Ctrl-C)
void signal_handler(int sig) {
    printf("Ctrl-C pressed, cleaning up and exiting..\n");
    keepgoing = FALSE;
    // Terminate the blocked read from the socket
    close(sockfd);
    pthread_kill(readstatusThId, SIGTERM);
    pthread_kill(senderThId, SIGTERM);
}

/****************************************************************
 * Sender thread
 ****************************************************************/
// In linux, the joystick is supposed to be blocking,
// so we'll create a thread that blocks to read it
void* sender_thread(void *arg) {
    struct timespec ts;

    if (pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL)) {
        printf("Failed to set the cancel type for rcv_from_robot");
        exit(1);
    }

    sleep(1);  // give main a chance to finish

    while (keepgoing) {
        // Are we connected yet?
        if (bConnected) {
            pthread_mutex_lock(&data_mutex);

            // printf("lmspeed = %4d ", lmspeed); 
            // printf("rmspeed = %4d\r", rmspeed);
            
            send_to_robot(STARTBYTE,
                       pwmf,
                       lmspeed,
                       lmbrake,
                       rmspeed,
                       rmbrake,
                       sv[0],
                       sv[1],
                       sv[2],
                       sv[3],
                       sv[4],
                       sv[5],
                       devibrate,
                       sensitivity,
                       LOWBAT,
                       I2CADDRESS,
                       I2CFREQ);
            
            pthread_mutex_unlock(&data_mutex);
            // Go to sleep for 50ms
            ts.tv_sec = 0;
            ts.tv_nsec = 50000000;
            nanosleep(&ts, NULL);
            old_lmspeed = lmspeed;
            old_rmspeed = rmspeed;
        } else {
            // Nope.  So take a nap and check later
            sleep(1);
        }

    }
    pthread_exit(0);
}

/****************************************************************
 * Read return thread
 ****************************************************************/
// Read the return status if we ever get anything back
void* read_status_thread(void *arg) {

    if (pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL)) {
        printf("Failed to set the cancel type for rcv_from_robot");
        exit(1);
    }

    while (keepgoing) {
        // are we connected yet?
        if (bConnected) {
            rcv_from_robot();
        } else {
            // Nope.  So, go to sleep for a second
            sleep(1);
        }
    }
    pthread_exit(0);
}

void setup(void) {
    int num_bytes;
    int res;
    int c, index;  /* required for the getopt calls */
    void *thread_result;
    pthread_attr_t attr;
    struct sched_param priority;

    //setupjs
    joystick_fd = open("/dev/input/js0", O_RDONLY);
    if (joystick_fd < 0) {
        printf("Failed to open joystick");
        return;
    }
    printf("Opened joystick...\n");
    int axes = 0, buttons = 0;
    char name[128];
    ioctl(joystick_fd, JSIOCGAXES, &axes);
    ioctl(joystick_fd, JSIOCGBUTTONS, &buttons);
    ioctl(joystick_fd, JSIOCGNAME(sizeof(name)), &name);
    axis = (int *)calloc(axes, sizeof(int));
    button = (char *)calloc(buttons, sizeof(char));

    printf("%s\n  %d Axes %d Buttons\n", name, axes, buttons);

    /* Go ahead and initialize the mutex in case we need it */
    res = pthread_mutex_init(&data_mutex, NULL);
    if (res != 0) {
        perror("Mutex initialization failed");
        exit(EXIT_FAILURE);
    }

    /* Initialize the thread attribute structure */
    res = pthread_attr_init(&attr);
    if (res != 0) {
        perror("Attribute initialization failed");
        exit(EXIT_FAILURE);
    }

    if (bUsePriorities) {
        /* default for NPTL is PTHREAD_IMPLICIT_SCHED */
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

        res = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (res != 0) {
            perror("setschedpolicy initialization failed");
            exit(EXIT_FAILURE);
        }

        priority.sched_priority = JOYSTICK_TH_PRIORITY;
        res = pthread_attr_setschedparam(&attr, &priority);
        if (res != 0) {
            perror("pthread_attr_setschedparam failed");
            exit(EXIT_FAILURE);
        }
    }

    res = pthread_create(&senderThId, &attr, sender_thread, NULL);
    if (res != 0) {
        perror("Joystick thread creation failed");
        exit(EXIT_FAILURE);
    }

    if (bUsePriorities) {
        priority.sched_priority = READ_STATUS_TH_PRIORITY;
        res = pthread_attr_setschedparam(&attr, &priority);
    }

    res = pthread_create(&readstatusThId, &attr, read_status_thread, NULL);
    if (res != 0) {
        perror("Read status thread creation failed");
        exit(EXIT_FAILURE);
    }

    pthread_attr_destroy(&attr);

}

void send_to_robot(unsigned int sbyte,
                unsigned int pfreq,
                int lspeed,
                unsigned int lbrake,
                int rspeed,
                unsigned int rbrake,
                int sv0,
                int sv1,
                int sv2,
                int sv3,
                int sv4,
                int sv5,
                unsigned int dev,
                int sens,
                int lowbat,
                unsigned int i2caddr,
                unsigned int i2cfreq) {
    int num_bytes;

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

    // This should be a blocking socket, so we'll wait here until
    // everything's written to the stack
    // Unfortunately, the protocol stack can deliver to the robot
    // in any way it feels like
    num_bytes = write(sockfd, send_data, TREX_MSG_SIZE);
    if (num_bytes < 0) perror("ERROR writing to socket\n");
    return;
}

void rcv_from_robot(void) {
    int num_bytes;
    int i;
    int count = 0;
    int buffer_index = 0;
    int num_left = STATUS_MSG_SIZE;
    __u8 d[STATUS_MSG_SIZE] = { 0 };

    // Handle perverse IP stack delivery cases
    while (buffer_index < STATUS_MSG_SIZE) {
        num_bytes = read(sockfd, &d[buffer_index], num_left);
        if (num_bytes < 0) {
            printf("Failed to read from socket\n");
            return;
            break;
        } else {
            // Handle the bookkeeping
            buffer_index += num_bytes;
            num_left -= num_bytes;
        }
    }
    return;
}

void print_usage(char *program) {
   fprintf(stderr, "Usage: %s [options]\n%s\n%s\n%s\n", program,
           "-h This help",
           "-i The IPv4 address of the robot listener",
           "-p The port number you want to connect to");
}


int main(int argc, char *argv[]) {
    int num_bytes;
    int res;
    int c, index;          // required for the getopt calls
    int indx;
    char hostIP[16];       // Big enough to hold max IPv4 address string 
    void *thread_result;
    int optval = 1;
    socklen_t optlen = sizeof(optval);

    setup();

    // Set the signal callback for Ctrl-C
    signal(SIGINT, signal_handler);

    // zero out the socket structure
    memset((char *)&robot_addr, '\0', sizeof(robot_addr));

    // Set the default robot IP address
    memcpy(hostIP, hostname, strlen(hostname));  // save IP for later
    if (inet_pton(AF_INET, hostname, &robot_addr.sin_addr) <= 0) {
        printf("\n inet_pton error occured setting default robot IP\n");
        exit(1);
    }

    while ((c = getopt(argc, argv, "hi:p:")) != -1) switch (c) {
        case 'h':
        case '?':
            print_usage(argv[0]);
            exit(0);
            break;
        case 'i':
            // They entered the host addr
            // get the host info
            memcpy(hostIP, optarg, strlen(optarg));  // save IP for later
            if (inet_pton(AF_INET, optarg, &robot_addr.sin_addr) <= 0) {
                printf("\n inet_pton error occured\n");
                exit(1);
            }
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

    // This call should always succeed 
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket()");
        exit(1);
    } 

    // host byte order
    robot_addr.sin_family = AF_INET;
    // short, network byte order
    printf("driver_station - The remote host is: %s and port number is %d \n", hostIP, portno);

    robot_addr.sin_port = htons(portno);

    printf("Waiting for the connection to the robot...\n");
    // Try to make a connection 3x if no connection by then give up 
    for (indx=0; indx < 3; indx++) {
        if (connect(sockfd, (struct sockaddr *)&robot_addr, sizeof(robot_addr)) == -1) {
            perror("connect()");
            printf("Trying again...\n");
        } else {
            bConnected = TRUE;
            printf("driver_station - The connect() is OK...\n");
            /* We'll leave the Nagle algorithm in effect to reduce packet count */
            /*
             * enable SO_REUSEADDR so we can restart the server immediately
             */
            if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&optval, optlen) < 0) {
                printf("setsockopt SO_REUSEADDR failed\n");
                close(sockfd);
                return(ERROR);
            }
            /*
             *  Here we turn on keepalives to give us some modicum of safety
             *  We'll start keepalives after the first second and then send
             *  1 per second.  If we miss 5 in a row, declare the link down,
             *  safe the robot and clean up
             */
            // Turn on keepalive acks
            if (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (char *)&optval, optlen) < 0) {
                printf("setsockopt SO_KEEPALIVE failed\n");
                close(sockfd);
                return(ERROR);
            }
            // allow 5 dropped acks before declaring the link down
            optval = 5;
            if (setsockopt(sockfd, SOL_TCP, TCP_KEEPCNT, (char *)&optval, optlen) < 0) {
                printf("setsockopt TCP_KEEPCNT failed\n");
                close(sockfd);
                return(ERROR);
            }
            // Wait no more than 1 second before sending the keepalive probes
            optval = 1;
            if (setsockopt(sockfd, SOL_TCP, TCP_KEEPIDLE, (char *)&optval, optlen) < 0) {
                printf("setsockopt TCP_KEEPIDLE failed\n");
                close(sockfd);
                return(ERROR);
            }
            // Send keepalive probes every second
            optval = 1;
            if (setsockopt(sockfd, SOL_TCP, TCP_KEEPINTVL, (char *)&optval, optlen) < 0) {
                printf("setsockopt TCP_KEEPINTVL failed\n");
                close(sockfd);
                return(ERROR);
            }

            break;
        }
    }

    if (bConnected) {
        while (keepgoing) {
            if (read(joystick_fd, &xbox, sizeof(struct js_event)) < 0) {
                perror("Joystick read failed");
            } else {
                //	CHECK THE EVENT
                switch (xbox.type & ~JS_EVENT_INIT) {
                case JS_EVENT_AXIS:
                    axis[xbox.number] = xbox.value;
                    if (xbox.number == LEFT_JS_Y) {
                        /* invert the forward vs. backwards*/
                        axis[LEFT_JS_Y] *= -1;
                    }
                    if (xbox.number == RIGHT_JS_Y) {
                        /* invert the forward vs. backwards*/
                        axis[RIGHT_JS_Y] *= -1;
                    }
                    if (xbox.number == DPAD_Y) {
                        /* invert the forward vs. backwards*/
                        axis[DPAD_Y] *= -1;
                    }
                    break;
                case JS_EVENT_BUTTON:
                    button[xbox.number] = xbox.value;
                    break;
                }

                // Lock against modding the lmspeed and rmspeed 
                pthread_mutex_lock(&data_mutex);

                if ((axis[LEFT_JS_Y] > DEAD_HIGH) || (axis[LEFT_JS_Y] < DEAD_LOW)) { 
                  lmspeed = (HALF_POWER * axis[LEFT_JS_Y]);
                } else {
                    // set it to stop
                    lmspeed = 0;
                }
                
                if ((axis[RIGHT_JS_Y] > DEAD_HIGH) || (axis[RIGHT_JS_Y] < DEAD_LOW)) {
                  rmspeed = (HALF_POWER * axis[RIGHT_JS_Y]);
                } else {
                    // set it to stop
                    rmspeed = 0;
                }
                // release the lock around the lmspeed and rmspeed variables
                pthread_mutex_unlock(&data_mutex);
            }
        }
    } else {
        printf("Unable to connect to the robot!  Exiting...\n");
        // Shut 'er down...
        keepgoing=FALSE;
        pthread_kill(readstatusThId, SIGTERM);
        pthread_kill(senderThId, SIGTERM);
    }

    printf("\nMain is waiting for threads to finish...\n");
    res = pthread_join(senderThId, &thread_result);
    if (res != 0) {
        perror("Thread join failed");
        exit(EXIT_FAILURE);
    }

    printf("driver_station terminated\n");

    return (EXIT_SUCCESS);
}

