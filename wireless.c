#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <syslog.h>
#include "wireless.h"
#define DEBUG_LVL_1 1
#define DEBUG_LVL_2 1
#define DEBUG_DATA 1
#define DEBUG_COUNTER1 1
#define DEBUG_COUNTER2 1
#define DATA "||"

#define RED		"\x1b[31m"
#define GREEN   "\x1b[32m"
#define YELLOW  "\x1b[33m"
#define CLR		"\x1b[0m"


unsigned char local_buffer[8];
unsigned char prev_buf[8];
// typedef int (*FuncPtr)(unsigned char *);
int i;
char *portname0 = "/dev/ttyUSB0";
char *portname1 = "/dev/ttyUSB1";
int fd;
int wlen;
int loop_end = 0;
int detach_connection = 0;
int detach_data = 0;
int re_init_data = 0;
pthread_t thread_data;
pthread_t thread_connection;
FuncPtr gpTestFP = NULL;
int read_UART (void);
int subscribeCB (FuncPtr pFunc);
int read_in_progress = 0;
int wait_connect_start,wait_connect_finish;

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
		#if DEBUG_LVL_1 
        printf(" Error from tcgetattr: %s\n", strerror(errno));
		#endif
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		#if DEBUG_LVL_1
        printf(" Error from tcsetattr: %s\n", strerror(errno));
		#endif
        return -1;
    }
    return 0;
}


void *usb_connection( void *arg)
{
	do
	{
		printf("[%s] detach_connection : %d\n",__func__,detach_connection);
		if(detach_connection != 1)
		{
			close(fd);
			fd = open(portname0, O_RDWR | O_NOCTTY | O_SYNC);
			if (fd < 0) {
				#if DEBUG_LVL_1
				printf(RED"[%s] Error opening %s: %s\n"CLR,__func__, portname0, strerror(errno));
				#endif
				close(fd);
				
				fd = open(portname1, O_RDWR | O_NOCTTY | O_SYNC);
				if (fd < 0) {
					#if DEBUG_LVL_1
					printf(RED"[%s] Error opening %s: %s\n"CLR,__func__, portname1, strerror(errno));
					#endif
						close(fd);
				}
			}
			else
			{
				/*baudrate 115200, 8 bits, no parity, 1 stop bit */
				printf(GREEN"[%s] CONNECTION SUCCESS\n"CLR,__func__);
				set_interface_attribs(fd, B9600);
				detach_connection = 1;
				re_init_data = 1;
			}
			usleep(500000);
		}
		else
		{
			printf("[%s] check connection IDLE\n",__func__);
			sleep(1);
		}
	} while (1);
}

void *loop_function( void *arg)
{
    int counter_DeviceCheck = 0;
    int counter_DeviceCheck2 = 0, diff = 0;
	int count_sts;
    int count=0,i,status,id_rx[7],send_Msg =0;
    int var1 = 0;
    int first_in = 1;
    unsigned char prev = 0;
	int read_start;
	int check_data = 0;
	int check_init = 0;
    
        do 
        {
			printf("[%s] detach_connection : %d\n",__func__,detach_connection);
            unsigned char uart_buf[80];
            
            unsigned char incomingID_flag = 0;
			unsigned char temp_data = 0;
            
            int rdlen;
            
			printf("[%s] detach_connection : %d\n",__func__,detach_connection);
			
			if(detach_connection == 1)
			{
				if (re_init_data == 1)
				{
					counter_DeviceCheck = 0;
					counter_DeviceCheck2 = 0, diff = 0;
					count_sts;
					count=0,i,status,id_rx[7],send_Msg =0;
					var1 = 0;
					first_in = 1;
					prev = 0;
					re_init_data = 0;
					incomingID_flag = 0;
					incomingID_flag = 0;
					check_data = 0;
					check_init = 0;
					memset(uart_buf,0,sizeof(uart_buf));
				}
				
				rdlen = read(fd, uart_buf, sizeof(uart_buf) - 1);
				if(rdlen > 0)
				{
					counter_DeviceCheck++;
					unsigned char   *p;
					for (p = uart_buf; rdlen-- > 0; p++)
					{    
						incomingID_flag = *p;
					}
					
					printf(YELLOW"[%s] incomingID_flag : 0x%02x \n"CLR,__func__,incomingID_flag);
					if(read_in_progress)
					{
						printf("[%s] Read Device ID in Progress : %d \n",__func__,read_in_progress);
						count++;
						local_buffer[1+count] = incomingID_flag;
						if (count == 6)
						{
							#if DEBUG_DATA
							printf(GREEN" --local_buffer--- \n");
							printf("| no. of data : %d|\n",count+2);
							printf(" ----------------- \n");
							
							for (i = 0; i < 8; i++)
							{
								printf("| byte[%d] : 0x%02x |\n",i,local_buffer[i]);   
							}
							printf(" ----------------- \n"CLR);
							
							#endif
								read_in_progress = 0;
								count = 0;
							if ( gpTestFP != NULL && first_in && local_buffer[0] != 0 && local_buffer[0]==var1 && local_buffer[1] == 0x71 && local_buffer[0] != 0x61)
							{
								#if DEBUG_LVL_2
								printf(" send msg from ID read\n");
								#endif
								gpTestFP((unsigned char *)&local_buffer);
								memset(local_buffer,0,sizeof(local_buffer));
								first_in = 0;
							}
							else if (local_buffer[0] == 0x61 || local_buffer[0] == 0x64 || local_buffer[0] == 0x65 || local_buffer[0] == 0x66)
							{
								temp_data = local_buffer[0];
								memset(local_buffer,0,sizeof(local_buffer));
								local_buffer[0] = temp_data;
								gpTestFP((unsigned char *)&local_buffer);
								memset(local_buffer,0,sizeof(local_buffer));
								temp_data = 0;
								first_in = 0;
							}
						}
					}    
					else
					{
						#if DEBUG_DATA
						printf(" ------------------------\n");
						printf("| incomingID_flag = 0x%02x |\n",incomingID_flag);
						#endif
						
						if (incomingID_flag == 0x71)
						{
							#if DEBUG_LVL_2
							printf(" reading ID\n");
							#endif
							read_in_progress = 1;
							local_buffer[0] = var1;
							local_buffer[1] = incomingID_flag;
							send_Msg = 0;
						}
						else if ( prev == incomingID_flag)
						{

							switch (incomingID_flag)
							{   
		
								case 0x61:
										#if DEBUG_LVL_2
										printf(" ----------------------------\n");
										printf("| STATUS :: Charger stand by |\n");
										printf(" ----------------------------\n");
										#endif
										count = 0;
										read_in_progress = 0;
										local_buffer[0] = incomingID_flag;
										send_Msg = 1;
										first_in = 1;
								break;
				
								case 0x62:
										#if DEBUG_LVL_2
										printf(" ---------------------------------\n");
										printf("| STATUS :: Basic power profile\n |"); 
										printf(" ---------------------------------\n");
										#endif
										count = 0;
										var1 = incomingID_flag;
										local_buffer[0] = var1;
										read_in_progress = 0;
										send_Msg = 0;
									
								break;
				
								case 0x63:
										#if DEBUG_LVL_2
										printf(" ----------------------------------\n");
										printf("| STATUS :: Extended power profile |\n");
										printf(" ----------------------------------\n");
										#endif
										var1 = incomingID_flag;
										local_buffer[0] = var1;
										read_in_progress = 0;
										count = 0;
										send_Msg = 0;
								break;
		
								case 0x64:
											#if DEBUG_LVL_2
											printf(" -----------------------------------------------------\n");
											printf("| STATUS :: Metal Object Detected during pwr transfer |\n");
											printf(" -----------------------------------------------------\n");
											#endif
											count = 0;
											read_in_progress = 0;
											local_buffer[0] = incomingID_flag;
											send_Msg = 1;
											first_in = 1;
									
								break;
				
								case 0x65:
										#if DEBUG_LVL_2
										printf(" -------------------------------------------------\n");
										printf("| STATUS :: Metal Object Detected during stand by |\n");
										printf(" -------------------------------------------------\n");
										#endif
										count = 0;
										read_in_progress = 0;
										local_buffer[0] = incomingID_flag;
										send_Msg = 1;
										first_in = 1;
								break;
				
								case 0x66:
										#if DEBUG_LVL_2
										printf(" -----------------------------------------------------------------------\n");
										printf("| STATUS :: Metal Object Detected during stand by + receiver on charger |\n");
										printf(" ----------------------------------------------------------------------\n");
										#endif
										count = 0;
										read_in_progress = 0;
										local_buffer[0] = incomingID_flag;
										send_Msg = 1;
										first_in = 1;
									
								break;
								
								case 0x68:
										#if DEBUG_LVL_2
										printf(" ---------------------------------\n");
										printf("| STATUS :: Samsung Fast Charging |\n");
										printf(" ---------------------------------\n");
										#endif
										count = 0;
										var1 = incomingID_flag;
										local_buffer[0] = var1;
										read_in_progress = 0;
										send_Msg = 0;
								break;
								  
								default:
									#if DEBUG_LVL_2
									printf(" ----------------------------------------------\n");
									printf("|STATUS :: Wireless_Charger/Unrecognized UART ID |\n");
									printf(" ----------------------------------------------\n");
									#endif 
								break;
							}
						}
						else
						{
							prev = incomingID_flag;
						}
							
							#if DEBUG_DATA
								printf(GREEN" ---------------- \n");
								printf("| no. of data : 1|\n");
							for (i = 0; i < 8; i++)
							{
								printf("| byte[%d] : 0x%02x |\n",i,local_buffer[i]);
							}
								printf(" ---------------- \n"CLR);
							#endif
							
						if (gpTestFP != NULL && send_Msg)
						{
							send_Msg = 0;
							gpTestFP((unsigned char *)&local_buffer);
							memset(local_buffer,0,sizeof(local_buffer));
						}
							
					}
					
				}
			
				printf("[%s] read start : %d\n",__func__,read_start = 0);
				counter_DeviceCheck--;
				printf("[%s] counter_DeviceCheck : %d\n",__func__,counter_DeviceCheck);
				
				if(counter_DeviceCheck < 0)
				{
					counter_DeviceCheck2++;
				}

				if (counter_DeviceCheck2 == 5)
				{
						printf(RED"[%s] counter_DeviceCheck2 : %d\n"CLR,__func__,counter_DeviceCheck2);
						local_buffer[0] = 0x60;
						gpTestFP((unsigned char *)&local_buffer);
						memset(local_buffer,0,sizeof(local_buffer));
						counter_DeviceCheck2 = 0;
						counter_DeviceCheck = 0;
						detach_connection = 0;
				}
			}
			else
			{
				printf(RED"[%s] Waiting for connection \n"CLR,__func__);
				usleep(500000);
			}
            
        } while (1);
    /* simple noncanonical input */
    
}

int subscribeCB(FuncPtr pFunc)
{
    if (pFunc != NULL)
    {
        gpTestFP = pFunc;
    }
}

int read_UART(void)
{   
    int i,j,ret;
	i=1;
	j=2;
	pthread_t tid;
	pthread_t tid2;
	
	pthread_create(&tid, NULL, usb_connection, (void *)&i); 
	pthread_create(&tid, NULL, loop_function, (void *)&j);

    return 0;
}
