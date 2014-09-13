/*
 * This node makes a serial connection with the joystick for
 * controlling the end-effector of the Davinci Robot
 *
 */

// Libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <sys/ioctl.h>


// Namespaces
using namespace std;

// Defined constants
#define PORT "/dev/ttyUSB0"			// connected port
#define BAUDRATE B230400			// Baud rate = 230400 bits/s
#define USEFUL_BYTES 150			//
#define NR_OF_BYTES_RECEIVED 18		// Amount of bytes when FPGA transmits data
#define NR_OF_BYTES_SEND 10			// Amount of bytes when PC transmits data

// Functions
int open_port(void);						// Opens the correct port
int configure_port(int fd);					// Configures the port, sets baudrate, charachter size etc.
void process_block(vector<uint8_t> block);	// Processes the message of 18 bits long into real currents and positions


int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick"); // Initiate new ROS node

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(1000);

    int fd = open_port();
    configure_port(fd);
    vector<uint8_t> buffer;		// Shift register which contains serial data
    vector<uint8_t> message;	// Message of 18 bytes is stored in here when it is found in buff
    buffer.clear();				// Makes the buffer empty

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
    	bool message_found = false;
    	int bytes;
        int fd = open_port();
        configure_port(fd);
        ioctl(fd, FIONREAD, &bytes);
        //printf("Size fd: %d\n", bytes);

    	int x = 0;
    	uint8_t this_byte;
    	message.clear();

    	if (bytes>USEFUL_BYTES)
    	{
			uint8_t useles[bytes-USEFUL_BYTES];
			read(fd,&useles, bytes-USEFUL_BYTES);
			//ioctl(fd, FIONREAD, &bytes);
    	}

    	int byt_cnt = 0;
    	while ((byt_cnt <USEFUL_BYTES-1) && !message_found)
    	{

    		read(fd,&this_byte,1);
    		//printf("Size fd: %d\n", bytes);
    		buffer.push_back(this_byte);
    		//printf("Byte: %x\n",this_byte);
    		//printf("Size queue: %d :  0: %x\t 1: %x\t 18: %x\t 19: %x\t \n", buff.size(),buff[0],buff[1],buff[18],buff[19]);

    		if(buffer.size()==20)
    		{
    			if(buffer[0]==255 && buffer[1]==0 && buffer[2]<16 && buffer[18]==255 && buffer[19]==0)
    			{
					//printf("found start byte\n");

					for(int j=0;j<18;j++)
					{
						buffer.erase(buffer.begin());
						message.push_back(buffer[0]);
						printf("%x\t",message[j]);
					}
					//buff.clear();
					printf("\n");
					process_block(block);
					message_found = true;
					//break;
				}
				else
				{
					//printf("Not found\n");
					buffer.erase(buffer.begin());
				}
			}
    		byt_cnt++;
    	}


    	uint8_t send_bytes[]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
    	/* bytes send {Byte_1, Byte_2, Byte_3, Byte_4, Byte_5, Byte_6, Byte_7, Byte_8, Byte_9, Byte_10}
    	 * Byte_1: Start byte 0x00 (0b00000000)
    	 * Byte_2: Sign of Current motor 1; MSB = 0 Clamp, MSB = 1 Open
    	 * Byte_3: Amplitude of Current motor 1
    	 * Byte_4: Sign of Current motor 2: MSB = 0 Right->Left, MSB = 1 Left->Right
    	 * Byte_5: Amplitude of Current motor 2
    	 * Byte_6: Sign of Current motor 3: MSB = 0 Anti-clockwise, MSB = 1 Clockwise
    	 * Byte_7: Amplitude of Current motor 3
    	 * Byte_8: Sign of Current motor 4: MSB = 0 Down, MSB = 1 Up
    	 * Byte_9: Amplitude of Current motor 4
    	 * Byte_10: Stop byte 0xFF (0b11111111)
    	 */
    	//write(fd,send_bytes,NR_OF_BYTES_SEND);


    	close(fd);
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

        loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate

    }
    uint8_t send_bytes[]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
    write(fd,send_bytes,NR_OF_BYTES_SEND);
    return 0;
}

int open_port(void)
{

	const char *device = PORT;
	int fd;
	fd = open(device, O_RDWR);// | O_NOCTTY | O_NDELAY);
	/*
	 * device: 		The path to set the serial port
	 * fd:			The returned file handle for the device. -1 if an error occured
	 * O_RDWR:		Opens port for reading and writing
	 * O_NOCTTY:	The port never becomes the controlling terminal of the process
	 * O_DELAY:		Use non-blocking I/O. On some systems this also means the RS232 DCD signal line is ignored.
	 */

	if(fd == -1) // Port cannot be opened
		{
			printf("ERROR: Unable to open /dev/ttyUSB0. \n");
		}
		else
		{
			fcntl(fd, F_SETFL, 0);
			//printf("SUCCESFULL: /dev/ttyUSB0 port is open.\n");
		}

	return(fd);
}
int configure_port(int fd)
{
	struct termios port_settings;      // structure to store the port settings in

    tcgetattr(fd, &port_settings);

	cfsetispeed(&port_settings, B230400);    // set baud rates
	cfsetospeed(&port_settings, B230400);


	//port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits
	//port_settings.c_cflag &= ~CSTOPB;
	//port_settings.c_cc[VMIN] = 20;
	//port_settings.c_cc[VTIME] = 10;

	port_settings.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD; //| CRTSCTS


	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	// TCSANOW:	The configuration is changed immediately

	return(fd);

}

void process_block(vector<uint8_t> block)
{
	uint8_t current_1;
	uint8_t position_1;
	uint8_t current_2;
	uint8_t position_2;
	uint8_t current_3;
	uint8_t position_3;
	uint8_t current_4;
	uint8_t position_4;


}
