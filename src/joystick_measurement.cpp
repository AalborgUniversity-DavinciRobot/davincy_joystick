// http://en.wikibooks.org/wiki/Serial_Programming/termios


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
#define USEFUL_BYTES 150
#define NR_OF_BYTES_RECEIVED 18		// Amount of bytes when FPGA transmits data
#define NR_OF_BYTES_SEND 10			// Amount of bytes when PC transmits data


// Functions
int open_port(void);				// Opens the correct port
int configure_port(int fd);			// Configures the port, sets baudrate, charachter size etc.
void process_block(int i,vector<uint8_t> message);
double hex_to_dec(uint8_t MSB, uint8_t LSB);



int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick"); // Initiate new ROS node

    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate rate(100);

    int fd = open_port();
    configure_port(fd);
    vector<uint8_t> buffer;
    buffer.clear();


    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
        int bytes = 0;
        buffer.clear();
        //ioctl(fd, FIONREAD, &bytes);
        tcflush(fd,TCIFLUSH);
        usleep(5000);	// 5 milliseconds  (18Bytes @ 1kHz -> 5*18=90 bytes per 5 ms)

        ioctl(fd, FIONREAD, &bytes);
        //printf("Size fd fluched: %d\n", bytes);

    	uint8_t this_byte;

    	if (bytes>37){
			for (int i=0;i<37;i++)
			{
				read(fd,&this_byte,1);
				buffer.push_back(this_byte);
			}

			for (int i=0;i<buffer.size();i++)
			{
				if(buffer[i]==255 && buffer[i+1]==0 && buffer[i+2]<16 && buffer[i+18]==255 && buffer[i+19]==0)
				{
					printf("found start byte\n");
					process_block(i+1, buffer);
				}

			}

    	}

    	uint8_t send_bytes[]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 00, 0xFF};
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
    	tcflush(fd,TCOFLUSH);	// Flush Output buffer before sending new data
    	//write(fd,send_bytes,NR_OF_BYTES_SEND);

        //ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate

    }
    uint8_t send_bytes[]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
    tcflush(fd,TCOFLUSH);
    write(fd,send_bytes,NR_OF_BYTES_SEND);
    return 0;
}

int open_port(void)
{
	const char *device = PORT;
	int fd;
	fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	/*
	 * device: 		The path to set the serial port
	 * fd:			The returned file handle for the device. -1 if an error occured
	 * O_RDWR:		Opens port for reading and writing
	 * O_NOCTTY:	The port never becomes the controlling terminal of the process
	 * O_NONBLOCK	Non-blocking I/O
	 * O_DELAY:		Use non-blocking I/O. On some systems this also means the RS232 DCD signal line is ignored.
	 */

	if(fd == -1) // Port cannot be opened
		{
			printf("ERROR: Unable to open /dev/ttyUSB0. \n");
		}
		else
		{
			printf("SUCCESFULL: /dev/ttyUSB0 port is open.\n");
		}

	return(fd);
}
int configure_port(int fd)
{
	struct termios old_port_settings, new_port_settings;      // structure to store the port settings in

    tcgetattr(fd, &old_port_settings);

    // port settings
    cfmakeraw(&new_port_settings);
    new_port_settings.c_cflag &= ~CSIZE;
    new_port_settings.c_cflag = BAUDRATE | CS8;
    new_port_settings.c_cflag |= (CLOCAL | CREAD);
    new_port_settings.c_cflag &= ~(PARENB | PARODD); // No parity
    new_port_settings.c_cflag &= ~CRTSCTS; // No hardware handshake
    new_port_settings.c_cflag &= ~CSTOPB; // 1 stopbit
    new_port_settings.c_iflag = IGNBRK;
    new_port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    new_port_settings.c_lflag = 0;
    new_port_settings.c_oflag = 0;
    //new_port_settings.c_cc[VTIME] = 1;
    //new_port_settings.c_cc[VMIN] = 60;

	tcsetattr(fd, TCSANOW, &new_port_settings);    // apply the settings to the port
	// TCSANOW:	The configuration is changed immediately

	return(fd);
}

void process_block(int i,vector<uint8_t> message)
{
	double current1 = hex_to_dec(message[i+1],message[i+2]);
	double position1 = hex_to_dec(message[i+3],message[i+4]);
	double current2 = hex_to_dec(message[i+5],message[i+6]);
	double position2 = hex_to_dec(message[i+7],message[i+8]);
	double current3 = hex_to_dec(message[i+9],message[i+10]);
	double position3 = hex_to_dec(message[i+11],message[i+12]);
	double current4 = hex_to_dec(message[i+13],message[i+14]);
	double position4 = hex_to_dec(message[i+15],message[i+16]);

	printf("Current1:%4.0lf\t\tCurrent2:%4.0lf\t\tCurrent3:%4.0lf\t\tCurrent4:%4.0lf\n", current1,current2,current3,current4);
	printf("Pos1:%4.0lf\t\tPos2:%4.0lf\t\tPos3:%4.0lf\t\tPos4:%4.0lf\n", position1,position2,position3,position4);
}

double hex_to_dec(uint8_t MSB, uint8_t LSB)
{
	uint8_t tmp[2] = {MSB,LSB};
	uint8_t hex;
	vector<bool> binary;
	double dec = 0.0;

	binary.clear();
	for (int j=0;j<2;j++)
	{
		hex = tmp[j];
		bool bin[8];
		for(int i=0;i<8;i++)
		{
			bin[i]=hex%2;
			hex = hex/2;
		}
		int top, bottom;
		for(bottom=0,top =7; bottom<8; bottom++,top--)
		{
			binary.push_back(bin[top]);
		}
	}

	for (int i=0;i<binary.size();i++)
	{
		dec += (double)binary[i]*pow(double(2),double(16-i-1));
		}
	return dec;
}
