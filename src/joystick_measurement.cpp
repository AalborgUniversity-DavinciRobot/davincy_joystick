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


// Functions used
int open_port(void);								// Opens the correct port
int configure_port(int port);						// Configures the port, sets baudrate, charachter size etc.
vector<double> process_message(int i,vector<uint8_t> message);	// Returns 4 Currents and 4 Positions
double hex_to_dec(uint8_t MSB, uint8_t LSB);		// Converts the hex values into decimal numbers
vector<double> initialize_position(int port, vector<uint8_t> buffer);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick"); // Initiate new ROS node

    ros::NodeHandle n("~");
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate rate(10);

    /*int i1 = 0;
    int i2 = 0;
    int i3 = 0;
    int i4 = 0;
    n.getParam("i1",i1);
    n.getParam("i2",i2);
    n.getParam("i3",i3);
    n.getParam("i4",i4);*/


    int port = open_port();
    configure_port(port);
    vector<uint8_t> buffer;
    buffer.clear();

    vector<double> init_pos;

    init_pos=initialize_position(port,buffer);

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {

        int bytes = 0;
        vector<double> measurements;
        buffer.clear();
        tcflush(port,TCIFLUSH);
        usleep(5000);	// 5 milliseconds  (18Bytes @ 1kHz -> 5*18=90 bytes per 5 ms)

        ioctl(port, FIONREAD, &bytes);	// get number of bytes available at port and store in bytes
        //printf("Size port flushed: %d\n", bytes);

    	uint8_t this_byte;

    	if (bytes>37)
    	{
			for (int i=0;i<37;i++)
			{
				read(port,&this_byte,1);
				buffer.push_back(this_byte);
			}

			for (int i=0;i<buffer.size();i++)
			{
				if(buffer[i]==255 && buffer[i+1]==0 && buffer[i+2]<16 && buffer[i+18]==255 && buffer[i+19]==0)
				{
					//printf("found start byte\n");
					measurements = process_message(i+1, buffer);
					break;
				}

			}

    	}

    	/*for (int i=0;i<measurements.size();i++)
    	{
    		printf("%4.0lf\t",measurements[i]);
    	}printf("\n");*/

    	//uint8_t send_bytes[]={0x00, 0x00, i1, 0x00, i2, 0x00, i3, 0x00, i4, 0xFF};
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
    		//tcflush(port,TCOFLUSH);	// Flush Output buffer before sending new data
    		//write(port,send_bytes,10);

        //ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate

    }
    uint8_t send_bytes[]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
    tcflush(port,TCOFLUSH);
    //write(port,send_bytes,10);
    return 0;
}

int open_port(void)
{
	/* This function opens the serial port and returns a handle to the port.
	 * The port is configured as non-blocking and open for reading and writing
	 * PORT = the name of the port, here defined as /dev/ttyUSB0
	 *
	 */
	const char *device = PORT;
	int port;
	port = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	/*
	 * device: 		The path to set the serial port
	 * fd:			The returned file handle for the device. -1 if an error occured
	 * O_RDWR:		Opens port for reading and writing
	 * O_NOCTTY:	The port never becomes the controlling terminal of the process
	 * O_NONBLOCK	Non-blocking I/O
	 * O_DELAY:		Use non-blocking I/O. On some systems this also means the RS232 DCD signal line is ignored.
	 */

	if(port == -1) // Port cannot be opened
		{
			printf("ERROR: Unable to open /dev/ttyUSB0. \n");
		}
		else
		{
			printf("SUCCESFULL: /dev/ttyUSB0 port is open.\n");
		}

	return(port);
}
int configure_port(int port)
{
	/* Functions that configures the serial port.
	 * Basic settings are applied and all other are turned of for simplicity
	 * Baudrate is set to 230400 Bits/second
	 *
	 */
	struct termios old_port_settings, new_port_settings;      // structure to store the port settings in

    tcgetattr(port, &old_port_settings);

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

	tcsetattr(port, TCSANOW, &new_port_settings);    // apply the settings to the port
	// TCSANOW:	The configuration is changed immediately

	return(port);
}

vector<double> process_message(int i,vector<uint8_t> message)
{
	/* Function calculates real currents and positions out of the message
	 * Uses hex_to_dec function to convert the hexadecimal values into decimal values
	 * which can be interpreted in a better way
	 *
	 */
	vector<double> measurements;
	measurements.push_back(hex_to_dec(message[i+1],message[i+2]));
	measurements.push_back(hex_to_dec(message[i+3],message[i+4])*360/2048);
	measurements.push_back(hex_to_dec(message[i+5],message[i+6]));
	measurements.push_back(hex_to_dec(message[i+7],message[i+8])*360/2048);
	measurements.push_back(hex_to_dec(message[i+9],message[i+10]));
	measurements.push_back(hex_to_dec(message[i+11],message[i+12])*360/2048);
	measurements.push_back(hex_to_dec(message[i+13],message[i+14]));
	measurements.push_back(hex_to_dec(message[i+15],message[i+16])*360/2048);
	return measurements;
}

double hex_to_dec(uint8_t MSB, uint8_t LSB)
{
	/* The hexadecimal values are converted to a decimal value by this function
	 * Each measurements has 2 Bytes, here called MSB and LSB which are combined before calculating decimal value
	 * Transforms hex number first to binary and then to decimal
	 */
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

vector<double> initialize_position(int port, vector<uint8_t> buffer)
{
	bool init_complete = false;
	int bytes = 0;
	vector<double> measurements;

	while (!init_complete)
	{
		buffer.clear();
		tcflush(port,TCIFLUSH);
		usleep(5000);

		ioctl(port, FIONREAD, &bytes);	// get number of bytes available at port and store in bytes

		uint8_t this_byte;

		if (bytes>37)
		{
			for (int i=0;i<37;i++)
			{
				read(port,&this_byte,1);
				buffer.push_back(this_byte);
			}

			for (int i=0;i<buffer.size();i++)
			{
				if(buffer[i]==255 && buffer[i+1]==0 && buffer[i+2]<16 && buffer[i+18]==255 && buffer[i+19]==0)
				{
					measurements = process_message(i+1, buffer);
					init_complete = true;
					printf("INITIALIZING COMPLETE\n");
				}

			}
		}
	}
	return measurements;
}
