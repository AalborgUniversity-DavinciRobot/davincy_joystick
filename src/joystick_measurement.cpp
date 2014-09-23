
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
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
#define GEAR_RATIO_1 5.1*2			// Gear ratio motor 1 only 256 steps, therefore *2
#define IMAX_1 1.0
#define GEAR_RATIO_2 19.0			// Gear ratio motor 2
#define IMAX_2 1.6
#define GEAR_RATIO_3 4.4			// Gear ratio motor 3
#define GEAR_RATIO_4 19.0			// Gear ratio motor 4
#define IMAX_4 1.7


// Functions used
int open_port(void);								// Opens the correct port
int configure_port(int port);						// Configures the port, sets baudrate, charachter size etc.
vector<double> process_message(int i,vector<uint8_t> message);	// Returns 4 Currents and 4 Positions
double hex_to_dec(uint8_t MSB, uint8_t LSB);		// Converts the hex values into decimal numbers
vector<double> initialize_position(int port, vector<uint8_t> buffer);
double delta_position(double Theta_old,double Theta_new);
void send_voltage(int port, int v1, int v2, int v3, int v4, int s1, int s2, int s3, int s4);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_publisher"); // Initiate new ROS node

    ros::NodeHandle n;
    ros::Publisher joystick_pub = n.advertise<sensor_msgs::JointState>("davinci_joystick/joint_states",1);
    ros::Rate rate(100);

    sensor_msgs::JointState joystick;
    joystick.name.resize(4);
    joystick.position.resize(4);
    joystick.velocity.resize(4);
    joystick.effort.resize(4);
    joystick.name[0] = "Motor_1";
    joystick.name[1] = "Motor_2";
    joystick.name[2] = "Motor_3";
    joystick.name[3] = "Motor_4";

    int port = open_port();
    configure_port(port);
    vector<uint8_t> buffer;
    buffer.clear();

    double I_to_bit =255.0/3.0;
    vector<double> data_init;
    data_init=initialize_position(port,buffer);
    sleep(3);
    vector<double> data_old = data_init;
    vector<double> data;
    double Theta1 = data_init[1];
    double Theta2 = data_init[3];
    double Theta3 = data_init[5];
    double Theta4 = data_init[7];
    double dTheta1 = 0.0;
    double dTheta2 = 0.0;
    double dTheta3 = 0.0;
    double dTheta4 = 0.0;

    double i1 = 0.0;
    int s1 = 0.0;
    double i2 = 0;
    int s2 = 0;
    double i3 = 0;
    int s3 = 0;
    double i4 = 0;
    int s4 = 0;

    int cnt = 0.0;

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {

        int bytes = 0;
        uint8_t this_byte;
        buffer.clear();
        tcflush(port,TCIFLUSH);
        usleep(7000);	// 5 milliseconds  (18Bytes @ 1kHz -> 5*18=90 bytes per 5 ms)

        ioctl(port, FIONREAD, &bytes);	// get number of bytes available at port and store in bytes

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
					data = process_message(i+1, buffer);
					dTheta1 = delta_position(data_old[1],data[1]);
					dTheta2 = delta_position(data_old[3],data[3]);
					dTheta3 = delta_position(data_old[5],data[5]);
					dTheta4 = delta_position(data_old[7],data[7]);
					Theta1 += dTheta1;
					Theta2 += dTheta2;
					Theta3 += dTheta3;
					Theta4 += dTheta4;
					break;
				}
			}
    	}

    	double I1 = data[0];
    	double I2 = data[2];
    	double I3 = data[4];
    	double I4 = data[6];
    	//if (dTheta1<0.0) { I1 *= -1;}
    	//if (dTheta2<0.0) { I2 *= -1;}
    	//if (dTheta3<0.0) { I3 *= -1;}
    	//if (dTheta4<0.0) { I4 *= -1;}
    	double total_I = I1+I2+I3+I4;

    	data_old = data;

    	/*if (cnt==100)
    	{
    		i1+=2.0/10;
    		cnt = 0.0;
    	}
    	if (i1>2.1)
    	{
    		i1 = 0.0;
    	}*/

    	i4 = cnt/1000.0*IMAX_4;
    	if (cnt==1000)
    	{
    		cnt=0;
    	}


    	printf("%d\t%lf\t%lf\n",cnt,I4,i4);
    	send_voltage(port,s1*255,i1*255.0/IMAX_1,s2*255,i2*255.0/IMAX_2,0,0,s4*255,i4*255.0/IMAX_4);


    	cnt++;
    	joystick.header.stamp = ros::Time::now();
    	joystick.position[0]= (Theta1-data_init[1])/GEAR_RATIO_1;
    	joystick.position[1]= (Theta2-data_init[3])/GEAR_RATIO_2;
    	joystick.position[2]= (Theta3-data_init[5])/GEAR_RATIO_3;
    	joystick.position[3]= (Theta4-data_init[7])/GEAR_RATIO_4;
    	joystick.velocity[0]=i1;
    	joystick.velocity[1]=i2;
    	joystick.velocity[2]=i3;
    	joystick.velocity[3]=i4;
    	joystick.effort[0]=I1;
    	joystick.effort[1]=I2;
    	joystick.effort[2]=I3;
    	joystick.effort[3]=I4;
    	joystick_pub.publish(joystick);

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate

    }
    send_voltage(port,0,0,0,0,0,0,0,0);
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
			ROS_INFO("ERROR: Unable to open /dev/ttyUSB0. \n");
		}
		else
		{
			ROS_INFO("SUCCESFULL: /dev/ttyUSB0 port is open.\n");
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
	double bit_to_I = 2.0/4095.0; //Bit to Current mapping
	double bit_to_degree = 360/2047.0;

	vector<double> measurements;
	measurements.push_back(hex_to_dec(message[i+1],message[i+2])*bit_to_I);
	measurements.push_back(hex_to_dec(message[i+3],message[i+4])*bit_to_degree);
	measurements.push_back(hex_to_dec(message[i+5],message[i+6])*bit_to_I);
	measurements.push_back(hex_to_dec(message[i+7],message[i+8])*bit_to_degree);
	measurements.push_back(hex_to_dec(message[i+9],message[i+10])*bit_to_I);
	measurements.push_back(hex_to_dec(message[i+11],message[i+12])*bit_to_degree);
	measurements.push_back(hex_to_dec(message[i+13],message[i+14])*bit_to_I);
	measurements.push_back(hex_to_dec(message[i+15],message[i+16])*bit_to_degree);
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
	bool init_m1 = false;
	bool init_m2 = false;
	bool init_m4 = false;
	double I_to_bit =255.0/2.0;
	int range = 2.0;
    double i1 = 0;
    int s1 = 0;
    double i2 = 0;
    int s2 = 0;
    double i3 = 0;
    int s3 = 0;
    double i4 = 0;
    int s4 = 0;
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

					if(measurements[1]<range || 360.0-measurements[1]<range)
					{ 	init_m1 = true; i1 = 0;}
					else if (measurements[1]<180)
					{
						i1 = 0.5; s1 = 0; init_m1=false;
					}
					else
					{	i1 = 0.5; s1 = 1; init_m1=false;}

					if(measurements[7]<range || 360.0-measurements[7]<range)
					{	init_m4 = true;	i4=0;}
					else if (measurements[7]<180.0)
					{	i4 =measurements[7]/100.0+0.5; s4 = 0;	init_m4 = false;}
					else
					{	i4 =measurements[7]/100.0+0.5;
						s4 = 1;
						init_m4 = false;
					}

					if(measurements[3]<range || 360.0-measurements[3]<range)
					{
						init_m2 = true;
						i2=0;
					}
					else if (measurements[3]<180.0)
					{
						i2=measurements[3]/100.0+0.1;
						s2 = 0;
						init_m2 = false;}
					else
					{
						i2=(360-measurements[3])/100.0+0.1;
						s2=1;
						init_m2 = false;}

					send_voltage(port,s1*255,I_to_bit*i1,s2*255,I_to_bit*i2,0,0,s4*255,I_to_bit*i4);
					init_complete =init_m1*init_m2*init_m4;
					//printf("POSITIONS:\t%3.2lf\t%2.2lf\t%3.2lf\n",measurements[1],measurements[3],measurements[7]);
				}
			}
		}
	}
	ROS_INFO("INITIALIZING COMPLETE\nINIT POSITIONS:\t%3.2lf\t%2.2lf\t%3.2lf\n",measurements[1],measurements[3],measurements[7]);

	return measurements;
}

double delta_position(double Theta_old,double Theta_new)
{
	double dTheta;

		if (Theta_old>290.0 && Theta_new<70.0)
		{
			dTheta = 360.0-Theta_old+Theta_new;
		}
		else if (Theta_old<70.0 && Theta_new>290.0)
		{
			dTheta = -Theta_old-(360-Theta_new);
		}
		else
		{
			dTheta = Theta_new-Theta_old;
		}
	return dTheta;
}

void send_voltage(int port, int s1, int v1, int s2, int v2, int s3, int v3, int s4, int v4)
{
	uint8_t send_bytes[]={0x00, s1, v1, s2, v2, s3, v3, s4, v4, 0xFF};
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
		tcflush(port,TCOFLUSH);	// Flush Output buffer before sending new data
		write(port,send_bytes,10);
}

