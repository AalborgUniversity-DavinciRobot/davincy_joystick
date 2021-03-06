
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Float64MultiArray.h"
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <sys/ioctl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Namespaces
using namespace std;

// Define constants
#define PORT "/dev/ttyUSB0"			// connected port
#define BAUDRATE B230400			// Baud rate = 230400 bits/s
#define FREQ 100					// Frequency at which node is running (160 Hz max, because we need to wait till buffer has enough bytes available (6ms)
#define M_PI 3.14159265358979323846	// PI
#define GEAR_RATIO_1 5.1*2			// Gear ratio motor 1 only 256 steps, therefore *2
#define GEAR_RATIO_2 19.0			// Gear ratio motor 2
#define GEAR_RATIO_3 4.4			// Gear ratio motor 3
#define GEAR_RATIO_4 19.0			// Gear ratio motor 4

#define IMAX_1 0.6
#define IMAX_2 0.5
#define IMAX_3 0.5
#define IMAX_4 0.5
#define BIT_PER_AMP 142.0


#define BIT_TO_CURRENT_MEASUREMENT 2.0/4095.0	// ratio to convert measured message into current [A] (12 bits, 2Amps max)
#define BIT_TO_DEGREE_MEASUREMENT 360/2047.0	// ratio to convert measured message into position [degree] (11bits, 360degrees)
#define DEGREE_TO_RAD M_PI/180					// ratio to convert degrees to radians

class Port
{
public:
	int port_handle;
	void open_port(void);

private:
	void configure_port(void);

};
void Port::open_port(void)
{
	/* This function opens the serial port and returns a handle to the port.
	 * The port is configured as non-blocking and open for reading and writing
	 * PORT = the name of the port, here defined as /dev/ttyUSB0
	 *
	 */
	const char *device = PORT;
	port_handle = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	/*
	 * device: 		The path to set the serial port
	 * fd:			The returned file handle for the device. -1 if an error occured
	 * O_RDWR:		Opens port for reading and writing
	 * O_NOCTTY:	The port never becomes the controlling terminal of the process
	 * O_NONBLOCK	Non-blocking I/O
	 * O_DELAY:		Use non-blocking I/O. On some systems this also means the RS232 DCD signal line is ignored.
	 */

	if(port_handle == -1) // Port cannot be opened
		{
			ROS_INFO("ERROR: Unable to open /dev/ttyUSB0. \n");
		}
		else
		{
			ROS_INFO("SUCCESFULL: /dev/ttyUSB0 port is open.\n");
			configure_port();
		}
}
void Port::configure_port(void)
{
	/* Functions that configures the serial port.
	 * Basic settings are applied and all other are turned of for simplicity
	 * Baudrate is set to 230400 Bits/second
	 *
	 */
	struct termios old_port_settings, new_port_settings;      // structure to store the port settings in

	tcgetattr(port_handle, &old_port_settings);

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

	// apply the settings to the port
	tcsetattr(port_handle, TCSANOW, &new_port_settings);
	// TCSANOW:	The configuration is changed immediately

}

class Message
{
public:
	int port;
	bool msg_found;
	vector<double> message;
	vector<double> current;
	vector<double> position;

	void get_message(void);
	void print_message(vector<double> vec);
	void send_message(vector<double> I_setpoint);

private:
	bool check_buffer(void);
	vector<double> process_message(int i,vector<uint8_t> block);
	double hex_to_dec(uint8_t MSB, uint8_t LSB);

};
bool Message::check_buffer(void)
{
	int bytes;
	tcflush(port,TCIFLUSH);
	usleep(6000);	// 6 milliseconds  (18Bytes @ 1ms -> 6*18=108 bytes per 6 ms)
	ioctl(port, FIONREAD, &bytes);	// get number of bytes available at port and store in bytes
	//ROS_INFO(" %d \n",bytes);
	if (bytes > 36)
	{
		return true;
	}
	else
	{
		ROS_INFO("NOT ENOUGH BYTES AVAILABLE AT PORT (%d)\n",bytes);
		return false;
	}
}
void Message::get_message(void)
{
	msg_found = false;
	vector<uint8_t> buffer;
	buffer.clear();
	uint8_t byte;
	if (check_buffer())
	{
		for (int i=0;i<37;i++)
		{
			read(port,&byte,1);
			buffer.push_back(byte);
		}
		for (int i=0;i<buffer.size();i++)
		{
			if(buffer[i]==255 && buffer[i+1]==0 && buffer[i+2]<16 && buffer[i+18]==255 && buffer[i+19]==0)
			{
				message = process_message(i+1,buffer);
				msg_found = true;
				break;
			}

		}
	}
	else
	{
		//message.clear();
	}
}
vector<double> Message::process_message(int i,vector<uint8_t> msg_raw)
{
	vector<double> msg_;
	msg_.push_back(hex_to_dec(msg_raw[i+1],msg_raw[i+2])*BIT_TO_CURRENT_MEASUREMENT);
	msg_.push_back(hex_to_dec(msg_raw[i+3],msg_raw[i+4])*BIT_TO_DEGREE_MEASUREMENT);
	msg_.push_back(hex_to_dec(msg_raw[i+5],msg_raw[i+6])*BIT_TO_CURRENT_MEASUREMENT);
	msg_.push_back(hex_to_dec(msg_raw[i+7],msg_raw[i+8])*BIT_TO_DEGREE_MEASUREMENT);
	msg_.push_back(hex_to_dec(msg_raw[i+9],msg_raw[i+10])*BIT_TO_CURRENT_MEASUREMENT);
	msg_.push_back(hex_to_dec(msg_raw[i+11],msg_raw[i+12])*BIT_TO_DEGREE_MEASUREMENT);
	msg_.push_back(hex_to_dec(msg_raw[i+13],msg_raw[i+14])*BIT_TO_CURRENT_MEASUREMENT);
	msg_.push_back(hex_to_dec(msg_raw[i+15],msg_raw[i+16])*BIT_TO_DEGREE_MEASUREMENT);

	position.clear();
	current.clear();
	current.push_back(msg_[0]);
	current.push_back(msg_[2]);
	current.push_back(msg_[4]);
	current.push_back(msg_[6]);
	position.push_back(msg_[1]);
	position.push_back(msg_[3]);
	position.push_back(msg_[5]);
	position.push_back(msg_[7]);

	return msg_;
}
double Message::hex_to_dec(uint8_t MSB, uint8_t LSB)
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
void Message::print_message(vector<double> vec)
{
	for (int i=0;i<vec.size();i++)
	{
		printf("%lf\t",vec[i]);
	}
	printf("\n");
}
void Message::send_message(vector<double> I_sp)
{
	/* Sends current setpoint to joystick
	 * Input:	port	->	port handle
	 * 			S_i		->	direction of current (0,1)
	 * 			I_i		->	Amplitude of current (0,IMAX_i)
	 */

	
	int S1,S2,S3,S4;
	if (I_sp[0]<0.0) {S1=0; I_sp[0]*=-1;}
	else {S1=1;}
	if (I_sp[1]<0.0) {S2=1; I_sp[1]*=-1;}
	else {S2=0;}
	if (I_sp[2]<0.0) {S3=1; I_sp[2]*=-1;}
	else {S3=0;}
	if (I_sp[3]<0.0) {S4=0; I_sp[3]*=-1;}
	else {S4=1;}
	

	uint8_t message[]={0x00, S1*255, I_sp[0]*BIT_PER_AMP*2, S2*255, I_sp[1]*BIT_PER_AMP, S3*255, I_sp[2]*BIT_PER_AMP, S4*255, I_sp[3]*BIT_PER_AMP, 0xFF};

	//uint8_t message[]={0x00, S1*255, 0, S2*255, 0, S3*255, 0, S4*255, 0, 0xFF};
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

	// Flush Output buffer before sending new data so buffer will be empty
	tcflush(port,TCOFLUSH);
	write(port,message,10);
}

class Joystick
{
public:
	double Theta1, Theta2, Theta3, Theta4;
	double dTheta1, dTheta2, dTheta3, dTheta4;

	double I1, I2, I3, I4;
	vector<double> Position;
	vector<double> Velocity;
	vector<double> I_setpoint;
	sensor_msgs::JointState joint_states;

	void update_position(vector<double> Theta_old, vector<double> Theta_new, double dT);
	void update_current(vector<double> I);
	void current_setpoint(double i1,double i2, double i3, double i4);
	void JoystickCallback(std_msgs::Float64MultiArray Isp);

	void create_joint_state_msg(void);
	void callibration(Message msg);


private:
	void limit_current(void);
	double delta_position(double Theta_old, double Theta_new);

};

void Joystick::JoystickCallback(std_msgs::Float64MultiArray Isp)
{
	
	if (Isp.data.size() == 4)
	{
		current_setpoint(Isp.data[0],Isp.data[1],Isp.data[2],Isp.data[3]);
	}
	else
	{
		current_setpoint(0.0,0.0,0.0,0.0);
	}
	
}

void Joystick::limit_current(void)
{
	if (I_setpoint[0] >IMAX_1)
	{ I_setpoint[0] = IMAX_1; }
	if (I_setpoint[0] <-IMAX_1)
	{ I_setpoint[0] = -IMAX_1; }

	if (I_setpoint[1] >IMAX_2)
	{ I_setpoint[1] = IMAX_2; }
	if (I_setpoint[1] <-IMAX_2)
	{ I_setpoint[1] = -IMAX_2; }

	if (I_setpoint[2] >IMAX_3)
	{ I_setpoint[2] = IMAX_3; }
	if (I_setpoint[2] <-IMAX_3)
	{ I_setpoint[2] = -IMAX_3; }

	if (I_setpoint[3] >IMAX_4)
	{ I_setpoint[3] = IMAX_4; }
	if (I_setpoint[3] <-IMAX_4)
	{ I_setpoint[3] = -IMAX_4; }
}
void Joystick::create_joint_state_msg(void)
{
	joint_states.header.stamp=ros::Time::now();
	joint_states.position.resize(5);
	joint_states.name.resize(5);
	joint_states.effort.resize(5);
	joint_states.velocity.resize(5);
	joint_states.name[0]="pinch";
	joint_states.position[0]=-Theta1/GEAR_RATIO_1*DEGREE_TO_RAD;
	joint_states.velocity[0]=Velocity[0]/GEAR_RATIO_1*DEGREE_TO_RAD;
	joint_states.effort[0]=I1;
	joint_states.name[1]="yaw";
	joint_states.position[1]=-Theta2/GEAR_RATIO_2*DEGREE_TO_RAD;
	joint_states.velocity[1]=Velocity[1]/GEAR_RATIO_2*DEGREE_TO_RAD;
	joint_states.effort[1]=I2;
	joint_states.name[2]="roll";
	joint_states.position[2]=-Theta3/GEAR_RATIO_3*DEGREE_TO_RAD;
	joint_states.velocity[2]=Velocity[2]/GEAR_RATIO_3*DEGREE_TO_RAD;
	joint_states.effort[2]=I3;
	joint_states.name[3]="pitch";
	joint_states.position[3]=-Theta4/GEAR_RATIO_4*DEGREE_TO_RAD;
	joint_states.velocity[3]=Velocity[3]/GEAR_RATIO_4*DEGREE_TO_RAD;
	joint_states.effort[3]=I4;
	joint_states.name[4]="pinch2";
	joint_states.position[4]=Theta1/GEAR_RATIO_1*DEGREE_TO_RAD;
	joint_states.velocity[4]=-Velocity[0]/GEAR_RATIO_1*DEGREE_TO_RAD;
	joint_states.effort[4]=I1;

}
double Joystick::delta_position(double Theta_old, double Theta_new)
{
	double range=60.0;
	double dTheta;
	if (Theta_old>(360.0-range) && Theta_new<range)		// Position has overflown
		{
			dTheta = 360.0-Theta_old+Theta_new;
		}
	else if (Theta_old<range && Theta_new>(360.0-range)) // Position has underflown
	{
		dTheta = -Theta_old-(360-Theta_new);
	}
	else
	{
		dTheta = Theta_new-Theta_old;			// Normal case
	}
	return dTheta;
}
void Joystick::update_position(vector<double> Theta_old, vector<double> Theta_new, double dT)
{
	dTheta1 = delta_position(Theta_old[0],Theta_new[0]);
	dTheta2 = delta_position(Theta_old[1],Theta_new[1]);
	dTheta3 = delta_position(Theta_old[2],Theta_new[2]);
	dTheta4 = delta_position(Theta_old[3],Theta_new[3]);

	Theta1 +=dTheta1;
	Theta2 +=dTheta2;
	Theta3 +=dTheta3;
	Theta4 +=dTheta4;
	Position.clear();
	Position.push_back(Theta1);
	Position.push_back(Theta2);
	Position.push_back(Theta3);
	Position.push_back(Theta4);
	Position.resize(4);

	Velocity.clear();
	Velocity.push_back(dTheta1/dT);
	Velocity.push_back(dTheta2/dT);
	Velocity.push_back(dTheta3/dT);
	Velocity.push_back(dTheta4/dT);	
	Velocity.resize(4);

}
void Joystick::current_setpoint(double i1,double i2, double i3, double i4)
{
	I_setpoint.clear();

	I_setpoint.push_back(i1);
	I_setpoint.push_back(i2);
	I_setpoint.push_back(i3);
	I_setpoint.push_back(i4);
	limit_current();

}
void Joystick::update_current(vector<double> I)
{
	I1=I[0];
	I2=I[1];
	I3=I[2];
	I4=I[3];
}


int main(int argc, char **argv)
{
	Joystick davinci_joystick;

	// ROS initialization
    ros::init(argc, argv, "joystick_get_state");
    ros::NodeHandle n;
    ros::Publisher joystick_pub = n.advertise<sensor_msgs::JointState>("davinci_joystick/joint_states",1);
    ros::Publisher joystick_pub_sim = n.advertise<sensor_msgs::JointState>("joint_states",1);
    ros::Subscriber Isetpoint_sub = n.subscribe("davinci_joystick/I_sp",1,&Joystick::JoystickCallback, &davinci_joystick);
    ros::Rate rate(FREQ);

    Port serial_port;
    serial_port.open_port();
    if (serial_port.port_handle == -1)
    {  	return 0;   }

    Message msg;
    Message msg_old;
    Message msg_init;

    msg.port=serial_port.port_handle;
    msg_old.port=serial_port.port_handle;
    msg_init.port=serial_port.port_handle;

    

    while (!msg_init.msg_found)
    {
    	msg_init.get_message();
    	ROS_INFO("GETTING INITIAL MESSAGE\n");
    }
    msg_init.print_message(msg_init.position);
    msg_init.print_message(msg_init.current);
    msg_old = msg_init;

    sleep(2);

    double last_time = ros::Time::now().toSec();
    double current_time = ros::Time::now().toSec();
    double delta_time = current_time - last_time;
    double start_time = current_time;
    davinci_joystick.current_setpoint(0,0,0,0);

    int lost = 0;
    int loop_count = 0;
    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
	loop_count++;
    	current_time = ros::Time::now().toSec();
    	

    	msg.send_message(davinci_joystick.I_setpoint);
    	msg.get_message();
    	if (msg.msg_found)
    	{
		delta_time = current_time - last_time;
    		davinci_joystick.update_position(msg_old.position,msg.position,delta_time);
    		davinci_joystick.update_current(msg.current);
    		davinci_joystick.create_joint_state_msg();
    		msg_old = msg;
    	}
	else
	{
		lost++; //amount of lost messages
	}
	/*if (loop_count % 1000 == 0)
	{
		printf("time: %lf msg: %d, lost: %d, perc: %lf \n",current_time-start_time,loop_count,lost,double(lost)/double(loop_count));	
	}*/

	joystick_pub.publish(davinci_joystick.joint_states);
	joystick_pub_sim.publish(davinci_joystick.joint_states);

    	ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
        last_time = current_time;
    }
    davinci_joystick.current_setpoint(0,0,0,0);
    msg.send_message(davinci_joystick.I_setpoint);
    return 0;
}

