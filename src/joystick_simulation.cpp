#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class Joystick
{

public:
	sensor_msgs::JointState joint_state;
	void jointCallback(sensor_msgs::JointState joint);
};

void Joystick::jointCallback(sensor_msgs::JointState joint)
{
	joint_state = joint;
}



int main(int argc, char **argv)
{

	Joystick joystick;

	ros::init(argc, argv, "simulation");
	ros::NodeHandle n;
	ros::Subscriber joint_sub = n.subscribe("davinci_joystick/joint_states", 1, &Joystick::jointCallback, &joystick);
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Rate rate(100);

	while (ros::ok())
	{

		joint_pub.publish(joystick.joint_state);

		ros::spinOnce();
		rate.sleep();
	}

}
