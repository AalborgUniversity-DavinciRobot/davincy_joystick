#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>

class Joystick
{

public:
	sensor_msgs::JointState joint_state;
	double current;
	void jointCallback(sensor_msgs::JointState joint);
};

void Joystick::jointCallback(sensor_msgs::JointState joint)
{
	int s = joint.name.size();
	joint_state.name.resize(s);
	joint_state.position.resize(s);
	joint_state.velocity.resize(s);
	joint_state.effort.resize(s);
	joint_state = joint;
	//ROS_INFO(" %lf \n",joint.position[2]);

	current = joint.position[2];
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

		printf(" %lf \n",joystick.current );

		ros::spinOnce();
		rate.sleep();
	}

}
