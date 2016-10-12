#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<math.h>
//Variable defination
std_msgs::Float64 g_vel_cmd;
double PI = 3.1415926;
double A = 1.0;
double omega = 0.01;

std_msgs::Float64 vel_cmd;
int main(int argc, char **argv){
	ros::init(argc, argv, "ps1_vel_commander");
	ros::NodeHandle n;
 	ros::Publisher vel_cmd_pulisher = n.advertise<std_msgs::Float64>("vel_cmd", 1);

 	ros::Rate naptime(1000);//this is the pulish frenquency;
 	double t = 0.0;//t is the time for vel_commander;
 	while (ros::ok()){
 		g_vel_cmd.data = A * sin(omega*t);
 		t++;
 		vel_cmd_pulisher.publish(g_vel_cmd);
 		ROS_INFO("velocity command is %f", g_vel_cmd.data);
        naptime.sleep();//wait for a while for the publisher to pulish the command at given frequency;
 	}
}