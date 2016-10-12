//PS2_service.cpp
//XinyuLi  10/07/2016;
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <my_ps2/PS2_ServiceMsg.h>
#include <math.h>

std_msgs::Float64 g_A;
std_msgs::Float64 g_omega;

bool velocity_callback(my_ps2::PS2_ServiceMsgRequest& request, 
						my_ps2::PS2_ServiceMsgResponse& response){
	//Just copy the request to response and copy the request data out into global data so that we can publish it;
	response.A = request.A;
	g_A.data = request.A;
	response.omega = request.omega;
	g_omega.data = request.omega;

	return true;
}



int main(int argc, char **argv){
	ros::init(argc, argv, "ps2_service");
	ros::NodeHandle n;
	//Instaniate a velocity service;
	ros::ServiceServer service = n.advertiseService("velocityService", velocity_callback);
	ros::Publisher p = n.advertise<std_msgs::Float64>("vel_cmd", 1);
	ROS_INFO("Velocity service is ready!!!");
    
    ros::Rate naptime(1000);
	std_msgs::Float64 velocity;
	double t = 0;
	while (ros::ok()){
		velocity.data = g_A.data*sin(g_omega.data*t);
		t++;
		ros::spinOnce();

		p.publish(velocity);
		ROS_INFO("vel_cmd is %f", velocity.data);
		naptime.sleep();
	}
	
	return 0;
}