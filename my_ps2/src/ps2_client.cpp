//PS2_client
//XinyuLi, 10/07/2016;

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <my_ps2/PS2_ServiceMsg.h>
using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "ps2_client");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<my_ps2::PS2_ServiceMsg>("velocityService");
	my_ps2::PS2_ServiceMsg srv;
	double A;
	double omega;
	while (ros::ok()){
		cout <<"Please enter A"<<endl;
		cin >> A;
		cout <<"Please enter omega"<<endl;
		cin >>omega;
		//transmit to srv
		srv.request.A = A;
		srv.request.omega = omega;
		if (client.call(srv)){
			cout <<"Response A is %f"<<srv.response.A;
			cout <<"Response omega is %f"<<srv.response.omega;

		}
	}
}
