#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps5_commander"); // name of this node will be "minimal_publisher2"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/ps5_robot/joint1_position_controller/command", 1);
    ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/ps5_robot/joint2_position_controller/command", 1);
    ros::Publisher joint3_pub = n.advertise<std_msgs::Float64>("/ps5_robot/joint3_position_controller/command", 1);
    //"topic1" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
    std_msgs::Float64 j1_input; //create a variable of type "Float64", 
    std_msgs::Float64 j2_input;
    std_msgs::Float64 j3_input;
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission
   
   ros::Rate naptime(10.0); //create a ros object from the ros “Rate” class; 
   //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    j1_input.data = 0.0;
    j2_input.data = 0.0;
    j3_input.data = 0.0;
    std_msgs::Float64 A;
    std_msgs::Float64 omega;
    A.data = 0.3;
    omega.data = 0.3;
    double t = 0.0;
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        //ROS_INFO("A is %f", A.data);
        //publish1
        j1_input.data = 0.8*sin(0.1*t); //increment by 0.001 each iteration
        joint1_pub.publish(j1_input); // publish the value--of type Float64-- 
        //publish2
        j2_input.data = A.data*sin(omega.data*t)*(-1.0); //increment by 0.001 each iteration
        joint2_pub.publish(j2_input); 
        //publish3
        j3_input.data = A.data*sin(omega.data*t); //increment by 0.001 each iteration
        joint3_pub.publish(j3_input); 
        t = t + 1.0;
        ROS_INFO("t is %f", t);
        ROS_INFO("j1 data is %f", j1_input.data);
        ROS_INFO("j2 data is %f", j2_input.data);
        ROS_INFO("j3 data is %f", j3_input.data);
        //to the topic "topic1"
	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
	naptime.sleep(); 
    }
}
