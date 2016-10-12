// my_ps3: a simple action server
// Wyatt Newman

#include<ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../my_ps3/action/PS3_Msg.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (PS3_Msg) and appended name (Action)
#include <my_ps3/PS3_MsgAction.h>

int g_count = 0;
bool g_count_failure = false;
std_msgs::Float64 g_vel_cmd;
double PI = 3.1415926;
std_msgs::Float64 g_A;
std_msgs::Float64 g_omega;
std_msgs::Int32 g_n;



class ps3_ActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in my_ps3/action/PS3_Msg.action
    // the type "PS3_MsgAction" is auto-generated from our name "PS3_Msg" and generic name "Action"
    actionlib::SimpleActionServer<my_ps3::PS3_MsgAction> as_;
    
    // here are some message types to communicate with our client(s)
    my_ps3::PS3_MsgGoal goal_; // goal message, received from client
    my_ps3::PS3_MsgResult result_; // put results here, to be sent back to the client when done w/ goal
 //   my_ps3::PS3_MsgFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    ps3_ActionServer(); //define the body of the constructor outside of class definition

    ~ps3_ActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<my_ps3::PS3_MsgAction>::GoalConstPtr& goal);
    void informClient();
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class ps3_ActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

ps3_ActionServer::ps3_ActionServer() :
   as_(nh_, "ps3_action", boost::bind(&ps3_ActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "ps3_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ps3_ActionServer...");
    // do any other desired initializations here...specific to your implementation
    g_A.data = goal_.A;
    g_omega.data = goal_.omega;
    g_n.data = goal_.n;
    
    as_.start(); //start the server running
    //S_INFO("1111");
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <my_ps3::PS3_MsgAction> customizes the simple action server to use our own "action" message 
// defined in our package, "my_ps3", in the subdirectory "action", called "PS3_Msg.action"
// The name "PS3_Msg" is prepended to other message types created automatically during compilation.
// e.g.,  "PS3_MsgAction" is auto-generated from (our) base name "PS3_Msg" and generic name "Action"
void ps3_ActionServer::executeCB(const actionlib::SimpleActionServer<my_ps3::PS3_MsgAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    
    //....
    g_A.data = goal->A;
    g_omega.data = goal->omega;
    g_n.data = goal->n;
    result_.A = goal->A;
    result_.omega = goal->omega;
    result_.n = goal->n;

    ros::NodeHandle n;
    ros::Publisher p = n.advertise<std_msgs::Float64>("vel_cmd", 1);
    ros::Rate naptime(1000);
    std_msgs::Float64 velocity;
    double t_n = (2.0*g_n.data*PI)/g_omega.data;
    double t = 0;
    while (ros::ok()){
        //ROS_INFO("ready to publish");
        if (t < t_n){
            velocity.data = g_A.data*sin(g_omega.data*t);
            t++;
            //ROS_INFO("t is ",t);
            //ros::spinOnce();

            p.publish(velocity);
            ROS_INFO("vel_cmd is %f", velocity.data);
            naptime.sleep();
        }
        else {
        //as_object.informClient();
        //when ncycles are over, pulish zero velocity command
        velocity.data = 0;
        p.publish(velocity);
        naptime.sleep();
        }
    }
    as_.setSucceeded(result_);


    // for illustration, populate the "result" message with two numbers:
    // the "input" is the message count, copied from goal->input (as sent by the client)
    // the "goal_stamp" is the server's count of how many goals it has serviced so far
    // if there is only one client, and if it is never restarted, then these two numbers SHOULD be identical...
    // unless some communication got dropped, indicating an error
    // send the result message back with the status of "success"

    //g_count++; // keep track of total number of goals serviced since this server was started
    //result_.output = g_count; // we'll use the member variable result_, defined in our class
    //result_.goal_stamp = goal->input;
    
    // the class owns the action server, so we can use its member methods here
   
    // DEBUG: if client and server remain in sync, all is well--else whine and complain and quit
    // NOTE: this is NOT generically useful code; server should be happy to accept new clients at any time, and
    // no client should need to know how many goals the server has serviced to date
    /*
    if (g_count != goal->input) {
        ROS_WARN("hey--mismatch!");
        ROS_INFO("g_count = %d; goal_stamp = %d", g_count, result_.goal_stamp);
        g_count_failure = true; //set a flag to commit suicide
        ROS_WARN("informing client of aborted goal");
        as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
    }
    else {
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
    }*/
}
/*
void informClient(){
    as_.setSucceeded(result_);
}
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "PS3_Msg_action_server_node"); // name this node 
    //ros::NodeHandle n;
    //ros::Publisher p = n.advertise<std_msgs::Float64>("vel_cmd", 1);
    //ros::Rate naptime(1000);


    ps3_ActionServer as_object; // create an instance of the class "ps3_ActionServer"
    
    //std_msgs::Float64 velocity;
    //double t_n = (2.0*g_n.data*PI)/g_omega.data;
    //double t = 0;
    /*
    ROS_INFO("g_A, %f", g_A.data);
    ROS_INFO("g_omega, %f", g_omega.data);
    ROS_INFO("g_n, %d", g_n.data);
    ROS_INFO("t , %f", t);
    ROS_INFO("t_n ,%f", t_n);
    */
    while (ros::ok()){
        /*
        if (t < t_n){
            velocity.data = g_A.data*sin(g_omega.data*t);
            t++;
            //ROS_INFO("t is ",t);
            //ros::spinOnce();

            p.publish(velocity);
            ROS_INFO("vel_cmd is %f", velocity.data);
            naptime.sleep();
        }
        else {
        //as_object.informClient();
        //when ncycles are over, pulish zero velocity command
        velocity.data = 0;
        p.publish(velocity);
        naptime.sleep();
        }*/
        ros::spinOnce();
    }

    return 0;
}

