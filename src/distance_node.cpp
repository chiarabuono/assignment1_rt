#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include <string.h>
#include "std_msgs/Float32.h"

float turtle1_pos[3];
float turtle2_pos[3];

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher distance_pub;

void stop_turtle1() {
    
    geometry_msgs::Twist turtleVel;
    turtleVel.linear.x = 0;
    turtleVel.linear.y = 0;
    turtleVel.angular.z = 0;
	
	pub1.publish(turtleVel);
}

void stop_turtle2() {
    
    geometry_msgs::Twist turtleVel;
    turtleVel.linear.x = 0;
    turtleVel.linear.y = 0;
    turtleVel.angular.z = 0;
	
	pub2.publish(turtleVel);
}

void turtle1_pose(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pos[0] = msg -> x;
    turtle1_pos[1] = msg -> y;
    turtle1_pos[2] = msg -> theta;

    /*stops the moving turtle if the position 
    is too close to the boundaries 
    (.e.g, x or y > 10.0, x or y < 1.0) */

    if (turtle1_pos[0] >= 10.0 || turtle1_pos[0] <= 1.0 || turtle1_pos[1] >= 10.0 || turtle1_pos[1] <= 1.0) {
        stop_turtle1();
        
    }
}

void turtle2_pose(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pos[0] = msg -> x;
    turtle2_pos[1] = msg -> y;
    turtle2_pos[2] = msg -> theta;

    if (turtle2_pos[0] >= 10.0 || turtle2_pos[0] < 1.0 || turtle2_pos[1] >= 10.0 || turtle2_pos[1] < 1.0) {
        stop_turtle2();
        
    }
}

void distance(float* pos1, float* pos2) {

    //checks the relative distance between turtle1 and turtle2
    float distance = std::sqrt(std::pow(pos1[0] - pos2[0], 2) + std::pow(pos1[1] - pos2[1], 2));
    
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    distance_pub.publish(distance_msg);
}

int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "distance_node");  
	ros::NodeHandle nh;

    ros::Subscriber sub_turtle1 = nh.subscribe("turtle1/pose", 1, turtle1_pose);
    ros::Subscriber sub_turtle2 = nh.subscribe("turtle2/pose", 1, turtle2_pose);

    pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);
    
    while (ros::ok) {
        

        /* publish on a topic the distance 
        (you can use a std_msgs/Float32 for that)*/
        distance_pub = nh.advertise<std_msgs::Float32>("turtles_distance", 1);
        distance(turtle1_pos, turtle2_pos);

        /*stops the moving turtle if the two turtles are “too close” 
        (you may set a threshold to monitor that)*/



        sleep(0.1);
        ros::spinOnce();
    }



    return 0;
}