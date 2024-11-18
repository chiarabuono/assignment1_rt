#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include <string.h>
#include "std_msgs/Float32.h"

float turtle1_pos[3];
float turtle2_pos[3];
float turtle1_prev[3];
float turtle2_prev[3];
float threshold = 2;

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher distance_pub;

void stop_turtle(int i) {
    geometry_msgs::Twist turtleVel;
    turtleVel.linear.x = 0;
    turtleVel.linear.y = 0;
    turtleVel.angular.z = 0;
	
	if (i == 1) pub1.publish(turtleVel);
    else pub2.publish(turtleVel);
}

void turtle_pose(const turtlesim::Pose::ConstPtr& msg, int i) {
    float* turtle_prev;
    float* turtle_pos;
    
    // Set the pointers based on the turtle index
    if (i == 1) {
        turtle_prev = turtle1_prev;
        turtle_pos = turtle1_pos;
    } else {
        turtle_prev = turtle2_prev;
        turtle_pos = turtle2_pos;
    }

    // Copy current position to previous
    std::memcpy(turtle_prev, turtle_pos, sizeof(float) * 3);
  
    turtle_pos[0] = msg -> x;
    turtle_pos[1] = msg -> y;
    turtle_pos[2] = msg -> theta;


    /*stops the moving turtle if the position 
    is too close to the boundaries 
    (.e.g, x or y > 10.0, x or y < 1.0) */
    bool boundary_conditions = turtle_pos[0] >= 10.0 || turtle_pos[0] < 1.0 || turtle_pos[1] >= 10.0 || turtle_pos[1] < 1.0;
    bool going_away_from_boundary = (turtle_prev[0] > turtle_pos[0] && turtle_prev[0] >= 10.0) ||        // right boundary
                                    (turtle_prev[0] < turtle_pos[0] && turtle_prev[0] < 1.0) ||          // left boundary
                                    (turtle_prev[1] > turtle_pos[1] && turtle_prev[1] >= 10.0) ||        // down boundary
                                    (turtle_prev[1] < turtle_pos[1] && turtle_prev[1] < 1.0);            // up boundary

    if (boundary_conditions && !going_away_from_boundary) {
        stop_turtle(i);
        
    }
}



void check_distance(float* pos1, float* pos2, float* prev1, float* prev2) {

    //checks the relative distance between turtle1 and turtle2
    float distance = std::sqrt(std::pow(pos1[0] - pos2[0], 2) + std::pow(pos1[1] - pos2[1], 2));
    float distance_prev = std::sqrt(std::pow(prev1[0] - prev2[0], 2) + std::pow(prev1[1] - prev2[1], 2));
    
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    distance_pub.publish(distance_msg);

    /*stops the moving turtle if the two turtles are “too close” 
    (you may set a threshold to monitor that)*/
    if (distance <= threshold && distance < distance_prev) {
        stop_turtle(1);
        stop_turtle(2);
    }
}

int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "distance_node");  
	ros::NodeHandle nh;
    ros::Rate rate(10);

    
    ros::Subscriber sub_turtle1 = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1, std::bind(turtle_pose, std::placeholders::_1, 1));
    ros::Subscriber sub_turtle2 = nh.subscribe<turtlesim::Pose>("turtle2/pose", 1, std::bind(turtle_pose, std::placeholders::_1, 2));

    pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);
    distance_pub = nh.advertise<std_msgs::Float32>("turtles_distance", 1);
    while(ros::ok) {
        check_distance(turtle1_pos, turtle2_pos, turtle1_prev, turtle2_prev);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}