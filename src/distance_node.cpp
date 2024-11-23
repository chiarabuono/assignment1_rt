#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include <string.h>
#include "std_msgs/Float32.h"


struct Turtle {
    float x;
    float y;
    float theta;
    float x_prev;
    float y_prev;
    float theta_prev;
};

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher distance_pub;

Turtle turtle1;
Turtle turtle2;
float threshold = 2;

void stop_turtle(int i) {
    geometry_msgs::Twist turtleVel;
    turtleVel.linear.x = 0;
    turtleVel.linear.y = 0;
    turtleVel.angular.z = 0;
	
	if (i == 1) pub1.publish(turtleVel);
    else pub2.publish(turtleVel);
}

void turtle_pose(const turtlesim::Pose::ConstPtr& msg, Turtle &turtle) {
    turtle.x_prev = turtle.x;
    turtle.y_prev = turtle.y;
    turtle.theta_prev = turtle.theta;

    turtle.x = msg->x;
    turtle.y = msg->y;
    turtle.theta = msg->theta;

    bool boundary_conditions = turtle.x >= 10.0 || turtle.x <= 1.0 || 
                               turtle.y >= 10.0 || turtle.y <= 1.0;
    bool going_away_from_boundary = (turtle.x_prev >= turtle.x && turtle.x_prev >= 10.0) ||
                                    (turtle.x_prev <= turtle.x && turtle.x_prev <= 1.0) || 
                                    (turtle.y_prev >= turtle.y && turtle.y_prev >= 10.0) || 
                                    (turtle.y_prev <= turtle.y && turtle.y_prev <= 1.0); 

    if (boundary_conditions && !going_away_from_boundary) {
        stop_turtle(&turtle == &turtle1 ? 1 : 2);
    }
}

void check_distance(Turtle t1, Turtle t2) {

    //checks the relative distance between turtle1 and turtle2
    float distance = std::sqrt(std::pow(t1.x - t2.x, 2) + std::pow(t1.y - t2.y, 2));
    float distance_prev = std::sqrt(std::pow(t1.x_prev- t2.x_prev, 2) + std::pow(t1.y_prev - t2.y_prev, 2));
    
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
    ros::Rate rate(100);

    
    ros::Subscriber sub_turtle1 = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1, std::bind(turtle_pose, std::placeholders::_1, std::ref(turtle1)));
    ros::Subscriber sub_turtle2 = nh.subscribe<turtlesim::Pose>("turtle2/pose", 1, std::bind(turtle_pose, std::placeholders::_1, std::ref(turtle2)));
    
    pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);
    distance_pub = nh.advertise<std_msgs::Float32>("turtles_distance", 1);

    while(ros::ok) {
        check_distance(turtle1, turtle2);
        ros::spinOnce();
        rate.sleep();
     }
    return 0;
}