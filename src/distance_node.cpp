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

// struct Turtle {
//     float x;
//     float y;
//     float theta;
// };

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
    turtle1_prev[0] = turtle1_pos[0]; turtle1_prev[1] = turtle1_pos[1]; turtle1_prev[2] = turtle1_pos[2];
    turtle1_pos[0] = msg -> x;
    turtle1_pos[1] = msg -> y;
    turtle1_pos[2] = msg -> theta;

    /*stops the moving turtle if the position 
    is too close to the boundaries 
    (.e.g, x or y > 10.0, x or y < 1.0) */
    bool boundary_conditions = turtle1_pos[0] >= 10.0 || turtle1_pos[0] < 1.0 || turtle1_pos[1] >= 10.0 || turtle1_pos[1] < 1.0;
    bool going_away_from_boundary = (turtle1_prev[0] > turtle1_pos[0] && turtle1_prev[0] >= 10.0) ||        // right boundary
                                    (turtle1_prev[0] < turtle1_pos[0] && turtle1_prev[0] < 1.0) ||          // left boundary
                                    (turtle1_prev[1] > turtle1_pos[1] && turtle1_prev[1] >= 10.0) ||        // down boundary
                                    (turtle1_prev[1] < turtle1_pos[1] && turtle1_prev[1] < 1.0);            // up boundary

    if (boundary_conditions && !going_away_from_boundary) {
        stop_turtle1();
        
    }
}

void turtle2_pose(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_prev[0] = turtle2_pos[0]; turtle2_prev[1] = turtle2_pos[1]; turtle2_prev[2] = turtle2_pos[2];
    turtle2_pos[0] = msg -> x;
    turtle2_pos[1] = msg -> y;
    turtle2_pos[2] = msg -> theta;

    bool boundary_conditions = turtle2_pos[0] >= 10.0 || turtle2_pos[0] < 1.0 || turtle2_pos[1] >= 10.0 || turtle2_pos[1] < 1.0;
    bool going_away_from_boundary = (turtle2_prev[0] > turtle2_pos[0] && turtle2_prev[0] >= 10.0) ||        // right boundary
                                    (turtle2_prev[0] < turtle2_pos[0] && turtle2_prev[0] < 1.0) ||          // left boundary
                                    (turtle2_prev[1] > turtle2_pos[1] && turtle2_prev[1] >= 10.0) ||        // down boundary
                                    (turtle2_prev[1] < turtle2_pos[1] && turtle2_prev[1] < 1.0);            // up boundary
    

    if (boundary_conditions && !going_away_from_boundary) {
        stop_turtle2();
        
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
        stop_turtle1();
        stop_turtle2();
    }
}

int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "distance_node");  
	ros::NodeHandle nh;

    ros::Subscriber sub_turtle1 = nh.subscribe("turtle1/pose", 1, turtle1_pose);
    ros::Subscriber sub_turtle2 = nh.subscribe("turtle2/pose", 1, turtle2_pose);

    pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);

    ros::Rate loop_rate(10);
    
    while (ros::ok) {
        

        /* publish on a topic the distance 
        (you can use a std_msgs/Float32 for that)*/
        distance_pub = nh.advertise<std_msgs::Float32>("turtles_distance", 1);
        check_distance(turtle1_pos, turtle2_pos, turtle1_prev, turtle2_prev);


        
        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}