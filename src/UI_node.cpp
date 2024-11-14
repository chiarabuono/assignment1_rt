#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include <string.h>

ros::Publisher pub;
ros::Subscriber sub;

void moveTurtle(const turtlesim::Pose::ConstPtr& msg, float velocity) {
	ROS_INFO("Turtle subscriber@[%f, %f, %f]", msg->x, msg->y, msg->theta);
	geometry_msgs::Twist turtleVel;
	
    turtleVel.linear.x = velocity;
    // turtleVel.angular.z = 2.0;
	
	pub.publish(turtleVel);
    
	}

void selectTurtle(ros::NodeHandle& nh, std::string nameTurtle, float velocity) {

    pub = nh.advertise<geometry_msgs::Twist>(nameTurtle + "/cmd_vel", 1);
    sub = nh.subscribe<turtlesim::Pose>(nameTurtle + "/pose", 1, std::bind(moveTurtle, std::placeholders::_1, velocity));

}

std::string chooseTurtle() {
    std::string input;
    std::cout << "\nChoose the turtle you want to move (1 or 2). Press q to exit: ";
    std::cin >> input;

    if (input == "q") return "exit";

    while (input != "1" && input != "2") {
        std::cout << "\nValue not acceptable. Choose between turtle1 (1) or turtle2 (2). Press q to exit: ";
        std::cin >> input;
        if (input == "q") {
            return "exit";
        }
    }
    return "turtle" + input; 
}

float selectVelocity() {
    float velocity;
    bool validInput = false;

    std::cout << "\nChoose the velocity of the turtle: ";

    while (!validInput) {

        
        std::cin >> velocity;

        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
            std::cout << "\nValue not accetable. Digit a number: ";

        } else {
            validInput = true;
        }
        sleep(1);
    }
    return velocity;
}



int main(int argc, char **argv) {
    
    // init node
    ros::init(argc, argv, "UI_node");  
	ros::NodeHandle nh;

    // Spawn a new turtle in the environment: turtle2
    ros::ServiceClient spawnClient =  nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn turtle2;
    turtle2.request.x = 2.0;  
	turtle2.request.y = 1.0;
	turtle2.request.theta = 0.0;
    turtle2.request.name = "turtle2";
    spawnClient.call(turtle2);
    
    //ros::Rate rate(1);
    
    while(ros::ok) {
        /* textual interface to retrieve the user command (cin)
        select the robot they want to control (turtle1 or turtle2), 
        and the velocity of the robot */

        std::string turtleChosen = chooseTurtle();
        if (turtleChosen == "exit") break;

        float velocityChosen = selectVelocity();  

        /*command sent for 1 second then robot stop, 
        and the user should be able again to insert the command*/
        selectTurtle(nh, turtleChosen, velocityChosen);
        
        sleep(1);
        ros::spinOnce();

    }
    
    return 0;
}
