#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include <string.h>

ros::Publisher pub;

void moveTurtle(float* velocity) {
	geometry_msgs::Twist turtleVel;
	
    turtleVel.linear.x = velocity[0];
    turtleVel.linear.y = velocity[1];
    turtleVel.angular.z = velocity[2];
	
	pub.publish(turtleVel);
    
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

    std::cout << " Choose the velocity of the turtle: ";

    while (!validInput) {
        std::cin >> velocity;

        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
            std::cout << "\nValue not accetable. Digit a number: ";
        } else {
            validInput = true;
        }
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
	turtle2.request.y = 2.0;
	turtle2.request.theta = 0.0;
    turtle2.request.name = "turtle2";
    spawnClient.call(turtle2);
    
    ros::Rate rate(10);

    float velocityChosen[3] = {0, 0, 0};
    
    while(ros::ok) {
        /* textual interface to retrieve the user command (cin)
        select the robot they want to control (turtle1 or turtle2), 
        and the velocity of the robot */

        std::string turtleChosen = chooseTurtle();
        if (turtleChosen == "exit") break;

        for (int i = 0; i < 3; i++) {
            if (i == 0) printf("\n[linear velocity x]");
            else if (i == 1) printf("\n[linear velocity y]");
            else printf("\n[angular velocity z]");
            velocityChosen[i] = selectVelocity();
        }

        pub = nh.advertise<geometry_msgs::Twist>(turtleChosen + "/cmd_vel", 1);

        /*command sent for 1 second then robot stop, 
        and the user should be able again to insert the command*/
        ros::Time start_time = ros::Time::now();
        while (ros::ok && ros::Time::now() - start_time < ros::Duration(1)) {
            moveTurtle(velocityChosen);
            ros::spinOnce();
            rate.sleep();
        }
        
        moveTurtle(new float[3]{0, 0, 0});
        ros::spinOnce();

    }
    
    return 0;
}
