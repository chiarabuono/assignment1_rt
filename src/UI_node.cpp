#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include <string.h>

ros::Publisher pub;

void selectTurtle(ros::NodeHandle& nh, std::string nameTurtle) {

    pub = nh.advertise<geometry_msgs::Twist>(nameTurtle + "/cmd_vel", 1);
    //ros::Subscriber sub2 = nh.subscribe(nameTurtle + "/pose", 1, turtleCallback);


}

std::string chooseTurtle() {
    int turtleChosen = 0;
    std::cout << "\nChoose the turtle you want move (1 or 2): ";
    std::cin >> turtleChosen;

    while (turtleChosen != 1 && turtleChosen != 2) {
        std::cout << "\nValue not accetable. Choose between turtle1 (1) or turtle2 (2): ";
        std::cin >> turtleChosen;
    }

    return "turtle" + std::to_string(turtleChosen); 
}

int selectVelocity() {
    int velocity;
    bool validInput = false;

    while (!validInput) {

        std::cout << "\nChoose the velocity of the turtle: ";
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
	turtle2.request.y = 1.0;
	turtle2.request.theta = 0.0;
    turtle2.request.name = "turtle2";
    spawnClient.call(turtle2);
    
    /* textual interface to retrieve the user command (cin)
    select the robot they want to control (turtle1 or turtle2), 
    and the velocity of the robot */
    
    std::string turtleChosen = chooseTurtle();
    selectTurtle(nh, turtleChosen);
    int velocityChosen = selectVelocity();
    
    /*command sent for 1 second then robot stop, 
    and the user should be able again to insert the command*/
    


    ros::spin();
    return 0;
}
