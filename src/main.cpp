#include "Game.h"
#include <time.h>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>

int main(int argc, char** argv)
{
    // Initialize ROS Node
    ros::init(argc, argv, "game");
    ros::NodeHandle nh;

    srand(static_cast<unsigned>(time(NULL)));
    
    Game game;

    ROS_INFO("Game started");
    ros::Subscriber sub = nh.subscribe("hand_tracker_publisher", 25, &Game::movePlayer, &game);


    game.run();

    // ros::spin();

    return 0;
}