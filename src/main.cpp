/********************************************************************************************
* This is the main entry point for the game.
* The game is a fun shooting game with gesture control, wrapped as a ROS package.
* You gain points as you shoot (RIGHT CLICK) at the falling debris and lose health as you collide with them.
* You can move the player either by using the keyboard (WASD) or by using hand movement.
* Make a fist to move the player and open your hand to stop the player.
* The game ends when you lose all your health.
*********************************************************************************************/
/* Author: Shivaram Srikanth 
   Date: 09/14/2023        */

#include "Game.h"

#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include <time.h>



int main(int argc, char** argv)
{
    // Initializing ROS Node.
    ros::init(argc, argv, "game");
    ros::NodeHandle nh;

    srand(static_cast<unsigned>(time(NULL)));
    
    // Initializing game object.
    Game game;

    ROS_INFO("Game started");

    // Subscribing to the hand tracker publisher.
    // The default rate is 25 Hz because my hand tracker publishes at 25 Hz.
    ros::Subscriber sub = nh.subscribe("hand_tracker_publisher", 25, &Game::movePlayer, &game);

    // Running the game.
    game.run();

    return 0;
}