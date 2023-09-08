# Space Apocalypse
A fun space shooter game in C++ and SFML.

## Contents
- Space Apocalypse
  - [1. Introduction](#1-introduction)
  - [2. Setup](#2-setup)
  - [3. Instructions](#3-instructions)
  - [4. Run](#4-run)
  - [5. Output](#5-output)

## 1. Introduction
This is a fun shooting game written in C++ and SFML that incorporates gesture control using Opencv and Google's MediaPipe library. The game is integrated with ROS-Noetic.

## 2. Setup
In order to setup the repo run the following commands.
Enter the src directory of your catkin workspace
```
cd ~/catkin_ws_name/src/
```
Clone the repository:
```
git clone https://github.com/Vram97/space_apocalypse.git
```
## 3. Instructions
- To control the movement of the player, use the WASD keys.
- To control the player using gesture, make a fist and move in the desired direction. Open the palm to stop the player.
- Use right click to shoot at the falling debris. Getting hit by them will reduce your health.

## 4. Run
This section discusses the setup of parameters and running the launch file.
```
roslaunch space_apocalypse StartGame.launch
```
## 5. Output
![Working GIF](https://github.com/Vram97/space_apocalypse/blob/master/game_tutorial.gif)

