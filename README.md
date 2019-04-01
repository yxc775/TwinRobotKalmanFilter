# TwinRobotKalmanFilter
two robots using specialized Kalman Filter with Beacon support

# How to Use

Install supported C++ lib and Gazebo library.
open temrinal and write:

**mkdir build**

**cd build**

**cmake ..**

**make**

to compile the file

After this step, move all dylib file from build file to replace all dylib file in parent folder.
open a new terminal and write

**gzserver -u projectWorld.world --verbose**

Then, open another terminal and write 

**gzclient**

which should allow you to run the simulation




