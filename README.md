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

After this step, **move the projectWorld.world file into the build folder.**  
Then, open terminal and input 
**gzserver -u projectWorld.world --verbose**

Then, open another terminal and write 
**gzclient**

this should allow you to run the simulation




