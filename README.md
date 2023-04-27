# bug0
HW5 for CSCI 5551. The Bug-0 algorithm attempts to navigate to a goal location by following a straight-line path from its current position to the goal, and turning to move around any obstacles in its path. 

# About this Implementation
Since the LaserScanner in the model only provides a 270 degree view around the robot (read: it has a blind spot behind), 
a decision had to be made as to what the robot would do in the case where the straight path towards the goal points somewhere in the blind spot. Either the
robot will assume that this "unkwown" region is (a) obstructed (read: it will NOT turn to face that direction) or (b) unobstructed (read: it WILL turn to face that direction). 
**This implementation uses option (a)**, and so when the goal lies in a direction that falls within the bot's sensory blind spot, it will continue along its current trajectory until
it turns enough to get a reading in that direction. This option was chosen because when implementing option (b), there arise situations where the bot will get stuck turning towards
the goal, only to find that it is blocked by an obstacle. Once it correctly avoids the obstacle, it will "forget" that a obstacle lies in its blind spot, and once again turn to that direction, getting stuck in a near-infinite loop. 

# Run Instructions
First, be sure to run this command in every terminal wherein you want to run ros commands. This can be set up to occur automatically by following the steps [here](https://answers.ros.org/question/206876/how-often-do-i-need-to-source-setupbash/).

    >> `source /opt/ros/noetic/setup.bash`  

In one terminal, start the core ROS program by running:

    >> roscore

In another terminal, navigate to `bug0/stage/` and run:

    >> rosrun stage_ros stageros bug-test.world
This will open a window with the `bug-test.world` map, which represents the robot with a dark blue square, with shaded blue regions where the LaserScan is collecting data.

In a third terminal, navigate to `bug0/` and run:

    >> catkin_make
    >> source devel/setup.bash
    >> rosrun stage_control bug0.py
This will build the necessary packages and start the main function in `bug0.py`
Follow the on-screen instructions and enter in your desired options for:
- A left/right turning robot
- The x and y locations of the goal
- The desired final robot orientation in radians (a number from -pi to pi)

The robot will then navigate to the goal according the bug0 algorithm. Once it has reached the desired final state (within a tolerance), the robot will stop and a final message will be printed before the program exits. 

# Notes for additional package install
If additional packages are desired, install them using the version that has the prefix `ros-noetic-`. 

# Notes for running in WSL2
For WSL, make sure your display output is being redirected via VcXsrv or some other application, else the map will not appear. See [this](https://jackkawell.wordpress.com/2020/06/12/ros-wsl2/) article for details.