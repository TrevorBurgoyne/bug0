# bug0
HW5 for CSCI 5551. The Bug-0 algorithm attempts to navigate to a goal location by following a straight-line path from its current position to the goal.

# Notes for package install
dashs in package names, use prefix `ros-noetic-`.
For WSL, make sure Display is being redirected to VcXsrv

    >> `source /opt/ros/noetic/setup.bash`  

In one terminal:

    >> roscore

In another terminal, navigate to `bug0/stage` and run:

    >> rosrun stage_ros stageros bug-test.world

In a third terminal, navigate to `bug0` and run:

    >> catkin_make
    >> source devel/setup.bash
    >> rosrun stage_control stage_mover.py