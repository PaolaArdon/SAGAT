# Package created after Michael Burke's adaptation from Travis Dewolf's blog. Recorded motions for PR2: pouring, stacking, handing over and shaking

# dmp_imitation

This package is based heavily on Travis DeWolf's [blog](https://studywolf.wordpress.com/2013/11/16/dynamic-movement-primitives-part-1-the-basics/)  and [repo](https://github.com/studywolf/pydmps), but modified for use with the PR2 and moveit/ros. Scott Niekum has a nice dmp [package](http://wiki.ros.org/dmp), which might also suit your needs better than this one.

# Functionality

## Imitation
* [fit_dmp_joints.py](https://github.com/mgb45/dmp_imitation/blob/master/src/fit_dmp_joints.py) fits a set of dmps to raw joint position, velocity and acceleration motion. The node fits a dmp to each continuous moving segment in a demonstration sequence. 
* [fit_dmp_pose.py](https://github.com/mgb45/dmp_imitation/blob/master/src/fit_dmp_pose.py) fits a set of dmps to end effector postion and orientation demonstrations. The node fits a dmp to each continuous moving segment in a demonstration sequence.

## Playback
* [play_dmp_joints.py](https://github.com/mgb45/dmp_imitation/blob/master/src/play_dmp_joints.py) plays back the set of recorded joint angle space dmps (unsafe).
* [play_dmp_pose.py](https://github.com/mgb45/dmp_imitation/blob/master/src/play_dmp_joints.py) plays back the set of recorded end-effector dmps, going through a moveit planner (safer, but lose original motion dynamics).
* [play_shifted_dmp.py](https://github.com/mgb45/dmp_imitation/blob/master/src/play_shifted_dmp.py) plays back the set of recorded end-effector dmps, going through a moveit planner, at an offset position in 3D space chosen by using rviz clicked_point functionality.

