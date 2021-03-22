# teb_planner_pa_matlab package

This package only contains the matlab wrapper (class) for the
teb_planner_pa package.

This folder contains two primary folders:
* [msgs](msgs)
    * costmap_converter
    * teb_local_planner
    * teb_planner_pa
    * visualisation_msgs
    * matlab_gen
* [scripts](scripts)
    * startup.m
    * TebPlanner.m
    * TebPlannerExample.m

'msgs' folder contains all the required custom messages and a matlab_gen folder.
The matlab_gen folder is a Matlab generated folder containing the custom messages
converted as per Matlab requirements. See 'Matlab' Section below.

## [Startup Matlab Script](scripts/startup.m)

To make sure if all the custom messages are correctly loaded in Matlab, a startup
script is created as a part of this package.
Run this script before starting to work with the TebPlanner class in Matlab.

## [TebPlanner Class](scripts/TebPlanner.m)

As mentioned in the package features, this Matlab Class contains various methods
to gather plan details from Matlab and send it to ROS teb_planner_pa package.
Provision of creating a teb plan either through Services or Topics is made.

**Class Methods:**

Method               | Inputs                     | Description
---------------------|----------------------------|----------------------------------------------------------
setStartPose         | x, y, theta                | Robot's start position(x,y) and direction(theta)
setGoalPose          | x, y, theta                | Robot's goal  position(x,y) and direction(theta)
setInitialPlan       | poses                      | Initial or Global plan poses [x1,y1,theta1; x2,y2,theta2; ...]
setStartVelocity     | vx, vy, omega              | Robot's linear (vx, vy) and angular(omega) start velocity
addCircularObstacle  | position, velocity, radius | position is a vector (x,y); velocity is a vector (vx, vy); radius is a scalar
addPolylineObstacle  | positions, velocity        | positions - minimum two   positions [x1,y1; x2,y2] to form a polyline obstacle; velocity - [vx, vy] components
addPolygonObstacle   | positions, velocity        | positions - minimum three positions [x1,y1; x2,y2; x3,y3] to form a polygon obstacle; velocity - [vx, vy] components
addWaypoint          | x, y                       | Waypoint Position
clearXxxxObstacle    | ---                        | Dedicated clear functions for each of the obstacle types
clearWaypoint        | ---                        | Deletes all waypoints
plan                 | ---                        | Requests for Teb Plan using a service call by sending all the plan details. Stores Response in one of the Class properties
plan_using_topics    | ---                        | Same as plan but based on the topics instead of a service call (for testing)
replan               | ---                        | Replanning using the previous plan details. Stores Response in one of the Class properties
replan_using_topics  | ---                        | Same as replan but based on topics instead of a service call
getResultPoses       | ---                        | Returns the planned trajectory consisting of poses
getResultFeedback    | ---                        | Returns the planned trajectory consisting of poses and timestamps


## Matlab

If you are running **Matlab 2020b** or newer - stay here.
Otherwise have a look [here](README_old.md).

Usage instructions based on
  https://de.mathworks.com/help/ros/gs/ros-system-requirements.html and
  https://de.mathworks.com/matlabcentral/answers/623103

Tested on:

* Matlab 2020b on Ubuntu 20.04 with ros noetic \
  (also prebuild messages are stored in matlab_gen)

### 1. Download package
Download this matlab package, which is part of the ProAut TEB-Planner
repository.

~~~~~
    e.g.
    $ mkdir -p ~/catkin_ws/src
    $ cd       ~/catkin_ws/src
    $ git clone https://github.com/TUC-ProAut/ros_teb_planner
    $ cd ros_teb_planner
    $ pwd

~~~~~

In the remainder of the instructions, it is assumed that the path
"~/catkin_ws/src/ros_teb_planner" is "REPO_PATH/".

### 2. Fixing cmake-dependencies
This step is only necessary, if you want to rebuild the messages.
**Before** starting matlab check and update LD_PRELOAD.

~~~~~
    $ echo "LD_PRELOAD=\"$LD_PRELOAD\""
    $ export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
~~~~~

If LD_PRELOAD was not empty before, consider adding the previous paths: \
  `$ export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6:$LD_PRELOAD"`

### 3. Setup Python
**After** starting matlab, check current python version.
If it is not the deprecated Python 2.7, then change the python enviroment.

~~~~~
    >> pyenv()
    >> pyenv('Version', 'python2.7')
~~~~~

### 4. Run rosgenmsg
Within Matlab, run rosgenmsg on the folder containing the custom message
definitions.

~~~~~
    >> rosgenmsg('REPO_PATH/teb_planner_pa_matlab/msgs/')
~~~~~

### 5. Restart Matlab
The caption says it all.

### 6. done
You should now be able to use these new message definitions.
