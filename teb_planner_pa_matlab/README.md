# teb_planner_pa_matlab package

This package only contains the matlab wrapper (class) for the
teb_planner_pa package.

This folder contains two primary folders:
* [msgs](matlab/msgs)
    * costmap_converter
    * teb_local_planner
    * teb_planner_pa
    * visualisation_msgs
    * matlab_gen
* [scripts](matlab/scripts)
    * startup.m
    * TebPlanner.m

'msgs' folder contains all the required custom messages and a matlab_gen folder.
The matlab_gen folder is a Matlab generated folder containing the custom messages
converted as per Matlab requirements. See 'Matlab' Section below.

## [Startup Matlab Script](matlab/scripts/startup.m)

To make sure if all the custom messages are correctly loaded in Matlab, a startup
script is created as a part of this package.
Run this script before starting to work with the TebPlanner class in Matlab.

## [TebPlanner Class](matlab/scripts/TebPlanner.m)

As mentioned in the package features, this Matlab Class contains various methods
to gather plan details from Matlab and send it to ROS teb_planner_pa package.
Provision of creating a teb plan either through Services or Topics is made.

**Class Methods:**

Method               | Inputs      | Description
---------------------|-------------|----------------------------------------------------------
setStartPose         | x, y, theta | Robot's start position(x,y) and direction(theta)
setGoalPose          | x, y, theta | Robot's goal  position(x,y) and direction(theta)
setInitialPlan       | poses       | Initial or Global plan poses; minimum 6 poses [x1,y1,theta1; ...x6,y6,theta6]
setStartVelocity     | vx, vy, omega | Robot's linear (vx, vy) and angular(omega) start velocity
addCircularObstacle  | position, velocity, radius | position is a vector (x,y); velocity is a vector (vx, vy); radius is a scalar
addPolylineObstacle  | positions, velocity | positions - minimum two positions [x1,y1; x2,y2] to form a polyline obstacle; velocity - [vx, vy] components
addPolygonObstacle   | positions, velocity | positions - minimum three positions [x1,y1; x2,y2; x3,y3] to form a polygon obstacle; velocity - [vx, vy] components
addWaypoint          | x, y       | Waypoint Position
clearXxxxObstacle    | ---        | Dedicated clear functions for each of the obstacle types
clearWaypoint        | ---        | Deletes all waypoints
plan                 | ---        | Requests for Teb Plan using a service call by sending all the plan details. Stores Response in one of the Class properties
plan_using_topics    | ---        | Same as plan but based on the topics instead of a service call (for testing)
getResultPoses       | ---        | Returns the planned trajectory consisting of poses
getResultFeedback    | ---        | Returns the planned trajectory consisting of poses and timestamps


## Matlab

Usage instructions based on
  https://de.mathworks.com/matlabcentral/answers/283695 \
"Updating existing custom message types with rosgenmsg"

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

### 2. Run rosgenmsg
Within matlab: run rosgenmsg on the folder containing the custom
message definitions.

~~~~~
    >> rosgenmsg('REPO_PATH/teb_planner_pa_matlab/msgs/')
~~~~~

In order to keep this instruction simple "MSGS_PATH" refers to
"REPO_PATH/teb_planner_pa_matlab/msgs/".

### 3. Edit the javaclasspath.txt
Follow the instructions to edit the javaclasspath.txt file.

In addition to the four JAR file paths, you also need to tell
Matlab to use these JAR files instead of the builtin ones. Add
the "**before**" token in front of the four JAR file paths:

~~~~~
    <before>
    MSGS_PATH/matlab_gen/jar/costmap_converter-0.0.11.jar
    MSGS_PATH/matlab_gen/jar/teb_local_planner-0.6.14.jar
    MSGS_PATH/matlab_gen/jar/teb_planner_pa_msgs-1.0.0.jar
    MSGS_PATH/matlab_gen/jar/visualization_msgs-1.12.7.jar
~~~~~

### 4. Restart Matlab
The caption says it all.

### 5. Regenerate files
Delete the previously generated Matlab files and run
rosgenmsg again. Now, it should pick up the new definitions:

~~~~~
    >> rmdir('MSGS_PATH/matlab_gen/', 's')
    >> rosgenmsg('MSGS_PATH')
~~~~~

### 6. done
You should now be able to use these new message definitions.
