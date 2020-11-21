# teb_planner_pa package

The teb_planner_pa package is developed with the primary aim of having a
systematic handshake between Matlab Environment and
ROS [teb_local_planner](http://wiki.ros.org/teb_local_planner) package.

## Features

* [wrapper node](src/teb_planner_node_pa.cpp) to call teb-planner
without navigation stack
    * based on test_optim_node
    * single topics for playing around/testing the basic functionalities of
    the teb-planner
    * service for convenient call (setting all plan details)/ also topic interface
    for testing
* [test-node](src/service_test_node_pa.cpp) to check service-based
interface

*Hint: This package is currently implemented and tested only for ROS kinetic and*
*Matlab R2020a*


## Nodes

The nodes of this package make use of services and topics to interact with the
teb_local_planner in a simplified manner.


### 1. teb_planner_node_pa
This node is developed by customising the test_optim_node of teb_local_planner
package. It is a standalone node that calls the teb-planner once the
plan-details are provided to it.


```
    roslaunch teb_planner_pa teb_planner_node_pa.launch
```

**Node Topics**

Topic Name      | Type                 | Description
----------------|----------------------|------------------------------------------------------------------
/set_start2d    | geometry_msgs/Pose2D | Sets the start pose of the trajectory with inputs (x, y, theta)
/set_goal2d     | geometry_msgs/Pose2D | Sets the goal pose of the trajectory with inputs (x, y, theta)
/set_initialplan| geometry_msgs/PoseArray | Sets the initial plan or global plan for teb planner
/add_obstacles  | costmap_converter/ObstacleArrayMsg| Adds obstacles (possible types are point, line, circular, polygonal)
/clear_obstacles| std_msgs/Empty       | Delete all obstacles
/add_waypoints  | nav_msgs/Path        | Adds waypoints (via-points)
/clear_waypoints| std_msgs/Empty       | Deletes all waypoints
/set_startvelocity| geometry_msgs/Twist| Sets the start velocity of the robot (vx, vy, omega)
/request        | [teb_planner_pa_msgs/Request](../teb_planner_pa_msgs/msg/Request.msg)| Service-like topic (see also Plan Service and Custom Messages)
/respond        | [teb_planner_pa_msgs/Respond](../teb_planner_pa_msgs/msg/Respond.msg)| Service-like topic (see also Plan Service and Custom Messages)
/request_replan | std_msgs/Empty       | Service-like topic to replan using previous plan details (see also Plan Service)

**Node Services**

Service Name | Type           | Description
-------------|----------------|------------------------------------------------------------------
/plan        | [teb_planner_pa_msgs/Plan](../teb_planner_pa_msgs/srv/Plan.srv) | Sets all plan-details (e.g. start pose), calls the teb-planner optimisation and returns the resulting trajectory
/replan      | [teb_planner_pa_msgs/Plan](../teb_planner_pa_msgs/srv/Plan.srv) | Replans using previous plan details and returns the resulting trajectory

*The 'plan' may take a while to provide the service depending on the plan-details*
*and package parameters*



### 2. service_test_node_pa
This node is developed for testing the interfaces (service and via topics) of
teb_planner_node_pa. The tests are triggered by publishing any message to one
of the two test topics. To perform the test using this node, the
teb_planner_node_pa must also be active.



```
    roslaunch teb_planner_pa service_test_node_pa.launch
```

**Node Topics**

Topic Name | Type           | Description
-----------|----------------|---------------------------------------------------
/test      | std_msgs/Empty | Requests for plan service of teb_planner_pa node by adding plan-details (Publisher for this topic written in service_test_node_pa.launch file)
/test2     | std_msgs/Empty | Same as 'test' topic

##  Plan Service and Custom Messages


The Plan Service contains request and response:

~~~~~
    teb_planner_pa_msgs/Request request
    ---
    teb_planner_pa_msgs/Respond respond
~~~~~

They are of custom message type [Request](../teb_planner_pa_msgs/msg/Request.msg) and [Respond](../teb_planner_pa_msgs/msg/Respond.msg)
respectively. Various messages types are combined to form the custom messages:

- **Request**: start, goal, waypoints, obstacles, start_vel
- **Respond**: path, poses, feedback, visualisation markers such as
  obstacle_point, obstacle_line, obstacle_polygon etc.

**Note:** As mentioned in 'Node Topics' section of the teb_planner_node_pa node,
the topics /request and /respond are also of message types teb_planner_pa_msgs/Request and
teb_planner_pa_msgs/Respond respectively. Thus, the names of topics and those of the
Service request and response belonging to similar custom message types are kept same.

Furthermore, the name of the Service request being 'request' and that of Service response
being 'respond' have resulted in unusual code lines in Matlab such as:

```
request.Request = obj.getRequestMsg();
obj.latestMsg = response.Respond;

```

In the above code lines, the service request is accessed as request.*Service_request_name*,
whereas the service response is accessed as response.*Service_response_name*.


## Links

### ROS Package
teb_planner_pa

*planned for publishing on ros buildfarm soon*

### Source code at github
https://github.com/TUC-ProAut/ros_teb_planner 

### ROS Build-Status and Documentation

ROS-Distribution              | Build-Status      | Documentation
---------------------|-------------|----------------------------------------------------------
Kinetic              | ---| ---
