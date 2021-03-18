# start and goal pose
# note: both poses will be ignored, if initial_plan is set
geometry_msgs/Pose start
geometry_msgs/Pose goal

# robot start and goal velocities
geometry_msgs/Twist start_vel
geometry_msgs/Twist goal_vel

# obstacles
costmap_converter/ObstacleArrayMsg obstacles

# waypoints
# note: waypoints will usually be ignored, if initial_plan is set
nav_msgs/Path waypoints

# critical corners
costmap_converter/ObstacleArrayMsg critical_corners

# initial plan
# note: always overrides start & goal pose, if not empty
# note: usually overrides waypoints, if not empty
nav_msgs/Path initial_plan
