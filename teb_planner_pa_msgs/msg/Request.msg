# start and goal pose
geometry_msgs/Pose start
geometry_msgs/Pose goal

# waypoints and obstacles
nav_msgs/Path                      waypoints
costmap_converter/ObstacleArrayMsg obstacles

# initial plan
geometry_msgs/PoseArray initial_plan

# robot start and goal velocities
geometry_msgs/Twist start_vel
geometry_msgs/Twist goal_vel
