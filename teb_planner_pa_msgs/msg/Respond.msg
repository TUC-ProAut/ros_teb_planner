# result
nav_msgs/Path                       path
geometry_msgs/PoseArray             poses

teb_local_planner/FeedbackMsg       feedback

# waypoints and obstacles
visualization_msgs/Marker           waypoints

visualization_msgs/Marker           obstacles_point
visualization_msgs/Marker           critical_corner_point
visualization_msgs/MarkerArray      obstacles_line
visualization_msgs/MarkerArray      obstacles_poly

# only if using homotopy classes
visualization_msgs/Marker           tebs
