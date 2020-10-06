/******************************************************************************
*                                                                             *
* service_test_node_pa.cpp                                                    *
* ========================                                                    *
*                                                                             *
* This is a simple test-node for checking the interfaces of the               *
* teb_planner_node_pa node. It is also an example of the overall usage.       *
*                                                                             *
*******************************************************************************
*                                                                             *
* Repository:                                                                 *
*   https://github.com/TUC-ProAut/ros_teb_planner                             *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
* Authors:                                                                    *
*   Bhakti Danve, Peter Weissig                                               *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2019-2020 TU Chemnitz                                         *
* All rights reserved.                                                        *
*                                                                             *
* Redistribution and use in source and binary forms, with or without          *
* modification, are permitted provided that the following conditions are met: *
*    * Redistributions of source code must retain the above copyright notice, *
*      this list of conditions and the following disclaimer.                  *
*    * Redistributions in binary form must reproduce the above copyright      *
*      notice, this list of conditions and the following disclaimer in the    *
*      documentation and/or other materials provided with the distribution.   *
*    * Neither the name of the copyright holder nor the names of its          *
*      contributors may be used to endorse or promote products derived from   *
*      this software without specific prior written permission.               *
*                                                                             *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS         *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  *
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR           *
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       *
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         *
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    *
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     *
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF      *
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                  *
*                                                                             *
******************************************************************************/



// local headers
#include "teb_planner_pa_msgs/Plan.h"
#include "teb_planner_pa_msgs/Request.h"

// ros headers
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <teb_local_planner/teb_local_planner_ros.h>
#include <teb_local_planner/optimal_planner.h>

// standard headers
#include <Eigen/Dense>



/***************************[global variables]*********************************/
ros::Subscriber    sub_test;
ros::Subscriber    sub_test2;
ros::ServiceClient service;
ros::Publisher     pub_test2;



/***************************[prototypes]***************************************/
teb_planner_pa_msgs::Request helper_create_request(void);
void CB_test(const std_msgs::EmptyConstPtr& msg);
void CB_test2(const std_msgs::EmptyConstPtr& msg);



/***************************[main]*********************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "service_test_node_pa");
    ros::NodeHandle n("~");
    ros::NodeHandle np;

    // setup callbacks for testing
    sub_test  = n.subscribe("test", 1, CB_test);
    sub_test2 = n.subscribe("test2", 1, CB_test2);

    // service client
    service = np.serviceClient<teb_planner_pa_msgs::Plan>(
      "teb_planner_node_pa/plan");

    // create publisher for test2
    pub_test2 = np.advertise<teb_planner_pa_msgs::Request>(
      "teb_planner_node_pa/request", 1);

    ros::spin();

    return 0;
}



/***************************[helper function]**********************************/
teb_planner_pa_msgs::Request helper_create_request(void)
{
    // start & goal
    geometry_msgs::Pose start_pose, goal_pose;
    tf2::Quaternion q;

    start_pose.position.x = -1;
    start_pose.position.y =  0;
    q.setRPY(0, 0, 0);
    start_pose.orientation = tf2::toMsg(q);

    goal_pose.position.x = 4;
    goal_pose.position.y = 1;
    q.setRPY(0, 0, 0.6);
    goal_pose.orientation = tf2::toMsg(q);


    // waypoints
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = 3;
    pose_stamped.pose.position.y = 0;
    waypoints.poses.push_back(pose_stamped);


    // obstacles
    costmap_converter::ObstacleArrayMsg obstacles;
    geometry_msgs::Point32 point;

    // obst_vector.push_back(ObstaclePtr(new PointObstacle( 2, 2)));
    costmap_converter::ObstacleMsg obst1;
    point.x = 2; point.y = 2;
    obst1.polygon.points.push_back(point);
    obstacles.obstacles.push_back(obst1);

    // obst_vector.push_back(ObstaclePtr(new CircularObstacle( 2,-1, 0.25)));
    costmap_converter::ObstacleMsg obst2;
    point.x = 2; point.y = -1;
    obst2.polygon.points.push_back(point);
    obst2.radius = 0.25;
    obstacles.obstacles.push_back(obst2);
    // wepet: circular obstacle is not shown in rviz!

    // obst_vector.push_back(ObstaclePtr(new LineObstacle ( 1, 0, 1, 1)));
    costmap_converter::ObstacleMsg obst3;
    point.x = 1; point.y = 0;
    obst3.polygon.points.push_back(point);
    point.x = 1; point.y = 1;
    obst3.polygon.points.push_back(point);
    obstacles.obstacles.push_back(obst3);


    // construct (service-)request
    teb_planner_pa_msgs::Request req;
    req.start = start_pose;
    req.goal  = goal_pose;
    req.waypoints = waypoints;
    req.obstacles = obstacles;

    return req;
}



/***************************[callbacks]****************************************/
void CB_test(const std_msgs::EmptyConstPtr& msg)
{
    // construct service
    teb_planner_pa_msgs::Plan serv;
    serv.request.request = helper_create_request();

    // call service
    if (service.call(serv))
    {
        ROS_INFO("service call ok :-)");
    }
    else
    {
        ROS_INFO("error during service call");
    }
}

void CB_test2(const std_msgs::EmptyConstPtr& msg)
{
    // publish to topic
    ROS_INFO("sending request as topic");
    pub_test2.publish(helper_create_request());
}
