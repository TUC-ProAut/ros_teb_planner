/******************************************************************************
*                                                                             *
* teb_planner_node_pa.cpp                                                     *
* =======================                                                     *
*                                                                             *
* This node is based on the test_optim_node.cpp. We extended the simple test  *
* node to have more configuration options while running:                      *
*   + changing start & goal pose                                              *
*   + adding & removing obstacles/waypoints                                   *
*   + resetting the planner                                                   *
*     (e.g. to change the the planner type or the robots' footprint)          *
*                                                                             *
* We are only interested in one-shot planning and not in continuously         *
* replanning. Therefore, we:                                                  *
*  + removed the timer for automatic replanning                               *
*  + added a service (and a topic) to force a replan                          *
*  + added a topic to republish the last result                               *
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
* Original Source Code                                                        *
*                                                                             *
*     Repository:                                                             *
*       https://github.com/rst-tu-dortmund/teb_local_planner                  *
*                                                                             *
*     File:                                                                   *
*       src/test_optim_node.cpp                                               *
*                                                                             *
*     Institute of Control Theory and Systems Engineering, TU Dortmund        *
*       https://rst.etit.tu-dortmund.de/                                      *
*                                                                             *
*     Author:                                                                 *
*       Christoph Rösmann                                                     *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2019-2020 TU Chemnitz                                         *
* Copyright (c) 2016-2019 TU Dortmund                                         *
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
#include "teb_local_planner/visualization_pa.h"

#include "teb_planner_pa_msgs/Plan.h"
#include "teb_planner_pa_msgs/Request.h"
#include "teb_planner_pa_msgs/Respond.h"

// ros headers
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <teb_local_planner/homotopy_class_planner.h>

// standard headers
#include <complex>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

/***************************[global variables]*********************************/
teb_local_planner::PlannerInterfacePtr planner;
teb_local_planner::TebVisualizationPaPtr visual;
std::vector<teb_local_planner::ObstaclePtr> obst_vector;
teb_local_planner::ViaPointContainer waypoints;
teb_local_planner::TebConfig config;
boost::shared_ptr <dynamic_reconfigure::Server
  <teb_local_planner::TebLocalPlannerReconfigureConfig> > dynamic_recfg;

ros::ServiceServer srv_plan;
ros::ServiceServer srv_replan;

ros::Subscriber sub_reset;
ros::Subscriber sub_clear;
ros::Subscriber sub_plan;
ros::Subscriber sub_publish;

ros::Subscriber sub_request;
ros::Subscriber sub_request_replan;
ros::Publisher  pub_respond;

ros::Subscriber sub_setStart2d;
ros::Subscriber sub_setGoal2d;
ros::Subscriber sub_setStart;
ros::Subscriber sub_setGoal;

ros::Subscriber sub_setInitialPlan;

ros::Subscriber sub_clearObstacles;
ros::Subscriber sub_setObstacles;
ros::Subscriber sub_addObstacles;

ros::Subscriber sub_clicked_points;
ros::Subscriber sub_clearWaypoints;
ros::Subscriber sub_setWaypoints;
ros::Subscriber sub_addWaypoints;

ros::Subscriber sub_setRobotStrtVelocity;

teb_local_planner::PoseSE2 start_pose, goal_pose;
std::vector<geometry_msgs::PoseStamped> init_plan;
geometry_msgs::Twist start_vel;


/***************************[prototypes]***************************************/
void CB_reconfigure(teb_local_planner::TebLocalPlannerReconfigureConfig&
  reconfig, uint32_t level);

bool service_plan(teb_planner_pa_msgs::Plan::Request  &req,
  teb_planner_pa_msgs::Plan::Response &res);
void CB_request(const teb_planner_pa_msgs::RequestConstPtr& msg);

bool service_replan(teb_planner_pa_msgs::Plan::Request  &req,
  teb_planner_pa_msgs::Plan::Response &res);
void CB_request_replan(const std_msgs::EmptyConstPtr& msg);

void CB_reset(const std_msgs::EmptyConstPtr& msg);
void CB_clear(const std_msgs::EmptyConstPtr& msg);
void CB_plan(const std_msgs::EmptyConstPtr& msg);
void CB_publish(const std_msgs::EmptyConstPtr& msg);

void CB_setStart2d(const geometry_msgs::Pose2DConstPtr& pose_msg);
void CB_setGoal2d(const geometry_msgs::Pose2DConstPtr& pose_msg);
void CB_setStart(const geometry_msgs::PoseConstPtr& pose_msg);
void CB_setGoal(const geometry_msgs::PoseConstPtr& pose_msg);

void CB_setInitialPlan(const geometry_msgs::PoseArray::ConstPtr& init_plan_msg);
void CB_clearInitialPlan(const std_msgs::EmptyConstPtr& msg);
void CB_setRobotStartVelocity(const geometry_msgs::Twist& vel_start);

void CB_clearObstacles(const std_msgs::EmptyConstPtr& msg);
void CB_setObstacles(const costmap_converter::ObstacleArrayMsg::ConstPtr&
  obst_msg);
void CB_addObstacles(const costmap_converter::ObstacleArrayMsg::ConstPtr&
  obst_msg);

void CB_clicked_point(const geometry_msgs::PointStampedConstPtr& point_msg);
void CB_clearWaypoints(const std_msgs::EmptyConstPtr& msg);
void CB_setWaypoints(const nav_msgs::Path::ConstPtr& waypoints_msg);
void CB_addWaypoints(const nav_msgs::Path::ConstPtr& waypoints_msg);
void updateWaypointsContainer(const std::vector<geometry_msgs::PoseStamped>&
  initial_plan, double min_separation);
bool checkGlobalWaypoints(void);
bool infoCustomWaypoints(void);



/***************************[main]*********************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "teb_planner_node_pa");
    ros::NodeHandle n("~");


    // load ros parameters from node handle
    config.loadRosParamFromNodeHandle(n);

    // setup dynamic reconfigure
    dynamic_recfg = boost::make_shared <dynamic_reconfigure::Server
      <teb_local_planner::TebLocalPlannerReconfigureConfig> >(n);
    dynamic_reconfigure::Server
      <teb_local_planner::TebLocalPlannerReconfigureConfig>::CallbackType
      cb = boost::bind(CB_reconfigure, _1, _2);
    dynamic_recfg->setCallback(cb);

    // setup callbacks for optimisation
    srv_plan    = n.advertiseService("plan", service_plan);
    srv_replan  = n.advertiseService("replan", service_replan);

    sub_reset   = n.subscribe("reset", 1, CB_reset);
    sub_clear   = n.subscribe("clear", 1, CB_clear);
    sub_plan    = n.subscribe("plan", 1, CB_plan);
    sub_publish = n.subscribe("publish", 1, CB_publish);

    // setup callbacks for service-like call
    sub_request        = n.subscribe("request", 5, CB_request);
    sub_request_replan = n.subscribe("request_replan", 5, CB_request_replan);
    pub_respond        = n.advertise<teb_planner_pa_msgs::Respond>(
      "respond", 1);


    // setup callbacks for start and goal pose
    sub_setStart2d = n.subscribe("set_start2d", 1, CB_setStart2d);
    sub_setGoal2d  = n.subscribe("set_goal2d", 1, CB_setGoal2d);
    sub_setStart   = n.subscribe("set_start", 1, CB_setStart);
    sub_setGoal    = n.subscribe("set_goal", 1, CB_setGoal);

    // setup callbacks for initial plan
    sub_setInitialPlan = n.subscribe("set_initialplan", 1, CB_setInitialPlan);

    // setup callbacks for obstacles
    sub_clearObstacles = n.subscribe("clear_obstacles", 1, CB_clearObstacles);
    sub_setObstacles   = n.subscribe("set_obstacles"  , 1, CB_setObstacles);
    sub_addObstacles   = n.subscribe("add_obstacles"  , 1, CB_addObstacles);

    // setup callbacks for waypoints
    sub_clicked_points = n.subscribe("/clicked_point", 5, CB_clicked_point);

    sub_clearWaypoints = n.subscribe("clear_waypoints", 1, CB_clearWaypoints);
    sub_setWaypoints   = n.subscribe("set_waypoints"  , 1, CB_setWaypoints);
    sub_addWaypoints   = n.subscribe("add_waypoints"  , 1, CB_addWaypoints);

    // set robot velocities
    sub_setRobotStrtVelocity = n.subscribe("set_startvelocity", 1,
      CB_setRobotStartVelocity);


    // Setup visualization
    visual = teb_local_planner::TebVisualizationPaPtr(new
      teb_local_planner::TebVisualizationPa(n, config));

    // instantiate planner
    CB_reset(std_msgs::EmptyConstPtr(new std_msgs::Empty()));


    // init variables
    goal_pose = teb_local_planner::PoseSE2(1, 1, 1);

    ros::spin();

    return 0;
}



/***************************[dynamic reconfigure]******************************/
void CB_reconfigure(teb_local_planner::TebLocalPlannerReconfigureConfig&
  reconfig, uint32_t level)
  {
    config.reconfigure(reconfig);
  }



/***************************[services]*****************************************/
bool service_plan(teb_planner_pa_msgs::Plan::Request  &req,
  teb_planner_pa_msgs::Plan::Response &res)
  {
    CB_setStart(geometry_msgs::PoseConstPtr(
      new geometry_msgs::Pose(req.request.start)));
    CB_setGoal(geometry_msgs::PoseConstPtr(
      new geometry_msgs::Pose(req.request.goal )));
    CB_setInitialPlan(geometry_msgs::PoseArray::ConstPtr(
      new geometry_msgs::PoseArray(req.request.initial_plan)));

    CB_setWaypoints(nav_msgs::PathConstPtr(
      new nav_msgs::Path(req.request.waypoints)));

    CB_setObstacles(costmap_converter::ObstacleArrayMsgConstPtr(
      new costmap_converter::ObstacleArrayMsg(req.request.obstacles)));

    CB_setRobotStartVelocity(geometry_msgs::Twist(req.request.start_vel));

    CB_clear(std_msgs::EmptyConstPtr(new std_msgs::Empty()));
    CB_plan(std_msgs::EmptyConstPtr(new std_msgs::Empty()));

    teb_local_planner::TebOptimalPlannerPtr planner_as_optimal =
      boost::dynamic_pointer_cast<teb_local_planner::TebOptimalPlanner>
      (planner);
    teb_local_planner::HomotopyClassPlannerPtr planner_as_homotopy =
      boost::dynamic_pointer_cast<teb_local_planner::HomotopyClassPlanner>
      (planner);

    if (planner_as_homotopy)
    {
        // homotopy class only
        res.respond.tebs     = visual->msgTebContainer(
          planner_as_homotopy->getTrajectoryContainer());

        // use best teb
        planner_as_optimal = (planner_as_homotopy->bestTeb());
    }
    if (planner_as_optimal)
    {
        res.respond.path  = visual->msgLocalPlan(planner_as_optimal->teb());
        res.respond.poses = visual->msgLocalPoses(planner_as_optimal->teb());
        res.respond.feedback = visual->msgFeedbackMessage(
          *planner_as_optimal, obst_vector);
    }

    res.respond.waypoints       = visual->msgViaPoints(waypoints);
    res.respond.obstacles_point = visual->msgObstaclesPoints(obst_vector);
    res.respond.obstacles_line  = visual->msgObstaclesLines(obst_vector);
    res.respond.obstacles_poly  = visual->msgObstaclesPoly(obst_vector);

    return true;
}

// replan without changing previous input parameters
bool service_replan(teb_planner_pa_msgs::Plan::Request  &req, teb_planner_pa_msgs::Plan::Response &res)
{
    CB_plan(std_msgs::EmptyConstPtr(new std_msgs::Empty()));

    teb_local_planner::TebOptimalPlannerPtr planner_as_optimal =
      boost::dynamic_pointer_cast<teb_local_planner::TebOptimalPlanner>
      (planner);
    teb_local_planner::HomotopyClassPlannerPtr planner_as_homotopy =
      boost::dynamic_pointer_cast<teb_local_planner::HomotopyClassPlanner>
      (planner);

    if (planner_as_homotopy)
    {
        // homotopy class only
        res.respond.tebs     = visual->msgTebContainer(
          planner_as_homotopy->getTrajectoryContainer());

        // use best teb
        planner_as_optimal = (planner_as_homotopy->bestTeb());
    }
    if (planner_as_optimal)
    {
        res.respond.path  = visual->msgLocalPlan(planner_as_optimal->teb());
        res.respond.poses = visual->msgLocalPoses(planner_as_optimal->teb());
        res.respond.feedback = visual->msgFeedbackMessage(
          *planner_as_optimal, obst_vector);
    }

    res.respond.waypoints       = visual->msgViaPoints(waypoints);
    res.respond.obstacles_point = visual->msgObstaclesPoints(obst_vector);
    res.respond.obstacles_line  = visual->msgObstaclesLines(obst_vector);
    res.respond.obstacles_poly  = visual->msgObstaclesPoly(obst_vector);

    return true;
}

// service-like topics
void CB_request(const teb_planner_pa_msgs::RequestConstPtr& msg)
{
    teb_planner_pa_msgs::Plan::Request req;
    teb_planner_pa_msgs::Plan::Response res;

    req.request = *msg;

    if (service_plan(req, res))
    {
        pub_respond.publish(res.respond);
    }
}

void CB_request_replan(const std_msgs::EmptyConstPtr& msg)
{
    teb_planner_pa_msgs::Plan::Request req;
    teb_planner_pa_msgs::Plan::Response res;

    if (service_replan(req, res))
    {
        pub_respond.publish(res.respond);
    }
}



/***************************[topics - planner]*********************************/
void CB_reset(const std_msgs::EmptyConstPtr& msg)
{
    ros::NodeHandle n("~");

    // Setup robot shape model
    teb_local_planner::RobotFootprintModelPtr robot_model =
      teb_local_planner::TebLocalPlannerROS::getRobotFootprintFromParamServer(n);

    // initiate planner
    if (config.hcp.enable_homotopy_class_planning)
    {
        planner = teb_local_planner::PlannerInterfacePtr(new
          teb_local_planner::HomotopyClassPlanner(
          config, &obst_vector, robot_model, visual, &waypoints));
        ROS_INFO("Homotopy class planner called");
    }
    else
    {
        planner = teb_local_planner::PlannerInterfacePtr(new
          teb_local_planner::TebOptimalPlanner(
          config, &obst_vector, robot_model, visual, &waypoints));
        ROS_INFO("Optimal planner called");
    }
    ROS_INFO("Planner resettet.");
}

void CB_clear(const std_msgs::EmptyConstPtr& msg)
{
    planner->clearPlanner();
    ROS_INFO("Planner cleared");
}

void CB_plan(const std_msgs::EmptyConstPtr& msg)
{
    if (init_plan.size() == 0)
    {
        ROS_INFO("Planning using start and goal pose :)");
        planner->plan(start_pose, goal_pose, &start_vel,
          config.goal_tolerance.free_goal_vel);
    }
    else
    {
        ROS_INFO("Planning using initial plan (ignoring start & goal pose)");
        planner->plan(init_plan, &start_vel,
          config.goal_tolerance.free_goal_vel);
    }
    CB_publish(std_msgs::EmptyConstPtr(new std_msgs::Empty()));
}

void CB_publish(const std_msgs::EmptyConstPtr& msg)
{
    planner->visualize();
    visual->publishObstacles(obst_vector);
    visual->publishViaPoints(waypoints);
    if (init_plan.size() != 0)
    {
        visual->publishGlobalPlan(init_plan);
    }
}



/***************************[topics - start and goal]**************************/
void CB_setStart2d(const geometry_msgs::Pose2DConstPtr& pose_msg)
{
    teb_local_planner::PoseSE2 pose_se2(pose_msg->x, pose_msg->y,
      pose_msg->theta);
    geometry_msgs::PosePtr pose_ptr(new geometry_msgs::Pose);
    pose_se2.toPoseMsg(*pose_ptr);

    CB_setStart(pose_ptr);
}

void CB_setGoal2d(const geometry_msgs::Pose2DConstPtr& pose_msg)
{
    teb_local_planner::PoseSE2 pose_se2(pose_msg->x, pose_msg->y,
      pose_msg->theta);
    geometry_msgs::PosePtr pose_ptr(new geometry_msgs::Pose);
    pose_se2.toPoseMsg(*pose_ptr);

    CB_setGoal(pose_ptr);
}

void CB_setStart(const geometry_msgs::PoseConstPtr& pose_msg)
{
    teb_local_planner::PoseSE2 pose_se2(*pose_msg);

    start_pose = pose_se2;
    ROS_INFO_STREAM("Starting pose set (" << start_pose << ").");
}

void CB_setGoal(const geometry_msgs::PoseConstPtr& pose_msg)
{
    teb_local_planner::PoseSE2 pose_se2(*pose_msg);
    goal_pose = pose_se2;
    ROS_INFO_STREAM("Goal pose set (" << goal_pose << ").");
}

void CB_setInitialPlan(const geometry_msgs::PoseArray::ConstPtr& init_plan_msg)
{
    double dist;
    double timestamp{0.0};

    // Clear initial plan
    CB_clearInitialPlan(std_msgs::EmptyConstPtr(new std_msgs::Empty()));

    // Add the poses
    for (size_t i = 0; i < (init_plan_msg->poses.size()); i++)
    {
        init_plan.push_back(geometry_msgs::PoseStamped());
        init_plan[i].pose = init_plan_msg->poses[i];
    }

    if (init_plan_msg->poses.size() != 0)
    {
        // Add the timestamps

        // Set sec to zero through msg
        init_plan[0].header.stamp.sec = init_plan_msg->header.stamp.sec;

        // Set frame_id to odom through msg
        init_plan[0].header.frame_id = init_plan_msg->header.frame_id;

        for (size_t i = 1; i < (init_plan.size()); i++)
        {
            std::complex<double> posediff((init_plan[i].pose.position.x -
              init_plan[i-1].pose.position.x),
              (init_plan[i].pose.position.y - init_plan[i-1].pose.position.y));

            dist = std::sqrt(std::norm(posediff));
            timestamp += (dist / config.robot.max_vel_x);
            init_plan[i].header.stamp.sec = timestamp;
        }

        // update via-points container
        if (checkGlobalWaypoints())
            updateWaypointsContainer(init_plan,
              config.trajectory.global_plan_viapoint_sep);

        ROS_INFO_STREAM("Initial plan set (" << init_plan.size() << "x).");
    }
}

void CB_clearInitialPlan(const std_msgs::EmptyConstPtr& msg)
{
    init_plan.clear();
    ROS_INFO("Initial Plan cleared.");

    // also clear waypoints if necessary
    if (checkGlobalWaypoints())
    {
        waypoints.clear();
        ROS_INFO("Waypoints cleared.");
    }
}

void CB_setRobotStartVelocity(const geometry_msgs::Twist& vel_start)
{
    start_vel.linear.x = vel_start.linear.x;
    start_vel.linear.y = vel_start.linear.y;
    start_vel.angular.z = vel_start.angular.z;

    ROS_INFO_STREAM("Start velocity is set to (x_l: " << start_vel.linear.x
      << ", y_l: " << start_vel.linear.y
      << ", z_a: " << start_vel.angular.z << ").");
}



/***************************[topics - obstacles]*******************************/
void CB_setObstacles(const costmap_converter::ObstacleArrayMsg::ConstPtr&
  obst_msg)
{
    CB_clearObstacles(std_msgs::EmptyConstPtr(new std_msgs::Empty()));
    CB_addObstacles(obst_msg);
}

void CB_clearObstacles(const std_msgs::EmptyConstPtr& msg)
{
    obst_vector.clear();
    ROS_INFO("Obstacles cleared.");
}

void CB_addObstacles(const costmap_converter::ObstacleArrayMsg::ConstPtr&
  obst_msg)
{
    size_t size_before = obst_vector.size();

    // Add custom obstacles obtained via message
    // (assume that all obstacles coordinates are specified in the
    //  default planning frame)
    for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
    {
        size_t size_old = obst_vector.size();
        if (obst_msg->obstacles.at(i).polygon.points.size() == 1)
        {
            if (obst_msg->obstacles.at(i).radius == 0)
            {
                obst_vector.push_back(teb_local_planner::ObstaclePtr(new
                  teb_local_planner::PointObstacle(
                  obst_msg->obstacles.at(i).polygon.points.front().x,
                  obst_msg->obstacles.at(i).polygon.points.front().y)));
            }
            else
            {
                obst_vector.push_back(teb_local_planner::ObstaclePtr(new
                  teb_local_planner::CircularObstacle(
                  obst_msg->obstacles.at(i).polygon.points.front().x,
                  obst_msg->obstacles.at(i).polygon.points.front().y,
                  obst_msg->obstacles.at(i).radius)));
            }
        }
        else
        {
            teb_local_planner::PolygonObstacle* polyobst = new
              teb_local_planner::PolygonObstacle;
            for (size_t j=0; j < obst_msg->obstacles.at(i).polygon.points.size(); ++j)
            {
                polyobst->pushBackVertex(
                  obst_msg->obstacles.at(i).polygon.points[j].x,
                  obst_msg->obstacles.at(i).polygon.points[j].y);
            }
            polyobst->finalizePolygon();
            obst_vector.push_back(teb_local_planner::ObstaclePtr(polyobst));
        }

        if (size_old != obst_vector.size())
        {
            obst_vector.back()->setCentroidVelocity(
              obst_msg->obstacles.at(i).velocities,
              obst_msg->obstacles.at(i).orientation);
        }
    }

    size_t size_diff = obst_vector.size() - size_before;
    if (size_diff > 0)
    {
        ROS_INFO_STREAM("Obstacles added (" << size_diff << "x).");
    }
}



/***************************[topics - waypoints]*******************************/
void CB_clicked_point(const geometry_msgs::PointStampedConstPtr& point_msg)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point_msg->point.x;
    pose.pose.position.y = point_msg->point.y;

    nav_msgs::PathPtr path_ptr(new nav_msgs::Path);
    path_ptr->poses.push_back(pose);

    CB_addWaypoints(path_ptr);
}

void CB_setWaypoints(const nav_msgs::Path::ConstPtr& waypoints_msg)
{
    if (!infoCustomWaypoints())
        return;

    CB_clearWaypoints(std_msgs::EmptyConstPtr(new std_msgs::Empty()));
    CB_addWaypoints(waypoints_msg);
}

void CB_clearWaypoints(const std_msgs::EmptyConstPtr& msg)
{
    if (!infoCustomWaypoints())
        return;

    waypoints.clear();
    ROS_INFO("Waypoints cleared.");
}

void CB_addWaypoints(const nav_msgs::Path::ConstPtr& waypoints_msg)
{
    if (!infoCustomWaypoints())
        return;

    size_t size_before = waypoints.size();

    for (const geometry_msgs::PoseStamped& pose : waypoints_msg->poses)
    {
        waypoints.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    size_t size_diff = waypoints.size() - size_before;
    if (size_diff > 0)
    {
        ROS_INFO_STREAM("Waypoints added (" << size_diff << "x).");

        if (config.optim.weight_viapoint <= 0)
            ROS_WARN("Note: via-points are deactivated, "
              "since 'weight_via_point' <= 0");
    }
}

void updateWaypointsContainer(const std::vector<geometry_msgs::PoseStamped>&
  initial_plan, double min_separation)
{
    waypoints.clear();

    if (min_separation <= 0)
        return;

    std::size_t prev_idx = 0;
    for (std::size_t i=1; i < initial_plan.size(); ++i)
    // skip first one, since we do
    // not need any point before the first 'min_separation'
    {
        // check separation to the previous via-point inserted
        if (teb_local_planner::distance_points2d(
          initial_plan[prev_idx].pose.position,
          initial_plan[i].pose.position) < min_separation)
            continue;

        // add via-point
        waypoints.push_back(Eigen::Vector2d(
          initial_plan[i].pose.position.x, initial_plan[i].pose.position.y));
        prev_idx = i;
    }

    if (waypoints.size() > 0)
    {
        ROS_INFO_STREAM("Waypoints added (" << waypoints.size() << "x).");
    }
}

bool checkGlobalWaypoints()
{
    return config.trajectory.global_plan_viapoint_sep > 0;
}

bool infoCustomWaypoints()
{
    if (checkGlobalWaypoints())
    {
        ROS_WARN("Waypoints are automatically set by the global plan "
          "('global_plan_viapoint_sep' > 0). "
          "Ignoring added/set/cleared waypoints.");

        return false;
    }
    else
    {
        return true;
    }
}
