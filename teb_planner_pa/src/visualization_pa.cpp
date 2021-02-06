/******************************************************************************
*                                                                             *
* visualization_pa.cpp                                                        *
* ====================                                                        *
*                                                                             *
* This class is creating messages (for path, poses and markers) without       *
* sending those. It is very similar to the original class it is based on,     *
* except it only contains the modified functions it needs.                    *
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
*       src/visualization.cpp                                                 *
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

// ros headers
#include <ros/ros.h>



namespace teb_local_planner
{

TebVisualizationPa::TebVisualizationPa() : TebVisualization()
{
}

TebVisualizationPa::TebVisualizationPa(ros::NodeHandle& nh, const TebConfig& cfg) : TebVisualization(nh, cfg)
{
    initializePa(nh, cfg);
}

void TebVisualizationPa::initialize(ros::NodeHandle& nh, const TebConfig& cfg)
{
    if (initialized_)
    {
        ROS_WARN("TebVisualizationPA already initialized. Reinitalizing...");
        initialized_ = false;
    }

    // call base class function
    TebVisualization::initialize(nh, cfg);

    // call local extension
    initializePa(nh, cfg);
}

void TebVisualizationPa::initializePa(ros::NodeHandle& nh, const TebConfig& cfg)
{
    // register topics
    initial_plan_pub_ = nh.advertise<nav_msgs::Path>("initial_plan", 1);

    // unregister topics
    global_plan_pub_.shutdown();
}




nav_msgs::Path TebVisualizationPa::msgLocalPlan(const TimedElasticBand& teb) const
{
    // check initialisation
    if ( printErrorWhenNotInitialized() ) return nav_msgs::Path();

    // create path msg
    nav_msgs::Path teb_path;
    teb_path.header.frame_id = cfg_->map_frame;
    teb_path.header.stamp = ros::Time::now();

    // fill path msgs with teb configurations
    for (int i=0; i < teb.sizePoses(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = teb_path.header.frame_id;
        pose.header.stamp = teb_path.header.stamp;
        pose.pose.position.x = teb.Pose(i).x();
        pose.pose.position.y = teb.Pose(i).y();
        pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb.getSumOfTimeDiffsUpToIdx(i);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb.Pose(i).theta());
        teb_path.poses.push_back(pose);
    }

    return teb_path;
}



geometry_msgs::PoseArray TebVisualizationPa::msgLocalPoses(const TimedElasticBand& teb) const
{
    // check initialisation
    if ( printErrorWhenNotInitialized() ) return geometry_msgs::PoseArray();

    // get teb path
    nav_msgs::Path teb_path;
    teb_path = msgLocalPlan(teb);

    // create pose_array (along trajectory)
    geometry_msgs::PoseArray teb_poses;
    teb_poses.header.frame_id = teb_path.header.frame_id;
    teb_poses.header.stamp = teb_path.header.stamp;

    // fill path msgs with teb configurations
    for (int i=0; i < teb_path.poses.size(); i++)
    {
        teb_poses.poses.push_back(teb_path.poses[i].pose);
    }
    return teb_poses;
}



visualization_msgs::Marker TebVisualizationPa::msgObstaclesPoints(const ObstContainer& obstacles) const
{
    // check initialisation
    if ( printErrorWhenNotInitialized() ) return visualization_msgs::Marker();

    // Visualize point obstacles
    visualization_msgs::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "PointObstacles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(2.0);

    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
        boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(*obst);
        if (!pobst) continue;

        if (cfg_->hcp.visualize_with_time_as_z_axis_scale < 0.001)
        {
            geometry_msgs::Point point;
            point.x = pobst->x();
            point.y = pobst->y();
            point.z = 0;
            marker.points.push_back(point);
        }
    else
        {
            // Spatiotemporally point obstacles become a line
            marker.type = visualization_msgs::Marker::LINE_LIST;
            geometry_msgs::Point start;
            start.x = pobst->x();
            start.y = pobst->y();
            start.z = 0;
            marker.points.push_back(start);

            geometry_msgs::Point end;
            double t = 20;
            Eigen::Vector2d pred;
            pobst->predictCentroidConstantVelocity(t, pred);
            end.x = pred[0];
            end.y = pred[1];
            end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*t;
            marker.points.push_back(end);
        }
    }

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}



visualization_msgs::MarkerArray TebVisualizationPa::msgObstaclesLines(const ObstContainer& obstacles) const
{
    // check initialisation
    if ( printErrorWhenNotInitialized() ) return visualization_msgs::MarkerArray();

    // Visualize line obstacles
    visualization_msgs::MarkerArray marker_array;

    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
        boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);
        if (!pobst) continue;

        visualization_msgs::Marker marker;
        marker.header.frame_id = cfg_->map_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "LineObstacles";
        marker.id = idx++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(2.0);
        geometry_msgs::Point start;
        start.x = pobst->start().x();
        start.y = pobst->start().y();
        start.z = 0;
        marker.points.push_back(start);
        geometry_msgs::Point end;
        end.x = pobst->end().x();
        end.y = pobst->end().y();
        end.z = 0;
        marker.points.push_back(end);

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}



visualization_msgs::MarkerArray TebVisualizationPa::msgObstaclesPoly(const ObstContainer& obstacles) const
{
    // check initialisation
    if ( printErrorWhenNotInitialized() ) return visualization_msgs::MarkerArray();

    // Visualize polygon obstacles
    visualization_msgs::MarkerArray marker_array;

    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
        boost::shared_ptr<PolygonObstacle> pobst = boost::dynamic_pointer_cast<PolygonObstacle>(*obst);
        if (!pobst) continue;

        visualization_msgs::Marker marker;
        marker.header.frame_id = cfg_->map_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "PolyObstacles";
        marker.id = idx++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(2.0);

        for (Point2dContainer::const_iterator vertex = pobst->vertices().begin();
        vertex != pobst->vertices().end(); ++vertex)
        {
            geometry_msgs::Point point;
            point.x = vertex->x();
            point.y = vertex->y();
            point.z = 0;
            marker.points.push_back(point);
        }

        // Also add last point to close the polygon
        // but only if polygon has more than 2 points (it is not a line)
        if (pobst->vertices().size() > 2)
        {
            geometry_msgs::Point point;
            point.x = pobst->vertices().front().x();
            point.y = pobst->vertices().front().y();
            point.z = 0;
            marker.points.push_back(point);
        }
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}


visualization_msgs::Marker TebVisualizationPa::msgViaPoints(const std::vector< Eigen::Vector2d,
Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
{
    // check initialisation
    if ( printErrorWhenNotInitialized() ) return visualization_msgs::Marker();

    visualization_msgs::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(2.0);

    for (std::size_t i=0; i < via_points.size(); ++i)
    {
        geometry_msgs::Point point;
        point.x = via_points[i].x();
        point.y = via_points[i].y();
        point.z = 0;
        marker.points.push_back(point);
    }

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    return marker;
}



visualization_msgs::Marker TebVisualizationPa::msgTebContainer(const TebOptPlannerContainer& teb_planner,
const std::string& ns)
{
    // check initialisation
    if ( printErrorWhenNotInitialized() ) return visualization_msgs::Marker();

    visualization_msgs::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    // Iterate through teb pose sequence
    for (TebOptPlannerContainer::const_iterator it_teb = teb_planner.begin(); it_teb != teb_planner.end(); ++it_teb)
    {
        // iterate single poses
        PoseSequence::const_iterator it_pose = it_teb->get()->teb().poses().begin();
        TimeDiffSequence::const_iterator it_timediff = it_teb->get()->teb().timediffs().begin();
        PoseSequence::const_iterator it_pose_end = it_teb->get()->teb().poses().end();
        std::advance(it_pose_end, -1);   // since we are interested in line segments, reduce end iterator by one.
        double time = 0;

        while (it_pose != it_pose_end)
        {
            geometry_msgs::Point point_start;
            point_start.x = (*it_pose)->x();
            point_start.y = (*it_pose)->y();
            point_start.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*time;
            marker.points.push_back(point_start);

            time += (*it_timediff)->dt();

            geometry_msgs::Point point_end;
            point_end.x = (*boost::next(it_pose))->x();
            point_end.y = (*boost::next(it_pose))->y();
            point_end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*time;
            marker.points.push_back(point_end);
            ++it_pose;
            ++it_timediff;
        }
    }
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    return marker;
}



FeedbackMsg TebVisualizationPa::msgFeedbackMessage(const TebOptimalPlanner& teb_planner,
const ObstContainer& obstacles)
{
    // check initialisation
    if ( printErrorWhenNotInitialized() ) return FeedbackMsg();

    FeedbackMsg msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = cfg_->map_frame;
    msg.selected_trajectory_idx = 0;

    msg.trajectories.resize(1);
    msg.trajectories.front().header = msg.header;
    teb_planner.getFullTrajectory(msg.trajectories.front().trajectory);

    // add obstacles
    msg.obstacles_msg.obstacles.resize(obstacles.size());
    for (std::size_t i=0; i < obstacles.size(); ++i)
    {
        msg.obstacles_msg.header = msg.header;

        // copy polygon
        msg.obstacles_msg.obstacles[i].header = msg.header;
        obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

        // copy id
        msg.obstacles_msg.obstacles[i].id = i;   // TODO: we do not have any id stored yet

        // orientation
        // msg.obstacles_msg.obstacles[i].orientation =; // TODO

        // copy velocities
        obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
    }

    return msg;
}



void TebVisualizationPa::publishInitialPlan(const std::vector<geometry_msgs::PoseStamped>& initial_plan) const
{
    if ( printErrorWhenNotInitialized() ) return;
    base_local_planner::publishPlan(initial_plan, initial_plan_pub_);
}



}   // namespace teb_local_planner
