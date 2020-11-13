/******************************************************************************
*                                                                             *
* visualization_pa.h                                                          *
* ==================                                                          *
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
*       include/teb_local_planner/visualization.h                             *
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



#ifndef TEB_LOCAL_PLANNER_VISUALIZATION_PA_H
#define TEB_LOCAL_PLANNER_VISUALIZATION_PA_H

// teb stuff
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/timed_elastic_band.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/optimal_planner.h>

// std
#include <iterator>
#include <string>
#include <vector>

// messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <teb_local_planner/FeedbackMsg.h>

namespace teb_local_planner
{

class TebOptimalPlanner;   //!< Forward Declaration


/**
 * @class TebVisualization
 * @brief Visualize stuff from the teb_local_planner
 */
class TebVisualizationPa : public TebVisualization
{
public:
    /**
     * @brief Default constructor
     * @remarks do not forget to call initialize()
     */
    TebVisualizationPa();

    /**
     * @brief Constructor that initializes the class and registers topics
     * @param nh local ros::NodeHandle
     * @param cfg const reference to the TebConfig class for parameters
     */
    TebVisualizationPa(ros::NodeHandle& nh, const TebConfig& cfg);

    /** @name Convert to ros messages */
    //@{

    /**
     * @brief Convert local plan to a ros message.
     *
     * @param teb const reference to a Timed_Elastic_Band
     */
     nav_msgs::Path msgLocalPlan(const TimedElasticBand& teb) const;

    /**
     * @brief Convert pose sequence to a ros message.
     *
     * @param teb const reference to a Timed_Elastic_Band
     */
     geometry_msgs::PoseArray msgLocalPoses(const TimedElasticBand& teb) const;

    /**
     * @brief Convert obstacle positions (only points) to a ros message
     *
     * @param obstacles Obstacle container
     */
    visualization_msgs::Marker msgObstaclesPoints(const ObstContainer& obstacles) const;

    /**
     * @brief Convert obstacle positions (only points) to a ros message
     *
     * @param critical_corners Critical Corners container
     */
    visualization_msgs::Marker msgCriticalCorners(const ObstContainer& critical_corners) const;


    /**
     * @brief Convert obstacle positions (only lines) to a ros message
     *
     * @param obstacles Obstacle container
     */
    visualization_msgs::MarkerArray msgObstaclesLines(const ObstContainer& obstacles) const;


    /**
     * @brief Convert obstacle positions (only polygons) to a ros message
     *
     * @param obstacles Obstacle container
     */
    visualization_msgs::MarkerArray msgObstaclesPoly(const ObstContainer& obstacles) const;

    /**
     * @brief Convert via-points to a ros message
     *
     * @param via_points via-point container
     */
    visualization_msgs::Marker msgViaPoints(const std::vector< Eigen::Vector2d,
                                            Eigen::aligned_allocator<Eigen::Vector2d> >& via_points,
                                            const std::string& ns = "ViaPoints") const;

    /**
     * @brief Convert multiple Tebs from a container class to ros message
     *
     * @param teb_planner Container of boost::shared_ptr< TebOptPlannerPtr >
     * @param ns Namespace for the marker objects
     */
    visualization_msgs::Marker msgTebContainer(const TebOptPlannerContainer& teb_planner,
                                               const std::string& ns = "TebContainer");

    /**
     * @brief Convert teb to a feedback message (single trajectory)
     *
     * The feedback message contains the planned trajectory
     * that is composed of the sequence of poses, the velocity profile and temporal information.
     * The feedback message also contains a list of active obstacles.
     * @param teb_planner the planning instance
     * @param obstacles Container of obstacles
     */
    FeedbackMsg msgFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles);

    //@}
};

//! Abbrev. for shared instances of the TebVisualization
typedef boost::shared_ptr<TebVisualizationPa> TebVisualizationPaPtr;

//! Abbrev. for shared instances of the TebVisualization (read-only)
typedef boost::shared_ptr<const TebVisualizationPa> TebVisualizationPaConstPtr;


}   // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER_VISUALIZATION_PA_H
