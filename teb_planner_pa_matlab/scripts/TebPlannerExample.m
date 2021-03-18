%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% TebPlannerExample.m                                                         %
% ===================                                                         %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% Repository:                                                                 %
%   https://github.com/TUC-ProAut/ros_teb_planner                             %
%                                                                             %
% Chair of Automation Technology, Technische Universität Chemnitz             %
%   https://www.tu-chemnitz.de/etit/proaut                                    %
%                                                                             %
% Authors:                                                                    %
%   Bhakti Danve                                                              %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% New BSD License                                                             %
%                                                                             %
% Copyright (c) 2019-2021 TU Chemnitz                                         %
% All rights reserved.                                                        %
%                                                                             %
% Redistribution and use in source and binary forms, with or without          %
% modification, are permitted provided that the following conditions are met: %
%    * Redistributions of source code must retain the above copyright notice, %
%      this list of conditions and the following disclaimer.                  %
%    * Redistributions in binary form must reproduce the above copyright      %
%      notice, this list of conditions and the following disclaimer in the    %
%      documentation and/or other materials provided with the distribution.   %
%    * Neither the name of the copyright holder nor the names of its          %
%      contributors may be used to endorse or promote products derived from   %
%      this software without specific prior written permission.               %
%                                                                             %
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS         %
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   %
% TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  %
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR           %
% CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       %
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         %
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; %
% OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    %
% WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     %
% OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF      %
% ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                  %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Test script to use TebPlanner Class methods to send data to
% teb_planner_node_pa. Make sure to keep the node running before executing
% the test_TebPlanner script. Use polygonal robot footprint of
% polygonal_robot_params.yaml file for better results.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% instantiate TebPlanner class
tebplan = TebPlanner;

%% set initial plan for the robot
% first segment: straight line along x-axis
poses = zeros(10,3);
poses(1:7 ,1) = -3:3; poses(1:7 ,2) = -0.5; poses(1:6 ,3) =  0;
                                            poses(7   ,3) =  pi/4;
% second segment: straight line along y-axis
poses(8:10,1) =    3; poses(8:10,2) =  1:3; poses(8:10,3) =  pi/2;
% set initial plan
tebplan.setInitialPlan(poses);

%% set start velocity of the robot
% vx = 0.5 m/s; vy = 0
tebplan.setStartVelocity(0.5,0);

%% add boundaries as polyline obstacles
tebplan.addPolylineObstacle([-3, 1,0; 2, 1,0; 2,3,0]);
tebplan.addPolylineObstacle([-3,-1,0; 4,-1,0; 4,3,0]);

%% add pedestrian as point obstacle
tebplan.addCircularObstacle([1,-0.5])

%% add waypoint (optional)
%tebplan.addWaypoint(-1,2);

% fine tuning: increase the weight of the waypoint to make the plan
%              pass through it
%              (dynamic reconfigure: Optimization/weight_viapoint)

%% plan using services
tebplan.plan();

%% publish the obstacles and robot footprint continuously
% get internal rosnode
rosnode = tebplan.getRosNode();
% create publisher and an empty message
pub = robotics.ros.Publisher(rosnode,'/teb_planner_node_pa/publish',...
    'std_msgs/Empty');
msg = rosmessage(pub);

% create a timer object
t=timer;
% set publish rate to 4 Hz
t.Period = 0.25;
t.ExecutionMode = 'fixedRate';
% publish on each timer event
t.TimerFcn = 'pub.send(msg)';
% start the timer
start(t);

% stop the timer (optional)
%stop(t);

%% further testing (optional)
% a) do continuous replanning
%t.TimerFcn = 'tebplan.replan()';

% b) minimize distance to boundaries
%    (dynamic reconfigure: Obstacles/min_obstacle_dist)

% c) auto generate waypoints from initial trajectory
%    (dynamic reconfigure: ViaPoints/global_plan_viapoint_sep)
