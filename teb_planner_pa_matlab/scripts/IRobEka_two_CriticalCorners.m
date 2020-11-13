%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% IRobEka_two_CriticalCorners.m                                               %
% =============================                                               %
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
%   Peter Weissig, Kenny Schlegel                                             %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% New BSD License                                                             %
%                                                                             %
% Copyright (c) 2020 TU Chemnitz                                              %
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

% Example script for the irobeka project
%
% Please call   $ roslaunch teb_planner_pa irobeka.launch
% before running this script.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% instantiate TebPlanner class
tebplan = TebPlanner;


%% set initial plan for the robot
% (this is just a straight line along x-axis)
poses = zeros(9,3);
poses(1:9 , 1) = -5:3 ; poses(1:9 , 2) =  1  ;
tebplan.setInitialPlan(poses);


%% create enviroment
tebplan.clearPolygonObstacle();
tebplan.clearPolylineObstacle();

% two lanes of vertical shelfes
base_obstacle=[ 0  , 0  ;   3.0, 0  ;   3.0, 8.0;   0  , 8.0];
for dy = (-1:0) * 10.0 + 2.0
    for dx = (-1:1) * 5.0 - 4.0
        tebplan.addPolygonObstacle(base_obstacle + [dx, dy]);
    end
end

% remove unused variables
clear dx dy base_obstacle;


%% add critical corners
tebplan.clearCriticalCorners();
tebplan.addCriticalCorner([-1  , 2  ]);
tebplan.addCriticalCorner([-1  , 0  ]);


%% plan
tebplan.plan_using_topics();


%% publish the obstacles and robot footprint continuously
% get internal rosnode
rosnode = tebplan.getRosNode();
% create publisher and an empty message
pub = robotics.ros.Publisher(rosnode,'/teb_planner_node_pa/publish',...
    'std_msgs/Empty');
msg = rosmessage(pub);

% create a timer object
t_pub = timer;
% set publish rate to 4 Hz
t_pub.Period = 0.25;
t_pub.ExecutionMode = 'fixedRate';
% publish on each timer event
t_pub.TimerFcn = 'pub.send(msg)';
% start the timer
start(t_pub);

% stop the timer (optional)
%stop(t);

%% further testing (optional)
% a) do continuous replanning
t_plan = timer;
t_plan.Period        = 10;
t_plan.ExecutionMode = 'fixedRate';
t_plan.TimerFcn      = 'tebplan.plan_using_topics()';
start(t_plan);

% b) do continuous replanning
t_replan = timer;
t_replan.Period        = 1;
t_replan.ExecutionMode = 'fixedRate';
t_replan.TimerFcn      = 'tebplan.replan_using_topics()';
%start(t_replan);
