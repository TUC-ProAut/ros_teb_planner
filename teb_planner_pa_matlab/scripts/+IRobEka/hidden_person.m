%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% +IrobEka/hidden_person.m                                                    %
% ========================                                                    %
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
% Copyright (c) 2020-2021 TU Chemnitz                                         %
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

%% setup flags
if (~exist('flags', 'var'))
    flags = struct();
end
if (~isfield(flags, 'plot_matlab'))
    % disable continuous plotting in helper.replan_10_times()
    flags.plot_matlab = false;
end

%% need a base scenario
if (~exist('tebplan', 'var'))
    % based on scenario 1.b) by default
    IRobEka.one_CriticalCorner()
    tebplan.clearPolylineObstacles()
    %tebplan.clearCriticalCorners
end

%% set virtual person & publish it's start position
person = struct();
person.pos_init = [1,  4  ];
person.pos_seen = [1,  0.5];
person.vel = [0, -0.75];

person.ros = struct();
person.ros.topic_name = '/person';
person.ros.topic_type = 'geometry_msgs/PointStamped';
person.ros.pub = robotics.ros.Publisher( tebplan.getRosNode(), ...
  person.ros.topic_name, person.ros.topic_type);

person.ros.msg = rosmessage(person.ros.topic_type);
person.ros.msg.Header.FrameId = 'odom';

% publish initial position of person
person.ros.msg.Point.X = person.pos_init(1);
person.ros.msg.Point.Y = person.pos_init(2);
person.ros.pub.send(person.ros.msg)


%% optimize & store initial trajectory
helper.replan_10_times();

% store initial trajectory
init_path = struct();
init_path.poses = tebplan.getResultFeedback();

% prepare publication of trajectory
init_path.ros = struct();
init_path.ros.topic_name = '/path';
init_path.ros.topic_type = 'nav_msgs/Path';
init_path.ros.pub = robotics.ros.Publisher( tebplan.getRosNode(), ...
  init_path.ros.topic_name, init_path.ros.topic_type);

init_path.ros.msg = rosmessage(init_path.ros.topic_type);
init_path.ros.msg.Header.FrameId = 'odom';
for i = 1:(size(init_path.poses, 1) - 1)
    pose = rosmessage('geometry_msgs/PoseStamped');
    pose.Pose.Position.X = init_path.poses(i,1);
    pose.Pose.Position.Y = init_path.poses(i,2);
    init_path.ros.msg.Poses(i) = pose;
    init_path.ros.pub.send(init_path.ros.msg);
end

% end of initial path - good position for a break point ;-)
pause(1)


%% switch to second part - person is just recognized close to the corner
% publish original trajectory
init_path.ros.pub.send(init_path.ros.msg);

% publish new position of person
person.ros.msg.Point.X = person.pos_seen(1);
person.ros.msg.Point.Y = person.pos_seen(2);
person.ros.pub.send(person.ros.msg)

% add person as obstacle
tebplan.clearCircularObstacles();
tebplan.addCircularObstacle(person.pos_seen, person.vel, 0.25);


%% advance simulation
for i = 15:25

    % print current pose number
    fprintf('\nstarting at pose %d - ', i);

    % reinitialize teb plan
    tebplan.setInitialPlan(init_path.poses(i:end,1:3));
    vel = norm(diff(init_path.poses([i-1 , i+1], 1:2))) / ...
          diff(init_path.poses([i-1 , i+1], 4));
    tebplan.setStartVelocity(vel);

    % replan trajectory
    helper.replan_10_times();

    % end of optimization - good position for a break point ;-)
    pause(1)
end
