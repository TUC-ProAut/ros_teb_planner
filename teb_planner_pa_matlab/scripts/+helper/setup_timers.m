%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% +helper/setup_timers.m                                                      %
% ======================                                                      %
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
%   Peter Weissig                                                             %
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


%% use flags to finetune script
if (~exist('flags', 'var'))
    flags = struct();
end
if (~isfield(flags, 'run_pub_timer'))
    flags.run_pub_timer = true;
end


%% initialize data structure
if (~exist('t', 'var') || ~isstruct(t))
    t = struct();
end

% create publisher & message for continuous
if (~isfield('t', 'pub'))
    if (~isfield('t', 'node'))
        if (~exist('tebplan', 'var'))
            tebplan = TebPlanner();
        end
        t.node = tebplan.getRosNode();
    end
    t.pub  = robotics.ros.Publisher(t.node, ...
      '/teb_planner_node_pa/publish', 'std_msgs/Empty');
end
if (~isfield('t', 'msg'))
    t.msg  = rosmessage(t.pub);
end

% publishing publish timer
if (~isfield('t', 't_pub'))
    t.t_pub = timer;
    t.t_pub.Period        = 1;
    t.t_pub.ExecutionMode = 'fixedRate';
    t.t_pub.TimerFcn      = 't.pub.send(t.msg)';

    if (flags.run_pub_timer)
        start(t.t_pub)
    end
end

% plan timer
if (~isfield('t', 't_plan'))
    t.t_plan = timer;
    t.t_plan.Period        = 20;
    t.t_plan.ExecutionMode = 'fixedRate';
    t.t_plan.TimerFcn      = 'tebplan.plan_using_topics();';
end

% replan timer
if (~isfield('t', 't_replan'))
    t.t_replan = timer;
    t.t_replan.Period        = 2;
    t.t_replan.ExecutionMode = 'fixedRate';
    t.t_replan.TimerFcn      = 'tebplan.replan_using_topics();';
end


%% start a certain timer
%start(t.t_pub);
%start(t.t_plan);
%start(t.t_replan);


%% stop all timers
%stop(timerfind())
