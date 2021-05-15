%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% +IrobEka/run_all.m                                                          %
% ==================                                                          %
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


%% bugfix and initialisation
helper.clear_workspace()
helper.bugfix_ros()



%% 1.a) base planner
str='scenario 1a - with outer wall - base teb-planner';
fprintf('\n%s\n', str); sgtitle(str);

IRobEka.one_CriticalCorner()
tebplan.clearCriticalCorners()

helper.replan_10_times()

%% 1.a) with cc
str='scenario 1a - with outer wall - considering critical corner';
fprintf('\n%s\n', str); sgtitle(str);

IRobEka.one_CriticalCorner()

helper.replan_10_times()



%% 1.b) base planner
str='scenario 1b - only inner wall - base teb-planner';
fprintf('\n%s\n', str); sgtitle(str);

IRobEka.one_CriticalCorner()
tebplan.clearCriticalCorners()
tebplan.clearPolylineObstacles()

helper.replan_10_times()

%% 1.b) with cc
str='scenario 1b - only inner wall - considering critical corner';
fprintf('\n%s\n', str); sgtitle(str);

IRobEka.one_CriticalCorner()
tebplan.clearPolylineObstacles()

helper.replan_10_times()



%% 2. base planner
str='scenario 2 - base teb-planner';
fprintf('\n%s\n', str); sgtitle(str);

IRobEka.two_CriticalCorners()
tebplan.clearCriticalCorners()

helper.replan_10_times()

%% 2. with cc
str='scenario 2 - considering critical corner';
fprintf('\n%s\n', str); sgtitle(str);

IRobEka.two_CriticalCorners()

helper.replan_10_times()



%% 3. base planner
str='hidden person - base teb-planner';
fprintf('\n%s\n', str);

flags.plot_matlab = false;

IRobEka.one_CriticalCorner()
tebplan.clearPolylineObstacles()
tebplan.clearCriticalCorners()

IRobEka.hidden_person()

%% 3. with cc
str='hidden person - considering critical corner';
fprintf('\n%s\n', str);

flags.plot_matlab = false;

IRobEka.one_CriticalCorner()
tebplan.clearPolylineObstacles()

IRobEka.hidden_person()
