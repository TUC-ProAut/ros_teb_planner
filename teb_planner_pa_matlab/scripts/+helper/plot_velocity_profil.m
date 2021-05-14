%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% +helper/plot_velocity_profil.m                                              %
% ==============================                                              %
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
% Copyright (c) 2021 TU Chemnitz                                              %
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


%% init matlab rosnode (if necessary)
% motiviated from here:
%  https://de.mathworks.com/matlabcentral/answers/370444#comment_846475
try
    [~] = rosnode('list');
catch exp    % Error from rosnode list
    rosinit
end

%% run as subfunction (to avoid polluting the workspace)
plot_vel__sub(tebplan.getResultFeedback(), tebplan.criticalCorners())


function plot_vel__sub(msg, cc)


    %% check msg
    if (isempty(msg))
        clf
        return
    end

    %% get data
    % const - rosinit is needed for rosparam to work
    try
        robot_radius = rosparam('get', '/teb_planner_node_pa/footprint_model/radius');
    catch exp
        robot_radius = 0.4;
    end

    robot_vmax   = rosparam('get', '/teb_planner_node_pa/max_vel_x');
    cc_dist      = rosparam('get', '/teb_planner_node_pa/critical_corner_dist');
    cc_vel       = rosparam('get', '/teb_planner_node_pa/critical_corner_vel');
    cc_dist_incl = rosparam('get', '/teb_planner_node_pa/critical_corner_inclusion_dist');

    % convert data
    if (nargin < 2)
        cc = [];
    end
    if (isstruct(cc))
        cc       = cell2mat({cc.x; cc.y})';
    end

    %% calculate
    % at the given pose
    % (which is a bit unfair - the velocity is calculated rel. to next pose)
    pos = msg(:,1:2);
    t   = msg(:,4  );
    vel = vecnorm(diff(pos), 2, 2) ./ diff(t);
    l   = [0; cumsum(vecnorm(diff(pos), 2, 2))];

    % center points
    % (this seems to be the right position for the velocities)
    pos_middle = (pos(1:(end-1), : )  + pos(2:end, : )) / 2;
    t_middle   = (  t(1:(end-1), : )  +   t(2:end, : )) / 2;
    l_middle   = (  l(1:(end-1), : )  +   l(2:end, : )) / 2;

    % distance between poses and CC
    % (also considering footprint of robot - simplified as radius)
    dist_cc = zeros(size(pos, 1), size(cc, 1));
    for icc = 1:size(cc, 1)
        dist_cc(:, icc) = vecnorm(pos - cc(icc, :), 2, 2) - robot_radius;
    end

    % use closest pose (current or next)
    %   (this is done within the preselection and the edge itself)
    temp        = dist_cc((1:end-1), :);
    temp(:,:,2) = dist_cc( 2:end   , :);
    dist_cc     = min(temp, [], 3);

    % only consider pose, if close enough
    %   (this is done within the preselection)
    mask = dist_cc < cc_dist_incl;
    dist_cc(~mask) = inf;

    % convert to scale
    %   (this is done within the edge itself)
    temp = dist_cc / cc_dist;
    % apply quadratic scaling is further away
    mask = temp > 1;
    temp(mask) = temp(mask) .^ 2;

    % convert to velocities
    temp = temp * cc_vel;
    % limit velocities to v_max
    mask = temp > robot_vmax;
    temp(mask) = robot_vmax;
    vmax = temp;

    % get min velocity for each pose
    vmax = min(vmax, [], 2);
    if (isempty(vmax))
        vmax = repmat(robot_vmax, length(l)-1);
    end

    %% plot results
    subplot(1,2,1)
    cla
    hold on
    plot(pos(:,1), pos(:,2), 'kx', 'DisplayName', 'reference points')
    plot(pos(:,1), pos(:,2), 'g-', 'DisplayName', 'path')
    if (~isempty(cc))
        plot(cc(:,1), cc(:,2), 'bo', 'MarkerSize', 5, ...
          'DisplayName', 'critical corners')
    end

    legend()
    xlabel('x in m')
    ylabel('y in m')
    title('path')
    axis equal


    subplot(2,2,2)
    cla
    hold on
    plot(l(1:(end-1)), diff(t), 'kx')
    plot(l_middle, diff(t), 'b-', 'LineWidth', 3)
    legend('@reference point', 'plotted at midpoints')
    xlabel('rel. position in m')
    ylabel('length of timeintervall in s')
    title('time intervalls')


    subplot(2,2,4)
    cla
    hold on
    plot(l(1:(end-1)), vel, 'kx')
    plot(l_middle, vel, 'b-', 'LineWidth', 3)
    plot(l(1:(end-1)), vmax, 'r-')
    legend('@reference point', 'plotted at midpoints', 'vmax_{circular robot}')
    xlabel('rel. position in m')
    ylabel('velocity in m/s')
    title('velocity profil')

    drawnow
end
