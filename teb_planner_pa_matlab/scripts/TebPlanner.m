%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% TebPlanner.m                                                                %
% ============                                                                %
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
%   Bhakti Danve, Peter Weissig                                               %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% New BSD License                                                             %
%                                                                             %
% Copyright (c) 2019-2020 TU Chemnitz                                         %
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

%Interface-class to access ROS TEB-Planner
classdef TebPlanner < handle

    properties
        maxTimeOut = 60; % maximum time for planning in seconds
    end

    properties (SetAccess = private)
        startPose           = [0, 0, 0]; % [x,y, theta]
        goalPose            = [2, 0, 0]; % [x,y, theta]
        initialPlan         = [];
        startVelocity       = [0, 0, 0]; % [vx,vy, omega]
        circularObstacle    struct; % [x,y, radius, vx,vy]
        polylineObstacle    struct;
        polygonObstacle     struct;
        waypoint            struct;

    end

    properties (Access = private)
        latestMsg = []; % latest received ros-msg
    end



    % constructor & destructor
    methods
        function obj = TebPlanner()
            %Construct an instance of this class

            % ... nothing todo ^^
        end
    end



    % planning
    methods
        % do the planning
        function result = plan(obj)

            % init result and internal variables
            result = false;
            obj.latestMsg = [];
            client = obj.getRosService(obj.maxTimeOut);

            % fill message with content
            request = rosmessage(client);
            request.Request = obj.getRequestMsg();

            % call the service
            response = call(client,request,'Timeout', obj.maxTimeOut);

            % check result
            if (isempty(response))
                return
            end

            % store response
            obj.latestMsg = response.Respond;
            result = true;
        end

        function result = plan_using_topics(obj)

            % init result and internal variables
            result = false;
            obj.latestMsg = [];

            % create publisher & subscriber (only if not created yet)
            pub = obj.getRosPublisher();
            sub = obj.getRosSubscriber();


            % fill message with content
            msg = obj.getRequestMsg();

            % trigger planning
            sub.NewMessageFcn = @(src, msg) obj.rosCallback(src,msg);
            pub.send(msg);

            start_time = datetime();
            while true
                if (~isempty(obj.latestMsg))
                    break
                end

                if (seconds(datetime() - start_time) >= obj.maxTimeOut)
                    warning('timeout using TEB-Planner');
                    return
                end

                drawnow();
                pause(0.01);
            end

            result = true;
        end
    end


    methods (Access = private)
        function rosCallback(obj, ~, data)
            % remove callback
            sub = obj.getRosSubscriber();
            sub.NewMessageFcn = [];

            % store message
            obj.latestMsg = data;
        end
    end



    % Plan Data collection required for TEB-Planning
    methods

        % Sets the starting pose for the teb-planner
        %   (theta is measured in radians)
        function setStartPose(obj, x, y, theta)

            if (nargin < 4); theta = 0; end
            if (nargin < 3); y     = 0; end
            if (nargin < 2); x     = 0; end
            obj.startPose = [x, y, theta];
        end

        % Gets the starting pose for the teb-planner
        function out = getStartPose(obj)

            out = obj.startPose;
        end

        % Sets the goal pose for the teb-planner
        %   (theta is measured in radians)
        function setGoalPose(obj, x, y, theta)

            if (nargin < 4); theta = 0; end

            obj.goalPose = [x, y, theta];
        end

        % Gets the goal pose for the teb-planner
        function out = getGoalPose(obj)

            out = obj.goalPose;
        end

        % Sets the start velocity for the teb-planner robot
        % Function Parameters:
        %     vx    - velocity component in x-direction
        %     vy    - velocity component in y-direction
        %     omega - angular velocity
        %             (measured in radians per second)
        function setStartVelocity(obj,vx,vy,omega)

            if (nargin < 4); omega = 0; end
            if (nargin < 3); vy    = 0; end
            if (nargin < 2); vx    = 0; end
            obj.startVelocity = [vx, vy, omega];
        end

        % Get start velocity of the robot
        function out = getStartVelocity(obj)

            out = obj.startVelocity;
        end

        % Add initial plan
        % Function Parameters:
        %     poses  - [x1,y1,theta1; x2,y2,theta2..; xn,yn,thetan]
        function setInitialPlan(obj,poses)

            if (nargin < 2)
                error('Initial plan not set correctly');
            elseif (length(poses) < 6)
                error('Initial plan should contain min. 6 poses');
            end
            obj.initialPlan = poses;
        end

        % Get the initial plan added for the robot
        function out = getInitialPlan(obj)

            out = obj.initialPlan;
        end

        % Adds circular obstacle
        % Function Parameters:
        %     position   - [x,y] position of the circular obstacle
        %     velocity   - [vx,vy] components of obstacle velocity
        %     radius     - radius of circular obstacle
        function addCircularObstacle(obj,position,velocity,radius)

            if (nargin < 4); radius   = 0; end
            if (nargin < 3); velocity = [0, 0]; end % vx, vy

            circular_obstacle = struct( ...
                'position', position,...
                'r'       , radius, ...
                'velocity', velocity);

            if (isempty(obj.circularObstacle))
                obj.circularObstacle = circular_obstacle;
            else
                obj.circularObstacle(end + 1) = circular_obstacle;
            end
        end

        % Gets the circular obstacles present in the scenario
        function out = getCircularObstacle(obj)

            out = obj.circularObstacle;
        end

        % Adds polyline obstacles;
        % Function Parameters:
        %    positions - should contain min. two 2-D positions
        %                for generating polyline
        %                e.g. [x1,y1; x2,y2; x3,y3]
        %    velocity   - [vx,vy] components of obstacle velocity
        function addPolylineObstacle(obj,positions,velocity)

            if (nargin < 3);  velocity = [0,0]; end

            polyline_obstacle = struct(...
                'positions', positions,...
                'velocity' , velocity);

            if (isempty(obj.polylineObstacle))
                obj.polylineObstacle = polyline_obstacle;
            else
                obj.polylineObstacle(end + 1) = polyline_obstacle;
            end
        end

        % Gets the polyline obstacles present in the scenario
        function out = getPolylineObstacle(obj)

            out = obj.polylineObstacle;
        end

        % Adds polygon obstacles;
        % Function Parameters:
        %    positions - should contain min. three 2-D positions
        %                for generating a polygon
        %                e.g. [x1,y1; x2,y2; x3,y3; x4,y4]
        %    velocity   - [vx,vy] components of obstacle velocity
        function addPolygonObstacle(obj,positions,velocity)

            if (nargin < 3);  velocity = [0,0]; end

            polygon_obstacle = struct(...
                'positions', positions,...
                'velocity'  , velocity);

            if (isempty(obj.polygonObstacle))
                obj.polygonObstacle = polygon_obstacle;
            else
                obj.polygonObstacle(end + 1) = polygon_obstacle;
            end
        end

        % Gets the polygon obstacles present in the scenario
        function out = getPolygonObstacle(obj)

            out = obj.polygonObstacle;
        end

        % Add way-points for the trajectory
        function addWaypoint(obj,x,y)

            way_point = struct(...
               'x', x, 'y', y);

            if (isempty(obj.waypoint))
                obj.waypoint = way_point;
            else
                obj.waypoint(end + 1) = way_point;
            end
        end

        % Get way-points added to generate trajectory
        function out = getWaypoint(obj)

            out = obj.waypoint;
        end

        % Delete all circular obstacles
        function clearCircularObstacle(obj)

            obj.circularObstacle = [];
        end

        % Delete all polyline obstacles
        function clearPolylineObstacle(obj)

            obj.polylineObstacle = [];
        end

        % Delete all polygon obstacles
        function clearPolygonObstacle(obj)

            obj.polygonObstacle = [];
        end

        % Delete all way-points
        function clearWaypoint(obj)

            obj.waypoint = [];
        end

        % Delete initial plan
        function clearInitialPlan(obj)

            obj.initialPlan = [];
        end

    end


    % access ros messages
    methods

        % get request msg
        function out = getRequestMsg(obj)

            % create request message
            topic_type = 'teb_planner_pa_msgs/Request';
            msg = rosmessage(topic_type);

            % fill starting pose
            msg.Start.Position.X = obj.startPose(1);
            msg.Start.Position.Y = obj.startPose(2);
            temp = eul2quat([obj.startPose(3), 0, 0]);
            msg.Start.Orientation.W = temp(1);
            msg.Start.Orientation.X = temp(2);
            msg.Start.Orientation.Y = temp(3);
            msg.Start.Orientation.Z = temp(4);

            % fill goal pose
            msg.Goal.Position.X = obj.goalPose(1);
            msg.Goal.Position.Y = obj.goalPose(2);
            temp = eul2quat([obj.goalPose(3), 0, 0]);
            msg.Goal.Orientation.W = temp(1);
            msg.Goal.Orientation.X = temp(2);
            msg.Goal.Orientation.Y = temp(3);
            msg.Goal.Orientation.Z = temp(4);

            if (~isempty(obj.initialPlan))
                % fill initial plan
                pose(1,length(obj.initialPlan)) = rosmessage...
                         ('geometry_msgs/Pose');

                % fill the initial pose frame to odom
                msg.InitialPlan.Header.FrameId = "odom";

                for i = 1:length(obj.initialPlan)
                    pose(i) = rosmessage('geometry_msgs/Pose');
                    pose(i).Position.X = obj.initialPlan(i,1);
                    pose(i).Position.Y = obj.initialPlan(i,2);

                    temp = eul2quat([obj.initialPlan(i,3), 0, 0]);
                    pose(i).Orientation.W = temp(1);
                    pose(i).Orientation.X = temp(2);
                    pose(i).Orientation.Y = temp(3);
                    pose(i).Orientation.Z = temp(4);

                    msg.InitialPlan.Poses(end+1) = pose(1,i);
                end
            end



            % fill intelligent actor start velocity
            msg.StartVel.Linear.X  = obj.startVelocity(1);
            msg.StartVel.Linear.Y  = obj.startVelocity(2);
            msg.StartVel.Angular.Z = obj.startVelocity(3);

            % Creating point or circular obstacles
            for i = 1:length(obj.circularObstacle)
                obst = rosmessage('costmap_converter/ObstacleMsg');
                obst.Radius = obj.circularObstacle(i).r;

                % add single point
                point = rosmessage('geometry_msgs/Point32');
                point.X = obj.circularObstacle(i).position(1,1);
                point.Y = obj.circularObstacle(i).position(1,2);

                obst.Polygon.Points = point;

                % add velocity
                vel = rosmessage('geometry_msgs/TwistWithCovariance');
                vel.Twist.Linear.X = obj.circularObstacle(i).velocity(1,1);
                vel.Twist.Linear.Y = obj.circularObstacle(i).velocity(1,2);

                obst.Velocities = vel;

                % add obst to message
                msg.Obstacles.Obstacles(end + 1) = obst;
            end


            % Creating polyline obstacles
            for i = 1:length(obj.polylineObstacle)
                % add dimensions
                for j = 1: size(obj.polylineObstacle(i).positions,1) - 1
                    polyl_obst = rosmessage('costmap_converter/ObstacleMsg');
                    % point 1
                    point1 = rosmessage('geometry_msgs/Point32');
                    point1.X = obj.polylineObstacle(i).positions(j,1);
                    point1.Y = obj.polylineObstacle(i).positions(j,2);

                    % point 2
                    point2 = rosmessage('geometry_msgs/Point32');
                    point2.X = obj.polylineObstacle(i).positions(j+1,1);
                    point2.Y = obj.polylineObstacle(i).positions(j+1,2);

                    % add points
                    polyl_obst.Polygon.Points    = point1;
                    polyl_obst.Polygon.Points(2) = point2;

                    % add message
                    msg.Obstacles.Obstacles(end + 1) = polyl_obst;
                end
            end


            % Creating polygon obstacles
            for i = 1:length(obj.polygonObstacle)
                polygon_obst = rosmessage('costmap_converter/ObstacleMsg');

                % add dimensions
                for j = 1: size(obj.polygonObstacle(i).positions,1)
                 point(j) = rosmessage('geometry_msgs/Point32');

                 point(j).X = obj.polygonObstacle(i).positions(j,1);
                 point(j).Y = obj.polygonObstacle(i).positions(j,2);
                 polygon_obst.Polygon.Points(j) = point(j);

                end

                % add velocity
                vel = rosmessage('geometry_msgs/TwistWithCovariance');
                vel.Twist.Linear.X = obj.polygonObstacle(i).velocity(1,1);
                vel.Twist.Linear.Y = obj.polygonObstacle(i).velocity(1,2);

                polygon_obst.Velocities = vel;
                msg.Obstacles.Obstacles(end + 1) = polygon_obst;
            end

            % Add waypoint
            for i = 1:length(obj.waypoint)
                point   = rosmessage('geometry_msgs/Point');
                point.X = obj.waypoint(i).x;
                point.Y = obj.waypoint(i).y;

                pose_stamped = rosmessage('geometry_msgs/PoseStamped');
                pose_stamped.Pose.Position  = point;
                msg.Waypoints.Poses(end + 1)= pose_stamped;
            end


            % return message
            out = msg;
        end

        % get response msg
        function out = getLatestResponse(obj)
            out = obj.latestMsg;
        end

        % get content of latest response msg(Poses)
        function out = getResultPoses(obj)
            if (isempty(obj.latestMsg))
                fprintf(['Can''t access poses - ', ...
                  'latest message is empty :-(']);
                return
            end
            p = obj.latestMsg.Poses.Poses;
            poses = zeros(length(p), 3);
            if (~isempty(p))
                for i = 1:length(p)
                    poses(i,1) = p(i).Position.X;
                    poses(i,2) = p(i).Position.Y;

                    temp = p(i).Orientation;
                    temp = [temp.W, temp.X, temp.Y, temp.Z];
                    poses(i,3) = quat2angle(temp);
                end
            end
            out = poses;
        end

        % get content of latest response msg(Feedback)
        function [out, all] = getResultFeedback(obj)
            if (isempty(obj.latestMsg))
                fprintf(['Can''t access feedback - ', ...
                  'latest message is empty :-(']);
                return
            end
            ts = obj.latestMsg.Feedback.Trajectories;
            ts_curr_index = 1 + ...
              obj.latestMsg.Feedback.SelectedTrajectoryIdx;

            all = cell(length(ts), 1);
            out = [];
            for ts_ind = 1:length(ts)
                t = ts(ts_ind).Trajectory;
                poses = zeros(length(t), 4);
                for i = 1:length(t)
                    poses(i,1) = t(i).Pose.Position.X;
                    poses(i,2) = t(i).Pose.Position.Y;

                    temp = t(i).Pose.Orientation;
                    temp = [temp.W, temp.X, temp.Y, temp.Z];
                    poses(i,3) = quat2angle(temp) / 180 * pi;

                    poses(i,4) = t(i).TimeFromStart.seconds;
                end
                all{ts_ind} = poses;

                if (ts_ind == ts_curr_index)
                    out = poses;
                end
            end
        end

    end



    % ros-interface
    methods (Static)
        function out = getRosNode()
            persistent rosnode;

            if (isempty(rosnode))
                node_name = 'rosnode_for_matlab';
                fprintf(['Creating ROS-node "', node_name, ...
                  '" for teb-planner\n']);
                rosnode = robotics.ros.Node(node_name);
            end

            out = rosnode;
        end

        function out = getRosService(timeout)
            persistent client;

            if (isempty(client))
                service_name='/teb_planner_node_pa/plan';
                %service_type='teb_planner_pa_msgs/Plan';

                rosnode = TebPlanner.getRosNode();

                fprintf(['Creating ROS-service client for "', ...
                  service_name, '"\n']);

                if (nargin < 1)
                  client = robotics.ros.ServiceClient(rosnode, ...
                    service_name);
                else
                  client = robotics.ros.ServiceClient(rosnode, ...
                    service_name, 'Timeout', timeout);
                end
            end

            out = client;
        end

        function out = getRosPublisher()
            persistent pub;

            if (isempty(pub))
                topic_name='/teb_planner_node_pa/request';
                topic_type='teb_planner_pa_msgs/Request';

                rosnode = TebPlanner.getRosNode();

                fprintf(['Creating ROS-publisher for "', topic_name, ...
                  '"\n']);
                pub = robotics.ros.Publisher(rosnode, ...
                  topic_name, topic_type);
            end

            out = pub;
        end

        function out = getRosSubscriber()
            persistent sub;

            if (isempty(sub))
                topic_name='/teb_planner_node_pa/respond';
                topic_type='teb_planner_pa_msgs/Respond';

                rosnode = TebPlanner.getRosNode();

                fprintf(['Creating ROS-subscriber for "', topic_name, ...
                  '"\n']);
                sub = robotics.ros.Subscriber(rosnode, ...
                  topic_name, topic_type);
            end

            out = sub;
        end

    end
end
