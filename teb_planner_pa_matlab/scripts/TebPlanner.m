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

%Interface-class to access ROS TEB-Planner
classdef TebPlanner < handle

    properties
        maxTimeOut = 60; % maximum time for planning in seconds
    end

    properties (SetAccess = private)
        startPose           struct; %   [x,y, theta]
        goalPose            struct; %   [x,y, theta]
        initialPlan         struct; % n*[x,y, theta]
        startVelocity       struct; %   [vx,vy, omega]
        circularObstacles   struct; % n*[x,y, radius, vx,vy]
        polylineObstacles   struct; % n*[points=m*[x,y]  , vx,vy]
        polygonObstacles    struct; % n*[points=m*[x,y]  , vx,vy]
        waypoints           struct; % n*[x,y]
        hasNewResponse = false; % flag for new received msg
                 %   will be set   on response (service or topic)
                 %   will be reset on access   (e.g. full feedback, poses, ...)
        storeNextTopicResponse = false; % this flag can have different types
                 % 'never'   ... do never store next response
                 % false     ... do not   store next response
                 % true      ... store next response only
                 % 2,3,...   ... store next 2,3,... responses only
                 % 'always'  ... store all received responses
    end

    properties (Access = private)
        latestMsg = []; % latest received ros-msg
    end



    % constructor & destructor
    methods
        function obj = TebPlanner()
            %Construct an instance of this class

            % initialize start & goal pose
            obj.setStartPose();
            obj.setGoalPose(2, 0);
            obj.setStartVelocity();

            % initialize empty structs to show fields
            obj.clearInitialPlan();
            obj.clearWaypoints();
            obj.clearObstacles();
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
            obj.latestMsg      = response.Respond;
            obj.hasNewResponse = true;

            result = true;
        end

        function result = plan_using_topics(obj)

            % init result
            result = false;

            % enable reception of next message
            obj.incTopicResponseCount()

            % create publisher & the message
            pub = obj.getRosPublisher();
            % fill message with content
            msg = obj.getRequestMsg();
            % trigger planning
            pub.send(msg);

            % wait for response, if result is requested
            if (nargout > 0)
                result = obj.waitForNewResponse();
            end
        end

        % do the re-planning
        function result = replan(obj)

            % init result and internal variables
            result = false;
            obj.latestMsg = [];
            client = obj.getRosReplanService(obj.maxTimeOut);

            % fill message with no content
            request = rosmessage(client);

            % call the service
            response = call(client,request,'Timeout', obj.maxTimeOut);

            % check result
            if (isempty(response))
                return
            end

            % store response
            obj.latestMsg      = response.Respond;
            obj.hasNewResponse = true;

            result = true;
        end

        function result = replan_using_topics(obj)

            % init result
            result = false;

            % enable reception of next message
            obj.incTopicResponseCount()

            % create publisher & the message
            pub = obj.getRosReplanPublisher();
            % fill message with content
            msg = rosmessage('std_msgs/Empty');
            % trigger planning
            pub.send(msg);

            % wait for response, if result is requested
            if (nargout > 0)
                result = obj.waitForNewResponse();
            end
        end

        % change response mode
        function setTopicResponseMode(obj, mode)

            % check parameters
            if (nargin < 2)
                mode = true; % one-time response
            end

            % check if variable is a number
            if (isnumeric(mode))
                if ((mode < -2) || (mode == 0))
                    mode = false;
                elseif (mode == -2)
                    mode = 'never';
                elseif (mode == -1)
                    mode = 'always';
                elseif (mode == 1)
                    mode = true;
                end

            elseif (~islogical(mode) && ...
              ~strcmp(mode, 'always') && ~strcmp(mode, 'never'))
                warning('unknown value(-type) for mode %s', mode)
                return
            end

            % store current value
            obj.storeNextTopicResponse = mode;

            % init subscriber, if necessary
            sub = obj.getRosSubscriber();

            % check if response is enabled
            if (~islogical(mode) || mode)
                if (isempty(sub.NewMessageFcn))
                    sub.NewMessageFcn = @(src, msg) obj.rosCallback(src,msg);
                end
            else
                if (~isempty(sub.NewMessageFcn))
                    sub.NewMessageFcn = [];
                end
            end
        end
    end


    methods (Access = private)
        function count = getTopicResponseCount(obj)

            % init result
            count = 0;

            % get current value
            value = obj.storeNextTopicResponse;

            % check if variable is bool
            if (islogical(value))
                if (value)
                    count = 1; % one-time response
                else
                    count = 0; % always off
                end
                return
            end

            % check if variable is equal to 'always' or 'never'
            if (strcmp(value, 'never'))
                count = -2; % never
                return
            end
            if (strcmp(value, 'always'))
                count = -1; % always on
                return
            end

            % check if variable is a number
            if (isnumeric(value))
                count = value;
                return
            end
        end

        function incTopicResponseCount(obj)

            % load current response count
            count = obj.getTopicResponseCount();

            % increment count
            if (count >= 0)
                count = count + 1;
            end

            % store current count (activates/deactivates the receiver)
            obj.setTopicResponseMode(count)
        end

        function active = decTopicResponseCount(obj)

            % init result
            active = false;

            % load current response count
            count = obj.getTopicResponseCount();

            % deactivate response & return, if disabled
            if ((count == 0) || (count <= -2))
                obj.setTopicResponseMode(false)
                return
            end

            % decrement count
            if (count > 0)
                count = count - 1;
            end

            % store current count (activates/deactivates the receiver)
            obj.setTopicResponseMode(count)

            active = true;
        end

        function rosCallback(obj, ~, data)

            if (obj.decTopicResponseCount())
                % store message
                obj.latestMsg      = data;
                obj.hasNewResponse = true;
            end
        end
    end


    % data required for TEB-Planning
    methods

        % Sets the starting pose for the teb-planner
        %   (theta is measured in radians)
        function pose = setStartPose(obj, x, y, theta)

            if (nargin < 4); theta = 0; end
            if (nargin < 3); y     = 0; end
            if (nargin < 2); x     = 0; end
            pose = struct( ...
              'x'    , x(1), ...
              'y'    , y(1), ...
              'theta', theta(1));
            obj.startPose = pose;
        end

        % Gets the starting pose for the teb-planner
        function out = getStartPose(obj)

            out = obj.startPose;
        end

        % Sets the goal pose for the teb-planner
        %   (theta is measured in radians)
        function pose = setGoalPose(obj, x, y, theta)

            if (nargin < 4); theta = 0; end
            pose = struct( ...
              'x'    , x(1), ...
              'y'    , y(1), ...
              'theta', theta(1));
            obj.goalPose = pose;
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
        function velocity = setStartVelocity(obj, vx, vy, omega)

            if (nargin < 4); omega = 0; end
            if (nargin < 3); vy    = 0; end
            if (nargin < 2); vx    = 0; end
            velocity = struct( ...
              'vx'   , vx(1), ...
              'vy'   , vy(1), ...
              'omega', omega(1));
            obj.startVelocity = velocity;
        end

        % Get start velocity of the robot
        function out = getStartVelocity(obj)

            out = obj.startVelocity;
        end



        % Add initial plan
        % Function Parameters:
        %     poses  - [x1,y1,theta1; x2,y2,theta2; ...; xn,yn,thetan]
        function initial_plan = setInitialPlan(obj, poses)

            if (size(poses, 2) < 3); poses(1,3) = 0; end

            obj.clearInitialPlan();
            initial_plan = obj.initialPlan;
            for i = 1:size(poses, 1)
                initial_plan(i).x     = poses(i,1);
                initial_plan(i).y     = poses(i,2);
                initial_plan(i).theta = poses(i,3);
            end

            obj.initialPlan = initial_plan;
        end

        % Get the initial plan added for the robot
        function out = getInitialPlan(obj)

            out = obj.initialPlan;
        end

        % Delete initial plan
        function clearInitialPlan(obj)

            obj.initialPlan = struct( ...
              'x'    , {}, ...
              'y'    , {}, ...
              'theta', {});
        end



        % Adds circular obstacle
        % Function Parameters:
        %     position   - [x,y] position of the circular obstacle
        %     velocity   - [vx,vy] components of obstacle velocity
        %     radius     - radius of circular obstacle
        function obstacle = addCircularObstacle(obj, ...
          position, velocity, radius)

            if (nargin < 4); radius   = 0; end
            if (nargin < 3); velocity = [0, 0]; end % vx, vy
            if (length(velocity) < 2); velocity(2) = 0; end
            if (length(position) < 2); position(2) = 0; end

            obstacle = struct( ...
              'x'      , position(1), ...
              'y'      , position(2), ...
              'radius' , radius(1), ...
              'vx'     , velocity(1), ...
              'vy'     , velocity(2));

            if (isempty(obj.circularObstacles))
                obj.circularObstacles = obstacle;
            else
                obj.circularObstacles(end + 1) = obstacle;
            end
        end

        % Gets the circular obstacles present in the scenario
        function out = getCircularObstacles(obj)

            out = obj.circularObstacles;
        end

        % Deletes all circular obstacles
        function clearCircularObstacles(obj)

            obj.circularObstacles = struct( ...
              'x'      , {}, ...
              'y'      , {}, ...
              'radius' , {}, ...
              'vx'     , {}, ...
              'vy'     , {});
        end

        % Adds polyline obstacles;
        % Function Parameters:
        %    points - should contain min. two 2-D points
        %                for generating polyline
        %                e.g. [x1,y1; x2,y2; x3,y3]
        %    velocity   - [vx,vy] components of obstacle velocity
        function obstacle = addPolylineObstacle(obj, points, velocity)

            if (nargin < 3);  velocity = [0,0]; end
            if (length(velocity) < 2); velocity(2) = 0; end
            assert(size(points,1) >= 2, ...
              'PolylineObstacle need at least 2 points');

            obstacle = struct( ...
              'points', struct('x', {}, 'y', {}), ...
              'vx'    , velocity(1), ...
              'vy'    , velocity(2));
            for i = 1:size(points, 1)
                obstacle.points(i).x = points(i,1);
                obstacle.points(i).y = points(i,2);
            end

            if (isempty(obj.polylineObstacles))
                obj.polylineObstacles = obstacle;
            else
                obj.polylineObstacles(end + 1) = obstacle;
            end
        end

        % Gets the polyline obstacles present in the scenario
        function out = getPolylineObstacles(obj)

            out = obj.polylineObstacles;
        end

        % Deletes all polyline obstacles
        function clearPolylineObstacles(obj)

            obj.polylineObstacles = struct( ...
              'points', {}, ...
              'vx'    , {}, ...
              'vy'    , {});
        end

        % Adds polygon obstacles;
        % Function Parameters:
        %    points - should contain min. three 2-D points
        %                for generating a polygon
        %                e.g. [x1,y1; x2,y2; x3,y3; x4,y4]
        %    velocity   - [vx,vy] components of obstacle velocity
        function obstacle = addPolygonObstacle(obj,points,velocity)

            if (nargin < 3);  velocity = [0,0]; end
            if (length(velocity) < 2); velocity(2) = 0; end
            assert(size(points,1) >= 3, ...
              'PolygonObstacle need at least 3 points');

            obstacle = struct( ...
              'points', struct('x', {}, 'y', {}), ...
              'vx'    , velocity(1), ...
              'vy'    , velocity(2));
            for i = 1:size(points, 1)
                obstacle.points(i).x = points(i,1);
                obstacle.points(i).y = points(i,2);
            end

            if (isempty(obj.polygonObstacles))
                obj.polygonObstacles = obstacle;
            else
                obj.polygonObstacles(end + 1) = obstacle;
            end
        end

        % Gets the polygon obstacles present in the scenario
        function out = getPolygonObstacles(obj)

            out = obj.polygonObstacles;
        end

        % Deletes all polygon obstacles
        function clearPolygonObstacles(obj)

            obj.polygonObstacles = struct( ...
              'points', {}, ...
              'vx'    , {}, ...
              'vy'    , {});
        end



        % Deletes all obstacles
        function clearObstacles(obj)

            obj.clearCircularObstacles();
            obj.clearPolylineObstacles();
            obj.clearPolygonObstacles();
        end



        % Adds one way-point to the trajectory
        function addWaypoint(obj, x, y)

            way_point = struct(...
               'x', x(1), ...
               'y', y(1));

            if (isempty(obj.waypoints))
                obj.waypoints = way_point;
            else
                obj.waypoints(end + 1) = way_point;
            end
        end

        % Gets all way-points added to generate trajectory
        function out = getWaypoints(obj)

            out = obj.waypoints;
        end

        % Delete all way-points
        function clearWaypoints(obj)

            obj.waypoints = struct(...
               'x', {}, ...
               'y', {});
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
            msg.Start.Position.X = obj.startPose.x;
            msg.Start.Position.Y = obj.startPose.y;
            temp = eul2quat([obj.startPose.theta, 0, 0]);
            msg.Start.Orientation.W = temp(1);
            msg.Start.Orientation.X = temp(2);
            msg.Start.Orientation.Y = temp(3);
            msg.Start.Orientation.Z = temp(4);

            % fill goal pose
            msg.Goal.Position.X = obj.goalPose.x;
            msg.Goal.Position.Y = obj.goalPose.y;
            temp = eul2quat([obj.goalPose.theta, 0, 0]);
            msg.Goal.Orientation.W = temp(1);
            msg.Goal.Orientation.X = temp(2);
            msg.Goal.Orientation.Y = temp(3);
            msg.Goal.Orientation.Z = temp(4);

            if (~isempty(obj.initialPlan))
                % fill initial plan
                for i = 1:length(obj.initialPlan)
                    pose_stamped = rosmessage('geometry_msgs/PoseStamped');
                    % set frame id
                    pose_stamped.Header.FrameId = 'odom';
                    % set point
                    pose_stamped.Pose.Position.X = obj.initialPlan(i).x;
                    pose_stamped.Pose.Position.Y = obj.initialPlan(i).y;
                    % set Orientation
                    temp = eul2quat([obj.initialPlan(i).theta, 0, 0]);
                    pose_stamped.Pose.Orientation.W = temp(1);
                    pose_stamped.Pose.Orientation.X = temp(2);
                    pose_stamped.Pose.Orientation.Y = temp(3);
                    pose_stamped.Pose.Orientation.Z = temp(4);

                    msg.InitialPlan.Poses(i) = pose_stamped;
                end
            end



            % set start velocity
            msg.StartVel.Linear.X  = obj.startVelocity.vx;
            msg.StartVel.Linear.Y  = obj.startVelocity.vy;
            msg.StartVel.Angular.Z = obj.startVelocity.omega;

            % Creating point or circular obstacles
            for i = 1:length(obj.circularObstacles)
                obst = rosmessage('costmap_converter/ObstacleMsg');
                obst.Radius = obj.circularObstacles(i).radius;

                % add single point
                point = rosmessage('geometry_msgs/Point32');
                point.X = obj.circularObstacles(i).x;
                point.Y = obj.circularObstacles(i).y;

                obst.Polygon.Points = point;

                % add velocity
                vel = rosmessage('geometry_msgs/TwistWithCovariance');
                vel.Twist.Linear.X = obj.circularObstacles(i).vx;
                vel.Twist.Linear.Y = obj.circularObstacles(i).vy;

                obst.Velocities = vel;

                % add obst to message
                msg.Obstacles.Obstacles(end + 1) = obst;
            end

            % Creating polyline obstacles
            for i = 1:length(obj.polylineObstacles)
                % add dimensions
                for j = 1:length(obj.polylineObstacles(i).points) - 1
                    polyl_obst = rosmessage('costmap_converter/ObstacleMsg');
                    % point 1
                    point1 = rosmessage('geometry_msgs/Point32');
                    point1.X = obj.polylineObstacles(i).points(j).x;
                    point1.Y = obj.polylineObstacles(i).points(j).y;

                    % point 2
                    point2 = rosmessage('geometry_msgs/Point32');
                    point2.X = obj.polylineObstacles(i).points(j+1).x;
                    point2.Y = obj.polylineObstacles(i).points(j+1).y;

                    % add points
                    polyl_obst.Polygon.Points    = point1;
                    polyl_obst.Polygon.Points(2) = point2;

                    % add message
                    msg.Obstacles.Obstacles(end + 1) = polyl_obst;
                end
            end


            % Creating polygon obstacles
            for i = 1:length(obj.polygonObstacles)
                polygon_obst = rosmessage('costmap_converter/ObstacleMsg');

                % add dimensions
                for j = 1:length(obj.polygonObstacles(i).points)
                    point(j) = rosmessage('geometry_msgs/Point32');

                    point(j).X = obj.polygonObstacles(i).points(j).x;
                    point(j).Y = obj.polygonObstacles(i).points(j).y;
                    polygon_obst.Polygon.Points(j) = point(j);

                end

                % add velocity
                vel = rosmessage('geometry_msgs/TwistWithCovariance');
                vel.Twist.Linear.X = obj.polygonObstacles(i).vx;
                vel.Twist.Linear.Y = obj.polygonObstacles(i).vy;

                polygon_obst.Velocities = vel;
                msg.Obstacles.Obstacles(end + 1) = polygon_obst;
            end

            % Add waypoint
            for i = 1:length(obj.waypoints)
                pose_stamped = rosmessage('geometry_msgs/PoseStamped');
                pose_stamped.Pose.Position.X  = obj.waypoints(i).x;
                pose_stamped.Pose.Position.Y  = obj.waypoints(i).y;
                msg.Waypoints.Poses(end + 1)= pose_stamped;
            end


            % return message
            out = msg;
        end

        % wait for response msg
        function result = waitForNewResponse(obj)

            % init result
            result = false;

            % otherwise, wait for response
            start_time = datetime();
            while true
                if (obj.hasNewResponse)
                    result = true;
                    return
                end
                count = obj.getTopicResponseCount();
                if ((count <= -2) || (count == 0))
                    result = false;
                    return
                end

                if (seconds(datetime() - start_time) >= obj.maxTimeOut)
                    warning('timeout using TEB-Planner');
                    return
                end

                drawnow('limitrate');
                pause(0.01);
            end
        end

        % get response msg
        function out = getResultMsg(obj)
            % reset internal flag, because latest msg will be accessed
            obj.hasNewResponse = false;
            % return latest message
            out = obj.latestMsg;
        end

        % get content of latest response msg(Poses)
        function out = getResultPoses(obj)
            % load latest message
            msg = obj.getResultMsg();
            % check if latest message is not empty
            if (isempty(msg))
                fprintf(['Can''t access poses - ', ...
                  'latest message is empty :-(']);
                return
            end
            % load poses
            p = msg.Poses.Poses;
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
        function [out, all] = getResultFeedback(obj, msg)

            % init result
            out = [];
            all = [];

            if ((nargin < 2) || isempty(msg))
                % load latest message
                msg = obj.getResultMsg();
                % check if latest message is not empty
                if (isempty(msg))
                    fprintf(['Can''t access feedback - ', ...
                      'latest message is empty :-(']);
                    return
                end
            end

            ts = msg.Feedback.Trajectories;
            ts_curr_index = 1 + ...
              msg.Feedback.SelectedTrajectoryIdx;

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
                    poses(i,3) = quat2angle(temp);

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

        function out = getRosReplanService(timeout)
            persistent replan_client;

            if (isempty(replan_client))
                service_name='/teb_planner_node_pa/replan';

                rosnode = TebPlanner.getRosNode();

                fprintf(['Creating ROS-service client for "', ...
                  service_name, '"\n']);

                if (nargin < 1)
                  replan_client = robotics.ros.ServiceClient(rosnode, ...
                    service_name);
                else
                  replan_client = robotics.ros.ServiceClient(rosnode, ...
                    service_name, 'Timeout', timeout);
                end
            end

            out = replan_client;
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

        function out = getRosReplanPublisher()
            persistent pub;

            if (isempty(pub))
                topic_name='/teb_planner_node_pa/request_replan';
                topic_type='std_msgs/Empty';

                rosnode = TebPlanner.getRosNode();

                fprintf(['Creating ROS-publisher for "', topic_name, ...
                  '"\n']);
                pub = robotics.ros.Publisher(rosnode, ...
                  topic_name, topic_type);
            end

            out = pub;
        end

    end
end
