classdef CSystem < handle
    % Class for a multi-robot system

    properties

        %% setup
        dim_                                % dimension
        dt_                                 % delta t for simulation and control
        ws_                                 % workspace

        %% timer
        time_global_            =   0;
        time_step_global_       =   0;

        %% object
        nRobot_                 =   0;
        nBoxObs_                =   0;
        MultiRobot_             =   {};
        MultiBoxObs_            =   {};

        %% system current real state
        multi_robot_goal_real_  =   [];     % goal
        multi_robot_pos_real_   =   [];     % pos
        multi_obs_pos_real_     =   [];
        multi_obs_size_real_    =   [];
        multi_obs_yaw_real_     =   [];
        multi_obs_vert_real_    =   [];

        %% system current est state
        multi_robot_pos_est_    =   [];     % pos
        multi_robot_pos_est_cov_=   [];     % pos est cov
        multi_obs_pos_est_      =   [];
        multi_obs_pos_est_cov_  =   [];

        gpgo_pos_est_    =   [];     % pos
        gpgo_pos_est_cov_    =   [];     % pos
        
        dpgo_pose_est_           =   [];     % distributed PGO pose estimate
        dpgo_pose_est_cov_       =   [];     % distributed PGO pose covariance

        %% collision info
        collision_mtx_          =   [];

        %% method
        method_                 =   0;      % 0 - BVC; 1 - BUAVC
        control_                =   0;      % 0 - one-step; 1 - mpc

        meas_and_comm_range     =   5;      % measurement and communication range
        reset_clients={};
        clients =  {};
        global_client = []; % global client for pose graph optimization service
        global_reset_client=[];

        est_type_ = 0;

        comm_client = []; % global communication service client
    end


    methods

        %%  =======================================================================
        function obj = CSystem(nRobot, nBoxObs, pr, ros2_node)

            obj.dim_    =   pr.dim;
            obj.dt_     =   pr.dtSim;
            obj.ws_     =   pr.ws;

            obj.method_ =   pr.method;
            obj.control_=   pr.control;

            obj.time_global_        =   0;
            obj.time_step_global_   =   0;

            obj.nRobot_ =   nRobot;
            obj.nBoxObs_=   nBoxObs;

            obj.est_type_=pr.est_type;

            for iRobot = 1 : nRobot
                if pr.robot_type == 0
                    obj.MultiRobot_{iRobot} = CSingleInt(pr, iRobot);
                elseif pr.robot_type == 1
                    obj.MultiRobot_{iRobot} = CDiffDrive(pr, iRobot);
                else
                    error('Robot type not defined!')
                end
            end

            for jBoxObs = 1 : nBoxObs
                obj.MultiBoxObs_{jBoxObs} = CBoxObs(pr, jBoxObs);
            end

            % vector and matrix initialization
            obj.multi_robot_pos_real_ 	=   zeros(obj.dim_, nRobot);
            obj.multi_robot_goal_real_ 	=   zeros(obj.dim_, nRobot);
            obj.multi_obs_pos_real_   	=   zeros(obj.dim_, nBoxObs);
            obj.multi_obs_size_real_  	=   zeros(obj.dim_, nBoxObs);
            obj.multi_obs_yaw_real_  	=   zeros(1, nBoxObs);

            obj.multi_robot_pos_est_ 	=   zeros(obj.dim_, nRobot);
            obj.multi_robot_pos_est_cov_=   zeros(obj.dim_, obj.dim_, nRobot);
            obj.multi_obs_pos_est_      =   zeros(obj.dim_, nBoxObs);
            obj.multi_obs_pos_est_cov_  =   zeros(obj.dim_, obj.dim_, nBoxObs);

            obj.gpgo_pos_est_           =   zeros(obj.dim_, nRobot);
            obj.gpgo_pos_est_cov_       =   zeros(obj.dim_, obj.dim_, nRobot);

            obj.collision_mtx_          =   zeros(nRobot, nRobot+nBoxObs);

            obj.meas_and_comm_range     =  pr.meas_and_comm_range;

            serviceName = "pose_graph_optimization";
            serviceType = "raido_interfaces/PoseGraphOptimization";
            resetServiceType = "raido_interfaces/Reset";

            timestamp_str = datestr(now, 'yyyy-mm-dd_HH-MM-SS');

            for iRobot = 1 : nRobot
                disp('initializing')
                iRobot
                %% initialize robot ros2 service client
                robot_service_name  = sprintf('dpgo_server_%c/pose_graph_optimization', iRobot-1+'a');
                obj.clients{iRobot} = ros2svcclient(ros2_node, robot_service_name, serviceType);
                robot_reset_service_name  = sprintf('dpgo_server_%c/reset', iRobot-1+'a')
                obj.reset_clients{iRobot} = ros2svcclient(ros2_node, robot_reset_service_name, resetServiceType);
                % Call the reset service (std_srvs/Empty)
                reset_msg = ros2message(obj.reset_clients{iRobot});
                reset_msg.message = timestamp_str;
                call(obj.reset_clients{iRobot}, reset_msg);
            end

            global_service_name = 'dpgo_server_global/' + serviceName;
            obj.global_client = ros2svcclient(ros2_node, global_service_name, serviceType);
            global_reset_service_name = 'dpgo_server_global/reset';
            obj.global_reset_client = ros2svcclient(ros2_node, global_reset_service_name, resetServiceType);
            % Call the global reset service (std_srvs/Empty)
            global_reset_msg = ros2message(obj.global_reset_client);
            global_reset_msg.message = timestamp_str;
            call(obj.global_reset_client, global_reset_msg);
            disp('initialized global')

            % Initialize global communicate_robot service client (only once)
            comm_service_name = '/communicate_robot';
            comm_service_type = 'raido_interfaces/CommunicateRobot';
            obj.comm_client = ros2svcclient(ros2_node, comm_service_name, comm_service_type);
        end

        function [rel_pos, idx_in_range, rel_cov] = getRelativeRobotMeas(obj, iRobot)
            % Returns relative positions of robots within measurement/comm range of iRobot (using real positions)
            % rel_pos: dim_ x N matrix of relative positions
            % idx_in_range: indices of robots (other than iRobot) within range

            rel_pos = [];
            idx_in_range = [];
            rel_cov       = zeros(obj.dim_,    obj.dim_,  0); % M×M×0, so rel_cov(:,:,end+1)=… works
            pos_i = obj.multi_robot_pos_real_(:, iRobot);
            for j = 1:obj.nRobot_
                if j == iRobot
                    continue;
                end
                pos_j = obj.multi_robot_pos_real_(:, j);
                dist = norm(pos_j - pos_i);
                if dist <= obj.meas_and_comm_range
                    rel_pos = [rel_pos, pos_j - pos_i];
                    idx_in_range = [idx_in_range, j];
                    rel_cov(:,:,end+1)=obj.multi_robot_pos_est_cov_(:,:,j);
                end
            end
        end

        %%  =======================================================================
        function getSystemState(obj)
            % Obtain sytem state, real and est

            for iRobot = 1 : obj.nRobot_
                % real
                obj.multi_robot_pos_real_(:, iRobot)  = obj.MultiRobot_{iRobot}.pos_real_;
                obj.multi_robot_goal_real_(:, iRobot) = obj.MultiRobot_{iRobot}.goal_final_;
                % est
                obj.MultiRobot_{iRobot}.getEstimatedState();
                obj.multi_robot_pos_est_(:, iRobot) = obj.MultiRobot_{iRobot}.pos_est_;
                obj.multi_robot_pos_est_cov_(:,:,iRobot) = obj.MultiRobot_{iRobot}.pos_est_cov_;
            end

            for jBoxObs = 1 : obj.nBoxObs_
                % real
                obj.multi_obs_pos_real_(:, jBoxObs)   = obj.MultiBoxObs_{jBoxObs}.pos_real_;
                obj.multi_obs_size_real_(:, jBoxObs)  = obj.MultiBoxObs_{jBoxObs}.size_;
                obj.multi_obs_yaw_real_(:, jBoxObs)   = obj.MultiBoxObs_{jBoxObs}.yaw_;
                obj.multi_obs_vert_real_(:, :, jBoxObs) = box2PolyVertsCons(obj.dim_, ...
                    obj.multi_obs_pos_real_(:, jBoxObs), ...
                    obj.multi_obs_size_real_(:, jBoxObs), ...
                    obj.multi_obs_yaw_real_(:, jBoxObs));
                % est
                obj.MultiBoxObs_{jBoxObs}.getEstimatedObsState();
                obj.multi_obs_pos_est_(:, jBoxObs) = obj.MultiBoxObs_{jBoxObs}.pos_est_;
                obj.multi_obs_pos_est_cov_(:,:,jBoxObs) = obj.MultiBoxObs_{jBoxObs}.pos_est_cov_;
            end

        end


        %%  =======================================================================
        function simSystemOneStep(obj)
            % Simulate the system for one step, given current system state
            obj.time_step_global_ = obj.time_step_global_ + 1;

            % sim for each robot using current system state
            for iRobot = 1 : obj.nRobot_

                % ===== ego robot estimated state =====
                if obj.est_type_ == 0
                    obj.MultiRobot_{iRobot}.pos_est_ = obj.multi_robot_pos_est_(:, iRobot);
                    obj.MultiRobot_{iRobot}.pos_est_cov_ = obj.multi_robot_pos_est_cov_(:,:,iRobot);
                elseif obj.est_type_ == 1
                    obj.MultiRobot_{iRobot}.pos_est_ = obj.gpgo_pos_est_(:, iRobot);
                    obj.MultiRobot_{iRobot}.pos_est_cov_ = obj.gpgo_pos_est_cov_(:,:,iRobot);
                else
                    error("not implemented")
                end

                % ===== compute local bound =====
                obj.MultiRobot_{iRobot}.computeLocalBound();

                % ===== obtain local robot info =====
                if obj.est_type_ == 0
                  obj.MultiRobot_{iRobot}.getLocalRobotInfo(...
                    obj.multi_robot_pos_est_, obj.multi_robot_pos_est_cov_);
                elseif obj.est_type_ == 1
                  obj.MultiRobot_{iRobot}.getLocalRobotInfo(...
                    obj.gpgo_pos_est_, obj.gpgo_pos_est_cov_);
                end


                % ===== obtain local box obs info =====
                obj.MultiRobot_{iRobot}.getLocalObsInfo(...
                    obj.multi_obs_pos_est_, obj.multi_obs_pos_est_cov_, ...
                    obj.multi_obs_size_real_, obj.multi_obs_yaw_real_);


                % ===== deadlock checking =====
                obj.MultiRobot_{iRobot}.deadlockChecking();

                % ===== choose current goal for avoiding deadlock =====
                obj.MultiRobot_{iRobot}.chooseCurrentGoal();

                % ===== compute local safe convex region hyperplanes =====
                if obj.method_ == 0
                    obj.MultiRobot_{iRobot}.computeBVC();
                elseif obj.method_ == 1
                    obj.MultiRobot_{iRobot}.computeBUAVC();
                end

                % ===== compute control input =====
                if obj.control_ == 0
                    obj.MultiRobot_{iRobot}.computeControlInput();
                elseif obj.control_ == 1
                    obj.MultiRobot_{iRobot}.computeControlInputMPC();
                end

                % if arrived, set to 0
                if obj.MultiRobot_{iRobot}.isArrived_ == 1
                    obj.MultiRobot_{iRobot}.u_ = 0*obj.MultiRobot_{iRobot}.u_;
                end

                % if in collision, set to 0
                if obj.MultiRobot_{iRobot}.isCollision_ >= 1
                    obj.MultiRobot_{iRobot}.u_ = 0*obj.MultiRobot_{iRobot}.u_;
                    fprintf('Robot %d in collision!\n', iRobot);
                end

                % some heuristics
                %                 if norm(obj.MultiRobot_{iRobot}.u_) < 0.2
                %                     obj.MultiRobot_{iRobot}.u_ = 0*obj.MultiRobot_{iRobot}.u_;
                %                 end

                % ===== simulate the robot one step =====
                obj.MultiRobot_{iRobot}.simulateOneStep();

            end

        end


        %%  =======================================================================
        function collisionChecking(obj)
            % Check if collision happens in the system

            obj.collision_mtx_ = collision_check(obj.nRobot_, obj.nBoxObs_, ...
                obj.multi_robot_pos_real_, obj.MultiRobot_{1}.radius_, ...
                obj.multi_obs_vert_real_);

            for iRobot = 1 : obj.nRobot_
                if obj.MultiRobot_{iRobot}.isCollision_ == 0
                    obj.MultiRobot_{iRobot}.isCollision_ = ...
                        min(1, sum(obj.collision_mtx_(iRobot, :)));
                end
            end

        end


        function getLocalRobotEstimate(obj, iRobot)
            % Placeholder for getting local robot estimate for iRobot
            % To be implemented
        end

        function [delta_pos_all, odom_cov_all] = getRobotOdometryEstimate(obj)
            % Calls getOdometryStep for each robot and returns all odometry estimates
            % delta_pos_all: dim_ x nRobot_ matrix of position changes
            % odom_cov_all: dim_ x dim_ x nRobot_ array of odometry covariances

            delta_pos_all = cell(1, obj.nRobot_);
            odom_cov_all = cell(1, obj.nRobot_);
            for iRobot = 1:obj.nRobot_
                [delta_pos, odom_cov] = obj.MultiRobot_{iRobot}.getOdometryStep();
                delta_pos_all{iRobot} = delta_pos;
                odom_cov_all{iRobot} = odom_cov;
            end
        end

        function [rel_pos_all, rel_cov_all, idx_in_range_all, delta_pos_all, odom_cov_all] = getGlobalMeasurements(obj)
            % Collects all relative position estimates and odometry estimates for all robots
            % rel_pos_all: cell array, each cell contains dim_ x N matrix of relative positions for each robot
            % idx_in_range_all: cell array, each cell contains indices of robots in range for each robot
            % delta_pos_all: dim_ x nRobot_ matrix of odometry position changes
            % odom_cov_all: dim_ x dim_ x nRobot_ array of odometry covariances

            rel_pos_all = cell(1, obj.nRobot_);
            idx_in_range_all = cell(1, obj.nRobot_);
            rel_cov_all = cell(1, obj.nRobot_); % Not used in this implementation, but can be added if needed
            for iRobot = 1:obj.nRobot_
                [rel_pos, idx_in_range, rel_cov] = obj.getRelativeRobotMeas(iRobot);
                rel_pos_all{iRobot} = rel_pos;
                idx_in_range_all{iRobot} = idx_in_range;
                rel_cov_all{iRobot} = rel_cov; % Store relative covariances if needed
            end
            [delta_pos_all, odom_cov_all] = obj.getRobotOdometryEstimate();
        end

        function cov_mat = covToMat6(obj, cov)
            % Convert covariance vector to matrix
            % Assumes cov_vec is a vector of length 6 for 2D or 9 for 3D
            cov_mat = eye(6, 6)*0.001;
            if obj.dim_ == 2
                cov_mat(1:2, 1:2) = cov;
            elseif obj.dim_ == 3
                cov_mat(1:3, 1:3) = cov;
            end
        end

        function [optimized_poses, covs] = globalPGO(obj)
            % Collect global measurements and send to global dpgo_server
            [rel_pos_all, rel_cov_all,idx_in_range_all,  delta_pos_all, odom_cov_all] = obj.getGlobalMeasurements();
            nRobot = obj.nRobot_;
            % Prepare PoseGraphOptimization service message
            msg = ros2message(obj.global_client);
            msg.headers.stamp = ros2message('builtin_interfaces/Time');
            msg.headers.frame_id = 'map';
            msg.initial_poses = repmat(ros2message('geometry_msgs/Pose'), 1, nRobot);
            for i = 1:nRobot
                msg.initial_poses(i).position.x = obj.multi_robot_pos_est_(1,i);
                msg.initial_poses(i).position.y = obj.multi_robot_pos_est_(2,i);
                if obj.dim_ > 2
                    msg.initial_poses(i).position.z = obj.multi_robot_pos_est_(3,i);
                end
                % Set orientation to identity quaternion
                msg.initial_poses(i).orientation.w = 1;
                msg.initial_poses(i).orientation.x = 0;
                msg.initial_poses(i).orientation.y = 0;
                msg.initial_poses(i).orientation.z = 0;
                msg.initial_pose_keys(i) = ros2message('raido_interfaces/MultiRobotKey');
                msg.initial_pose_keys(i).robot = char(i+'a'-1);
                msg.initial_pose_keys(i).label = 'p';
                msg.initial_pose_keys(i).index = int32(obj.time_step_global_);
            end
            % Fill constraints (relative measurements)
            tmp = ros2message('raido_interfaces/MultiRobotKey');
            msg.keys1 = tmp([]);
            msg.keys2 = tmp([]);
            tmp = ros2message("geometry_msgs/PoseWithCovariance");
            msg.constraints = tmp([]);
            for i = 1:nRobot
                idxs = idx_in_range_all{i};
                rels = rel_pos_all{i};
                rel_cov = rel_cov_all{i};
                for k = 1:length(idxs)
                    j = idxs(k);
                    msg.keys1(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                    msg.keys1(end).robot = char(i+'a'-1);
                    msg.keys1(end).label = 'p';
                    msg.keys1(end).index = int32(obj.time_step_global_);
                    msg.keys2(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                    msg.keys2(end).robot = char(j+'a'-1);
                    msg.keys2(end).label = 'p';
                    msg.keys2(end).index = int32(obj.time_step_global_);
                    pose_cov = ros2message('geometry_msgs/PoseWithCovariance');
                    pose_cov.pose.position.x = rels(1,k);
                    pose_cov.pose.position.y = rels(2,k);
                    if obj.dim_ > 2
                        pose_cov.pose.position.z = rels(3,k);
                    else
                        pose_cov.pose.position.z = 0;
                    end
                    pose_cov.pose.orientation.w = 1;
                    pose_cov.pose.orientation.x = 0;
                    pose_cov.pose.orientation.y = 0;
                    pose_cov.pose.orientation.z = 0;
                    pose_cov.covariance = reshape(covToMat6(obj, rel_cov(:,:,k))', 1, 36);
                    msg.constraints(end+1) = pose_cov;
                end
            end

            % Add odometry constraints for each robot
            for i = 1:nRobot
                if obj.time_step_global_ > 0
                    delta_pos = delta_pos_all{i};
                    odom_cov = odom_cov_all{i};
                    % Odometry is between previous and current pose of the same robot
                    msg.keys1(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                    msg.keys1(end).robot = char(i+'a'-1);
                    msg.keys1(end).label = 'p';
                    msg.keys1(end).index = int32(obj.time_step_global_-1);
                    msg.keys2(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                    msg.keys2(end).robot = char(i+'a'-1);
                    msg.keys2(end).label = 'p';
                    msg.keys2(end).index = int32(obj.time_step_global_);
                    pose_cov = ros2message('geometry_msgs/PoseWithCovariance');
                    pose_cov.pose.position.x = delta_pos(1);
                    pose_cov.pose.position.y = delta_pos(2);
                    if obj.dim_ > 2
                        pose_cov.pose.position.z = delta_pos(3);
                    else
                        pose_cov.pose.position.z = 0;
                    end
                    pose_cov.pose.orientation.w = 1;
                    pose_cov.pose.orientation.x = 0;
                    pose_cov.pose.orientation.y = 0;
                    pose_cov.pose.orientation.z = 0;
                    % Fill covariance from odom_cov_all
                    pose_cov.covariance = reshape(covToMat6(obj, odom_cov)', 1, 36);
                    msg.constraints(end+1) = pose_cov;
                else
                    % add priors to robot 0 for global optimization
                    msg.keys1(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                    msg.keys1(end).robot = char(i+'a'-1);
                    msg.keys1(end).label = 'p';
                    msg.keys1(end).index = int32(0);
                    msg.keys2(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                    msg.keys2(end).robot = char(i+'a'-1);
                    msg.keys2(end).label = 'p';
                    msg.keys2(end).index = int32(0);
                    pose_cov = ros2message('geometry_msgs/PoseWithCovariance');
                    pose_cov.pose.position.x = obj.multi_robot_pos_est_(1,i);
                    pose_cov.pose.position.y = obj.multi_robot_pos_est_(2,i);
                    if obj.dim_ > 2
                        pose_cov.pose.position.z = obj.multi_robot_pos_est_(3,i);
                    else
                        pose_cov.pose.position.z = 0;
                    end
                    pose_cov.pose.orientation.w = 1;
                    pose_cov.pose.orientation.x = 0;
                    pose_cov.pose.orientation.y = 0;
                    pose_cov.pose.orientation.z = 0;
                    pose_cov.covariance = reshape(eye(6)*0.01,1,36); % small covariance for prior
                    msg.constraints(end+1) = pose_cov;
                end
            end


            % Call the global PGO service and extract optimized poses
            response = call(obj.global_client, msg);
            % Extract optimized poses from the response (geometry_msgs/PoseWithCovariance[])
            if isfield(response, 'poses') && ~isempty(response.poses)
                nOpt = numel(response.poses);
                optimized_poses = zeros(obj.dim_, nOpt);
                covs = zeros(obj.dim_, obj.dim_, nOpt);
                assert(nOpt == obj.nRobot_)
                for i = 1:nOpt
                    iRobot = response.robots(i)-'a'+1;
                    optimized_poses(1, iRobot) = response.poses(i).pose.position.x;
                    optimized_poses(2, iRobot) = response.poses(i).pose.position.y;
                    if obj.dim_ > 2
                        optimized_poses(3, iRobot) = response.poses(i).pose.position.z;
                    end

                    cov_vec = response.poses(i).covariance;
                    cov_mat = reshape(cov_vec, 6, 6)';

                    covs(:,:,iRobot)=cov_mat(1:obj.dim_, 1:obj.dim_);
                end
            else
                % print warning if no optimized poses returned
                warning('No optimized poses returned from global PGO service.');
                disp(response);

                optimized_poses = [];
                covs = [];
            end
        end

        function [optimized_poses, covs]=updateRobotEstCovFromGlobalPGO(obj)
            % Call globalPGO and update multi_robot_pos_est_ and multi_robot_pos_est_cov_
            [optimized_poses, covs] = obj.globalPGO();
            if ~isempty(optimized_poses)
                for iRobot = 1:obj.nRobot_
                    obj.gpgo_pos_est_(:, iRobot) = optimized_poses(:, iRobot);
                    obj.gpgo_pos_est_cov_(:, :, iRobot) = covs(:, :, iRobot);
                end
            else
                warning('No optimized poses returned in updateRobotEstCovFromGlobalPGO. State not updated.');
            end
        end

        function [opt_pose, cov] = distributedPGO(obj, iRobot)
            % Collect local measurements for iRobot and send to its dpgo_server
            % Returns optimized pose and covariance for iRobot

            % Get relative measurements for iRobot
            [rel_pos, idx_in_range, rel_cov] = obj.getRelativeRobotMeas(iRobot);
            % Get odometry for iRobot
            [delta_pos_all, odom_cov_all] = obj.getRobotOdometryEstimate();
            delta_pos = delta_pos_all{iRobot};
            odom_cov = odom_cov_all{iRobot};

            % Prepare PoseGraphOptimization service message
            msg = ros2message(obj.clients{iRobot});
            msg.headers.stamp = ros2message('builtin_interfaces/Time');
            msg.headers.frame_id = 'map';

            % Add initial pose for iRobot
            msg.initial_poses = ros2message('geometry_msgs/Pose');
            msg.initial_poses(1).position.x = obj.multi_robot_pos_est_(1,iRobot);
            msg.initial_poses(1).position.y = obj.multi_robot_pos_est_(2,iRobot);
            if obj.dim_ > 2
                msg.initial_poses(1).position.z = obj.multi_robot_pos_est_(3,iRobot);
            else
                msg.initial_poses(1).position.z = 0;
            end
            msg.initial_poses(1).orientation.w = 1;
            msg.initial_poses(1).orientation.x = 0;
            msg.initial_poses(1).orientation.y = 0;
            msg.initial_poses(1).orientation.z = 0;
            msg.initial_pose_keys = ros2message('raido_interfaces/MultiRobotKey');
            msg.initial_pose_keys(1).robot = char(iRobot+'a'-1);
            msg.initial_pose_keys(1).label = 'p';
            msg.initial_pose_keys(1).index = int32(obj.time_step_global_);

            % Add initial poses for observed neighbors
            for n = 1:length(idx_in_range)
                j = idx_in_range(n);
                idx_pose = n+1; % 1 is iRobot, next are neighbors
                msg.initial_poses(idx_pose).position.x = obj.multi_robot_pos_est_(1,j);
                msg.initial_poses(idx_pose).position.y = obj.multi_robot_pos_est_(2,j);
                if obj.dim_ > 2
                    msg.initial_poses(idx_pose).position.z = obj.multi_robot_pos_est_(3,j);
                else
                    msg.initial_poses(idx_pose).position.z = 0;
                end
                msg.initial_poses(idx_pose).orientation.w = 1;
                msg.initial_poses(idx_pose).orientation.x = 0;
                msg.initial_poses(idx_pose).orientation.y = 0;
                msg.initial_poses(idx_pose).orientation.z = 0;
                msg.initial_pose_keys(idx_pose) = ros2message('raido_interfaces/MultiRobotKey');
                msg.initial_pose_keys(idx_pose).robot = char(j+'a'-1);
                msg.initial_pose_keys(idx_pose).label = 'p';
                msg.initial_pose_keys(idx_pose).index = int32(obj.time_step_global_);
            end

            % Fill constraints (relative measurements)
            tmp = ros2message('raido_interfaces/MultiRobotKey');
            msg.keys1 = tmp([]);
            msg.keys2 = tmp([]);
            tmp = ros2message("geometry_msgs/PoseWithCovariance");
            msg.constraints = tmp([]);
            for k = 1:length(idx_in_range)
                j = idx_in_range(k);
                msg.keys1(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                msg.keys1(end).robot = char(iRobot+'a'-1);
                msg.keys1(end).label = 'p';
                msg.keys1(end).index = int32(obj.time_step_global_);
                msg.keys2(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                msg.keys2(end).robot = char(j+'a'-1);
                msg.keys2(end).label = 'p';
                msg.keys2(end).index = int32(obj.time_step_global_);
                pose_cov = ros2message('geometry_msgs/PoseWithCovariance');
                pose_cov.pose.position.x = rel_pos(1,k);
                pose_cov.pose.position.y = rel_pos(2,k);
                if obj.dim_ > 2
                    pose_cov.pose.position.z = rel_pos(3,k);
                else
                    pose_cov.pose.position.z = 0;
                end
                pose_cov.pose.orientation.w = 1;
                pose_cov.pose.orientation.x = 0;
                pose_cov.pose.orientation.y = 0;
                pose_cov.pose.orientation.z = 0;
                pose_cov.covariance = reshape(obj.covToMat6(rel_cov(:,:,k))', 1, 36);
                msg.constraints(end+1) = pose_cov;
            end

            % Add odometry constraint for iRobot
            if obj.time_step_global_ > 0
                msg.keys1(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                msg.keys1(end).robot = char(iRobot+'a'-1);
                msg.keys1(end).label = 'p';
                msg.keys1(end).index = int32(obj.time_step_global_-1);
                msg.keys2(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                msg.keys2(end).robot = char(iRobot+'a'-1);
                msg.keys2(end).label = 'p';
                msg.keys2(end).index = int32(obj.time_step_global_);
                pose_cov = ros2message('geometry_msgs/PoseWithCovariance');
                pose_cov.pose.position.x = delta_pos(1);
                pose_cov.pose.position.y = delta_pos(2);
                if obj.dim_ > 2
                    pose_cov.pose.position.z = delta_pos(3);
                else
                    pose_cov.pose.position.z = 0;
                end
                pose_cov.pose.orientation.w = 1;
                pose_cov.pose.orientation.x = 0;
                pose_cov.pose.orientation.y = 0;
                pose_cov.pose.orientation.z = 0;
                pose_cov.covariance = reshape(obj.covToMat6(odom_cov)', 1, 36);
                msg.constraints(end+1) = pose_cov;
            else
                % add prior for first step
                msg.keys1(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                msg.keys1(end).robot = char(iRobot+'a'-1);
                msg.keys1(end).label = 'p';
                msg.keys1(end).index = int32(0);
                msg.keys2(end+1) = ros2message('raido_interfaces/MultiRobotKey');
                msg.keys2(end).robot = char(iRobot+'a'-1);
                msg.keys2(end).label = 'p';
                msg.keys2(end).index = int32(0);
                pose_cov = ros2message('geometry_msgs/PoseWithCovariance');
                pose_cov.pose.position.x = obj.multi_robot_pos_est_(1,iRobot);
                pose_cov.pose.position.y = obj.multi_robot_pos_est_(2,iRobot);
                if obj.dim_ > 2
                    pose_cov.pose.position.z = obj.multi_robot_pos_est_(3,iRobot);
                else
                    pose_cov.pose.position.z = 0;
                end
                pose_cov.pose.orientation.w = 1;
                pose_cov.pose.orientation.x = 0;
                pose_cov.pose.orientation.y = 0;
                pose_cov.pose.orientation.z = 0;
                pose_cov.covariance = reshape(eye(6)*0.01,1,36);
                msg.constraints(end+1) = pose_cov;
            end

            % Call the distributed PGO service for iRobot
            response = call(obj.clients{iRobot}, msg);
            % Extract optimized pose from the response
            if isfield(response, 'poses') && ~isempty(response.poses)
                opt_pose = cell(obj.nRobot_, 1);
                cov = cell(obj.nRobot_, 1);
                for i = 1:length(response.robots)
                    robot = response.robots(i)-'a'+1;
                    pose = response.poses(i);
                    opt_pose{robot} = zeros(obj.dim_, 1);
                    opt_pose{robot}(1) = pose.pose.position.x;
                    opt_pose{robot}(2) = pose.pose.position.y;
                    if obj.dim_ > 2
                        opt_pose{robot}(3) = pose.pose.position.z;
                    end
                    cov_vec = pose.covariance;
                    cov_mat = reshape(cov_vec, 6, 6)';
                    cov{robot} = cov_mat(1:obj.dim_, 1:obj.dim_);
                end
            else
                warning('No optimized pose returned from distributed PGO service for robot %d.', iRobot);
                disp(response);
                opt_pose = [];
                cov = [];
            end

        end

        function communicateRobots(obj)
            % Generate robot pairs from relative measurements and call the communicate_robot service
            if isempty(obj.comm_client)
                return;
            end
            robot_pairs = [];
            for r = 1:obj.nRobot_
                [~, idx_in_range, ~] = obj.getRelativeRobotMeas(r);
                for k = 1:length(idx_in_range)
                    i = r;
                    j = idx_in_range(k);
                    if i < j
                        robot_pairs = [robot_pairs; i, j];
                    elseif j < i
                        robot_pairs = [robot_pairs; j, i];
                    end
                end
            end
            if isempty(robot_pairs)
                return;
            end
            robot_pairs = char(robot_pairs + 'a' - 1); % Convert to char for service message
            robot_pairs = reshape(robot_pairs', 1, []); % Convert to row vector
            comm_msg = ros2message(obj.comm_client);
            comm_msg.robot_pairs = robot_pairs; % Set the robot pairs
            comm_msg.rounds = int32(10);
            % Add any additional fields as required by the service definition
            comm_response = call(obj.comm_client, comm_msg);
        end

        function [dpgo_poses, dpgo_covs] = updateRobotEstCovFromDistPGO(obj)
            dpgo_poses = cell(obj.nRobot_, 1);
            dpgo_covs = cell(obj.nRobot_, 1);
            for iRobot = 1:obj.nRobot_
                [opt_pose, cov] = obj.distributedPGO(iRobot);
                if ~isempty(opt_pose) && ~isempty(cov) && ~isempty(opt_pose{iRobot}) && ~isempty(cov{iRobot})
                    obj.dpgo_pose_est_(:, iRobot) = opt_pose{iRobot};
                    obj.dpgo_pose_est_cov_(:, :, iRobot) = cov{iRobot};
                    dpgo_poses{iRobot} = opt_pose;
                    dpgo_covs{iRobot} = cov;
                else
                    warning('No optimized pose returned in updateRobotEstCovFromDistPGO for robot %d. State not updated.', iRobot);
                end
            end
        end

    end

end