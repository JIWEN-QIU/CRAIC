#include<super_core/super_planner.h>

namespace super_planner {

    RET_CODE SuperPlanner::odomPlanTest(const Vec3f &pos,
                                        const Vec3f &vel,
                                        const Vec3f &goal_p) {
        viz_.ClearMarker(Visualizer::BACKUP_TRAJ_PUB);
        viz_.ClearMarker(Visualizer::EXP_TRAJ_PUB);
        viz_.ClearMarker(Visualizer::COMMIT_TRAJ_PUB);
        static int plan_from_rest_cnt = 0;
        std::lock_guard<std::mutex> guard(replan_lock_);
        if (robot_state_.rcv == false) {
            ROS_WARN(" -- [SUPER] in [PlanFromRest]: No odom, force return.");
            return FAILED;
        }
        gi_.goal_p = goal_p;
        vec_Vec3f viz_pts{goal_p, robot_state_.p};
        TimeConsuming t_viz("viz goal path", false);
        viz_.VisualizeGoalPath(viz_pts);
        time_consuming_[VISUALIZATION] += t_viz.stop();



        /// 1) First, shift the start_point to free space.
        Vec3f local_star_pt;
        robot_state_.p = pos;
        if (!shiftPointToNearestNoneOcc(robot_state_.p, local_star_pt)) {
            ROS_ERROR(" -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            return FAILED;
        }

        /// 2) Generate Exp traj
        ExpTrajInfo exp_traj_info;
        exp_traj_info.cur_vel = vel;
        BackTrajInfo back_traj_info;

        RET_CODE exp_ret_code = GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        if (exp_ret_code == FAILED) {
            ROS_WARN(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with %s.",
                     RET_CODE_str[exp_ret_code].c_str());
            return FAILED;
        } else {
            ROS_INFO(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        viz_.VisualizeExpTrajInColor(exp_traj_info.optimized_exp_traj);

        exp_traj_info.is_rest2rest = true;
        back_traj_info.ref_exp_traj = &exp_traj_info;
        back_traj_info.back_traj_start_TT = -1;
        RET_CODE back_ret_code = GenerateBackupTrajectory(back_traj_info);;

        if (back_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ROS_INFO(" -- [SUPER] in [PlanFromRest] GenerateBackupTrajectory SUCCESS.");
            }
            emer_stop = false;
            const std::lock_guard<std::mutex> lock(cmd_traj_info_.sample_traj_mutex_);
            exp_traj_info.optimized_exp_traj.start_WT = ros::Time::now().toSec();
            exp_traj_info.exp_yaw_traj.start_WT = exp_traj_info.optimized_exp_traj.start_WT;
            if (!exp_traj_info.optimized_exp_traj.
                    getPartialTrajectoryByTime(0, back_traj_info.back_traj_start_TT, cmd_traj_info_.pos_traj)) {
                ROS_ERROR(" -- [SUPER] in [PlanFromRest] getPartialTrajectoryByTime failed.");
                return FAILED;
            }
            cmd_traj_info_.pos_traj = cmd_traj_info_.pos_traj + back_traj_info.optimized_back_traj;

            if (!exp_traj_info.exp_yaw_traj.
                    getPartialTrajectoryByTime(0, back_traj_info.back_traj_start_TT, cmd_traj_info_.yaw_traj)) {
                ROS_ERROR(" -- [SUPER] in [PlanFromRest] getPartialTrajectoryByTime failed.");
                return FAILED;
            }
            cmd_traj_info_.yaw_traj =
                    cmd_traj_info_.yaw_traj + back_traj_info.back_yaw_traj;

            cmd_traj_info_.backup_traj_start_TT = back_traj_info.back_traj_start_TT;
            cmd_traj_info_.use_back_traj = true;

            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;
            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, cmd_traj_info_.backup_traj_start_TT);
            time_consuming_[VISUALIZATION] += t_viz.stop();
            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == type_utils::NO_NEED) {
            if (cfg_.print_log) {
                ROS_INFO(" -- [SUPER] in [PlanFromRest] GenerateBackupTrajectory Finish or NO_NEED.");
            }
            emer_stop = false;
            const std::lock_guard<std::mutex> lock(cmd_traj_info_.sample_traj_mutex_);
            exp_traj_info.optimized_exp_traj.start_WT = ros::Time::now().toSec();
            exp_traj_info.exp_yaw_traj.start_WT = exp_traj_info.optimized_exp_traj.start_WT;
            cmd_traj_info_.pos_traj = exp_traj_info.optimized_exp_traj;
            cmd_traj_info_.yaw_traj = exp_traj_info.exp_yaw_traj;

            cmd_traj_info_.backup_traj_start_TT = INFINITY;
            cmd_traj_info_.use_back_traj = false;
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;

            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, -1);
            time_consuming_[VISUALIZATION] += t_viz.stop();
            return SUCCESS;
        }
        ROS_WARN(" -- [SUPER] in [PlanFromRest] GenerateBackupTrajectory return [%s], force return",
                 RET_CODE_str[back_ret_code].c_str());
        return FAILED;
    }


    SuperPlanner::SuperPlanner(const ros::NodeHandle &nh, PlannerConfig &cfg) {
        nh_ = nh;
        cfg_ = cfg;
        viz_.Init(nh, cfg_);

        exp_traj_opt_.reset(new traj_opt::ExpTrajOpt(nh, cfg_.exp_traj_cfg));
        back_traj_opt_.reset(new traj_opt::BackupTrajOpt(nh, cfg_.back_traj_cfg));
        yaw_traj_opt_.reset(new traj_opt::YawTrajOpt(cfg_.yaw_dot_max));
        time_consuming_.resize(8);
        map_ptr_.reset(new ROGMap(nh_, cfg_.rog_map_cfg));
        astar_ptr_.reset(new Astar());
        astar_ptr_->initMap(nh_, map_ptr_);
        cg_ptr_.reset(new CorridorGenerator(nh, map_ptr_, cfg_.corridor_bound_dis, cfg_.corridor_line_max_length,
                                            cfg_.resolution, cfg_.rog_map_cfg.virtual_ground_height,
                                            cfg_.rog_map_cfg.virtual_ceil_height,
                                            cfg_.robot_r, cfg_.obs_skip_num, cfg_.iris_iter_num));
        cg_ptr_->SetLineNeighborList(cfg.seed_line_neighbour);
        robot_state_.rcv = false;
        planner_process_start_WT_ = ros::Time::now().toSec();

        /// this neighbor list is used for shift the start point.
        constexpr double neighbor_search_dis = 3.0;
        int neighbor_num = 2 * neighbor_search_dis / cfg_.resolution;
        for (int i = -neighbor_num; i <= neighbor_num; i++) {
            for (int j = -neighbor_num; j <= neighbor_num; j++) {
                for (int k = -neighbor_num; k <= neighbor_num; k++) {
                    Vec3f neighbor(i * cfg_.resolution, j * cfg_.resolution, k * cfg_.resolution);
                    if (neighbor.norm() < neighbor_search_dis) {
                        sphereical_neighbor_list_.push_back(neighbor);
                    }
                }
            }
        }
        std::sort(sphereical_neighbor_list_.begin(), sphereical_neighbor_list_.end(),
                  [](const Vec3f &a, const Vec3f &b) {
                      return a.norm() < b.norm();
                  });
        fov_checker_.reset(new FOVChecker(FOVType::OMNI,
                                          -1.0,
                                          -35.0,
                                          35.0));
        vec_Vec3i neighbor_list;
        const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);
        for (int i = -neighbor_step; i <= neighbor_step; i++) {
            for (int j = -neighbor_step; j <= neighbor_num; j++) {
                for (int k = -neighbor_step; k <= neighbor_step; k++) {
                    if (i == 0 && j == 0 && k == 0) {
                        continue;
                    }
                    if (i * i + j * j + k * k > neighbor_step * neighbor_step) {
                        continue;
                    }
                    neighbor_list.emplace_back(i, j, k);
                }
            }
        }
        astar_ptr_->setFineInfNeighbors(neighbor_list);
    }

    RET_CODE
    SuperPlanner::PlanFromRest(const Vec3f &goal_p,
                               const double &goal_yaw,
                               const bool &new_goal) {
        static int plan_from_rest_cnt = 0;
        std::lock_guard<std::mutex> guard(replan_lock_);
        if (robot_state_.rcv == false) {
            ROS_WARN(" -- [SUPER] in [PlanFromRest]: No odom, force return.");
            return FAILED;
        }
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        vec_Vec3f viz_pts{goal_p, robot_state_.p};
        TimeConsuming t_viz("viz goal path", false);
        viz_.VisualizeGoalPath(viz_pts);
        time_consuming_[VISUALIZATION] += t_viz.stop();



        /// 1) First, shift the start_point to free space.
        Vec3f local_star_pt;
        if (!shiftPointToNearestNoneOcc(robot_state_.p, local_star_pt)) {
            ROS_ERROR(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            return FAILED;
        }

        /// 2) Generate Exp traj
        ExpTrajInfo exp_traj_info;
        BackTrajInfo back_traj_info;

        RET_CODE exp_ret_code = GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        if (exp_ret_code == FAILED) {
            ROS_WARN(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with %s.",
                     RET_CODE_str[exp_ret_code].c_str());
            return FAILED;
        } else {
            ROS_INFO(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        viz_.VisualizeExpTrajInColor(exp_traj_info.optimized_exp_traj);

        exp_traj_info.is_rest2rest = true;
        back_traj_info.ref_exp_traj = &exp_traj_info;
        back_traj_info.back_traj_start_TT = -1;
        RET_CODE back_ret_code = GenerateBackupTrajectory(back_traj_info);;

        if (back_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ROS_INFO(" -- [SUPER] in [PlanFromRest] GenerateBackupTrajectory SUCCESS.");
            }
            emer_stop = false;
            const std::lock_guard<std::mutex> lock(cmd_traj_info_.sample_traj_mutex_);
            exp_traj_info.optimized_exp_traj.start_WT = ros::Time::now().toSec();
            exp_traj_info.exp_yaw_traj.start_WT = exp_traj_info.optimized_exp_traj.start_WT;
            if (!exp_traj_info.optimized_exp_traj.
                    getPartialTrajectoryByTime(0, back_traj_info.back_traj_start_TT, cmd_traj_info_.pos_traj)) {
                ROS_ERROR(" -- [SUPER] in [PlanFromRest] getPartialTrajectoryByTime failed.");
                return FAILED;
            }
            cmd_traj_info_.pos_traj = cmd_traj_info_.pos_traj + back_traj_info.optimized_back_traj;

            if (!exp_traj_info.exp_yaw_traj.
                    getPartialTrajectoryByTime(0, back_traj_info.back_traj_start_TT, cmd_traj_info_.yaw_traj)) {
                ROS_ERROR(" -- [SUPER] in [PlanFromRest] getPartialTrajectoryByTime failed.");
                return FAILED;
            }
            cmd_traj_info_.yaw_traj =
                    cmd_traj_info_.yaw_traj + back_traj_info.back_yaw_traj;

            cmd_traj_info_.backup_traj_start_TT = back_traj_info.back_traj_start_TT;
            cmd_traj_info_.use_back_traj = true;

            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;
            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, cmd_traj_info_.backup_traj_start_TT);
            time_consuming_[VISUALIZATION] += t_viz.stop();
            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == type_utils::NO_NEED) {
            if (cfg_.print_log) {
                ROS_INFO(" -- [SUPER] in [PlanFromRest] GenerateBackupTrajectory Finish or NO_NEED.");
            }
            emer_stop = false;
            const std::lock_guard<std::mutex> lock(cmd_traj_info_.sample_traj_mutex_);
            exp_traj_info.optimized_exp_traj.start_WT = ros::Time::now().toSec();
            exp_traj_info.exp_yaw_traj.start_WT = exp_traj_info.optimized_exp_traj.start_WT;
            cmd_traj_info_.pos_traj = exp_traj_info.optimized_exp_traj;
            cmd_traj_info_.yaw_traj = exp_traj_info.exp_yaw_traj;
   
            cmd_traj_info_.backup_traj_start_TT = INFINITY;
            cmd_traj_info_.use_back_traj = false;
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;

            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, -1);
            time_consuming_[VISUALIZATION] += t_viz.stop();
            return SUCCESS;
        }
        ROS_WARN(" -- [SUPER] in [PlanFromRest] GenerateBackupTrajectory return [%s], force return",
                 RET_CODE_str[back_ret_code].c_str());
        return FAILED;
    }


    RET_CODE
    SuperPlanner::ReplanOnce(const Vec3f &goal_p,
                             const double &goal_yaw,
                             const bool &new_goal) {
        benchmark_utils::TimeConsuming replan_total_t("ReplanOnce", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        // 3） 记录飞机当前位置（即replan开始的时间）
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;

        vec_Vec3f viz_pts{goal_p, robot_state_.p};
        TimeConsuming t_viz("tviz", false);
        viz_.VisualizeGoalPath(viz_pts);
        time_consuming_[VISUALIZATION] += t_viz.stop();

        /// 1) Replan EXP traj
        ExpTrajInfo exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        RET_CODE exp_ret_code = GenerateReplanExpTraj(last_exp_traj_info_, exp_traj_info);
        time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            ROS_WARN(" -- [SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            return FAILED;
        } else if (exp_ret_code == NEW_TRAJ) {
            if (cfg_.print_log) {
                ROS_INFO(" -- [SUPER] in [ReplanOnce]: Last epx traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            ROS_WARN(" -- [SUPER] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            exp_traj_info.back_traj_start_TT = -1;
            if (cfg_.print_log) {
                ROS_INFO(" -- [SUPER] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED) {
            if (cfg_.print_log)
                ROS_INFO(" -- [SUPER] in [ReplanOnce]: No need to replan a new exp traj, use last one.");
            // return FAILED;
        }
        exp_traj_info.empty = false;
        exp_traj_info.is_rest2rest = false;

        if (!exp_traj_info.exp_yaw_traj.empty()) {
            viz_.VisualizeYawTraj(exp_traj_info.optimized_exp_traj, exp_traj_info.exp_yaw_traj);
        }

        BackTrajInfo back_traj_info;
        back_traj_info.ref_exp_traj = &exp_traj_info;
        // 2）生成back轨迹
        TimeConsuming t_back("t_back", false);
        RET_CODE back_ret_code = GenerateBackupTrajectory(back_traj_info);
        time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

        {
            ft += time_consuming_[EPX_TRAJ_FRONTEND] + time_consuming_[BACK_TRAJ_FRONTEND];
            ft_cnt++;
            bt += time_consuming_[BACK_TRAJ_OPT] + time_consuming_[EXP_TRAJ_OPT];
            bt_cnt++;
        }

        double replan_dt = replan_total_t.stop();
        if (replan_dt > cfg_.replan_forward_dt * 0.9) {
            ROS_WARN(" -- [SUPER] in [ReplanOnce]: replan dt = %.3f", replan_dt);
            return FAILED;
        }

        if (back_ret_code == SUCCESS) {
            const std::lock_guard<std::mutex> lock(cmd_traj_info_.sample_traj_mutex_);
            Trajectory old_pos_traj, old_yaw_traj;
            if (!exp_traj_info.optimized_exp_traj.getPartialTrajectoryByTime(0,
                                                                             back_traj_info.back_traj_start_TT,
                                                                             old_pos_traj)) {
                ROS_ERROR(" -- [SUPER] in [ReplanOnce]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
            if (!exp_traj_info.exp_yaw_traj.getPartialTrajectoryByTime(0,
                                                                       back_traj_info.back_traj_start_TT,
                                                                       old_yaw_traj)) {
                ROS_ERROR(" -- [SUPER] in [ReplanOnce]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
            // update cmd traj info
            cmd_traj_info_.pos_traj = old_pos_traj + back_traj_info.optimized_back_traj;
            cmd_traj_info_.yaw_traj = old_yaw_traj + back_traj_info.back_yaw_traj;
            
            cmd_traj_info_.backup_traj_start_TT = back_traj_info.back_traj_start_TT;
            cmd_traj_info_.use_back_traj = true;

            exp_traj_info.back_traj_start_TT = back_traj_info.back_traj_start_TT;
            exp_traj_info.empty = false;
            last_exp_traj_info_ = exp_traj_info;
            emer_stop = false;
            gi_.new_goal = false;
            // For visualization
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, cmd_traj_info_.backup_traj_start_TT);
            time_consuming_[VISUALIZATION] += t_viz.stop();
            if (cfg_.print_log)
                ROS_INFO(" -- [SUPER] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) {
            // 这次生成backup轨迹的点没有意义,
            emer_stop = false;
            exp_traj_info.empty = false;
            last_exp_traj_info_ = exp_traj_info;
            emer_stop = false;
            gi_.new_goal = false;
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, -1);
            time_consuming_[VISUALIZATION] += t_viz.stop();
            if (cfg_.print_log)
                ROS_INFO(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            // 意味着exp轨迹就已经完全free了
            const std::lock_guard<std::mutex> lock(cmd_traj_info_.sample_traj_mutex_);
            cmd_traj_info_.pos_traj = exp_traj_info.optimized_exp_traj;
            cmd_traj_info_.yaw_traj = exp_traj_info.exp_yaw_traj;
            
            cmd_traj_info_.use_back_traj = false;
            cmd_traj_info_.backup_traj_start_TT = INFINITY;
            exp_traj_info.empty = false;
            last_exp_traj_info_ = exp_traj_info;
            emer_stop = false;
            gi_.new_goal = false;
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeCommitTrajectory(cmd_traj_info_.pos_traj, -1);
            time_consuming_[VISUALIZATION] += t_viz.stop();
            if (cfg_.print_log)
                ROS_INFO(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            return SUCCESS;
        }
        ROS_WARN(" -- [SUPER] in [ReplanOnce]: GenerateBackupTrajectory return %s, replan Failed return",
                 RET_CODE_str[back_ret_code].c_str());
        return FAILED;
    }

    void SuperPlanner::GetOneHeartbeat(quadrotor_msgs::PolynomialTrajectory &heartbeat, bool &traj_finish) {
        heartbeat.type = quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
        heartbeat.header.stamp = ros::Time::now();
        heartbeat.header.frame_id = "world";
        heartbeat.start_WT_pos = ros::Time(cmd_traj_info_.pos_traj.start_WT);
        double eval_t = (ros::Time::now().toSec() - cmd_traj_info_.pos_traj.start_WT);
        traj_finish = false;
        double total_dur = cmd_traj_info_.pos_traj.getTotalDuration();
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }

        bool last_emer = emer_stop;
        if (cmd_traj_info_.use_back_traj && eval_t > cmd_traj_info_.backup_traj_start_TT) {
            emer_stop = true;
            last_exp_traj_info_.empty = true;
        } else {
            emer_stop = false;
        }
    }

    void SuperPlanner::GetCmdTraj(quadrotor_msgs::PolynomialTrajectory &cmd_traj) {
        cmd_traj.header.stamp = ros::Time::now();
        cmd_traj.header.frame_id = "world";
        cmd_traj.type = quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ |
                        quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;

        Trajectory &pos_traj = cmd_traj_info_.pos_traj;
        Trajectory &yaw_traj = cmd_traj_info_.yaw_traj;

        double yaw_total_dur = yaw_traj.getTotalDuration();
        double pos_total_dur = pos_traj.getTotalDuration();

        cmd_traj.start_WT_pos = ros::Time(pos_traj.start_WT);

        cmd_traj.piece_num_pos = pos_traj.getPieceNum();
        cmd_traj.order_pos = 7;
        cmd_traj.time_pos.resize(pos_traj.getPieceNum());
        cmd_traj.coef_pos_x.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
        cmd_traj.coef_pos_y.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
        cmd_traj.coef_pos_z.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
        cmd_traj.start_WT_pos = ros::Time(pos_traj.start_WT);

        if (!yaw_traj.empty()) {
            cmd_traj.type = cmd_traj.type |
                            quadrotor_msgs::PolynomialTrajectory::YAW_TRAJ;
            cmd_traj.piece_num_yaw = yaw_traj.getPieceNum();
            cmd_traj.order_yaw = 7;
            double col_size = cmd_traj.order_yaw + 1;
            cmd_traj.coef_yaw.resize(cmd_traj.piece_num_yaw * col_size);
            cmd_traj.time_yaw.resize(cmd_traj.piece_num_yaw);
            for (int i = 0; i < cmd_traj.piece_num_yaw; i++) {
                Eigen::VectorXd yaw_coef = yaw_traj[i].getCoeffMat().row(0);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_yaw[col_size * i], col_size) = yaw_coef;
                cmd_traj.time_yaw[i] = yaw_traj[i].getDuration();
            }
            cmd_traj.start_WT_yaw = ros::Time(yaw_traj.start_WT);
        }

        for (int i = 0; i < cmd_traj.piece_num_pos; i++) {
            Eigen::Matrix<double, 3, 8> coef = pos_traj[i].getCoeffMat();
            Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_x[8 * i], 8) = coef.row(0);
            Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_y[8 * i], 8) = coef.row(1);
            Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_z[8 * i], 8) = coef.row(2);
            cmd_traj.time_pos[i] = pos_traj[i].getDuration();
        }
    }

    void SuperPlanner::GetOneCommandFromTraj(quadrotor_msgs::PositionCommand &pos_cmd, bool &traj_finish) {
        cmd_traj_info_.sample_traj_mutex_.lock();
        pos_cmd.trajectory_flag = 0;
        double eval_t = (ros::Time::now().toSec() - cmd_traj_info_.pos_traj.start_WT);
        traj_finish = false;
        bool last_emer = emer_stop;
        if (cmd_traj_info_.use_back_traj && eval_t > cmd_traj_info_.backup_traj_start_TT) {
            emer_stop = true;
            pos_cmd.trajectory_flag = 2;
            last_exp_traj_info_.empty = true;
        } else {
            emer_stop = false;
        }
        if (last_emer != emer_stop) {
            if (last_emer) {
                ROS_INFO_STREAM(" -- [CMD] Emergency Stop End ========================");
            } else {
                ROS_INFO_STREAM(" -- [CMD] Emergency Stop Start ========================");
            }
        }
        double total_dur = cmd_traj_info_.pos_traj.getTotalDuration();
        
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        double cur_yaw = geometry_utils::get_yaw_from_quaternion(robot_state_.q);
        cmd_gene_.GetOneCommandFromTraj(cmd_traj_info_.pos_traj, eval_t,
                                        pos_cmd,
                                        emer_stop);
        cmd_traj_info_.sample_traj_mutex_.unlock();

        /// Get Yaw planning
        static double last_yaw = robot_state_.yaw;
        if (!cmd_traj_info_.yaw_traj.empty()) {
            double yaw_eval_t = (ros::Time::now().toSec() - cmd_traj_info_.yaw_traj.start_WT);
            double yaw_total_t = cmd_traj_info_.yaw_traj.getTotalDuration();
            yaw_eval_t = yaw_eval_t > yaw_total_t ? yaw_total_t : yaw_eval_t;
            pos_cmd.yaw = cmd_traj_info_.yaw_traj.getPos((yaw_eval_t))[0];
            pos_cmd.yaw_dot = cmd_traj_info_.yaw_traj.getVel((yaw_eval_t))[0];
        } else {
//                print("No yaw traj.");
            pos_cmd.yaw = last_yaw;
            pos_cmd.yaw_dot = 0;
        }
        if (isnan(pos_cmd.yaw)) {
            pos_cmd.yaw = last_yaw;
            pos_cmd.yaw_dot = 0;
        } else {
            last_yaw = pos_cmd.yaw;
        }
        if (isnan(pos_cmd.yaw_dot)) {
            pos_cmd.yaw_dot = 0;
        }

    }

    void SuperPlanner::GetOneMpcCommandFromTraj(quadrotor_msgs::MpcPositionCommand &pos_cmd, bool &traj_finish) {
        cmd_traj_info_.sample_traj_mutex_.lock();
        double eval_t = (ros::Time::now().toSec() - cmd_traj_info_.pos_traj.start_WT);
        if (cmd_traj_info_.use_back_traj && eval_t > cmd_traj_info_.backup_traj_start_TT) {
            emer_stop = true;
        }
        double total_dur = cmd_traj_info_.pos_traj.getTotalDuration();
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        cmd_gene_.GetOneMpcCommandFromTraj(cmd_traj_info_.pos_traj, eval_t, pos_cmd, emer_stop);
        static double last_yaw = robot_state_.yaw;
        if (!cmd_traj_info_.yaw_traj.empty()) {
            double yaw_eval_t = (ros::Time::now().toSec() - cmd_traj_info_.yaw_traj.start_WT) - 0.01;
            double yaw_total_t = cmd_traj_info_.yaw_traj.getTotalDuration();
            for (int i = 0; i < 8; i++) {
                /// Get Yaw planning
                yaw_eval_t += 0.01;
                yaw_eval_t = yaw_eval_t > yaw_total_t ? yaw_total_t : yaw_eval_t;
                pos_cmd.cmds[i].yaw = cmd_traj_info_.yaw_traj.getPos((yaw_eval_t))[0];
                pos_cmd.cmds[i].yaw_dot = cmd_traj_info_.yaw_traj.getVel((yaw_eval_t))[0];
                if (isnan(pos_cmd.cmds[i].yaw)) {
                    pos_cmd.cmds[i].yaw = last_yaw;
                    pos_cmd.cmds[i].yaw_dot = 0;
                } else {
                    last_yaw = pos_cmd.cmds[i].yaw;
                }
                if (isnan(pos_cmd.cmds[i].yaw_dot)) {
                    pos_cmd.cmds[i].yaw_dot = 0;
                }
                if (pos_cmd.cmds[i].yaw_dot > cfg_.yaw_dot_max) {
                    pos_cmd.cmds[i].yaw_dot = cfg_.yaw_dot_max;
                    pos_cmd.cmds[i].yaw = last_yaw + pos_cmd.cmds[i].yaw_dot * 0.01;
                }
                if (pos_cmd.cmds[i].yaw_dot < -cfg_.yaw_dot_max) {
                    pos_cmd.cmds[i].yaw_dot = -cfg_.yaw_dot_max;
                    pos_cmd.cmds[i].yaw = last_yaw + pos_cmd.cmds[i].yaw_dot * 0.01;
                }
            }
        } else {
            for (int i = 0; i < 8; i++) {
                pos_cmd.cmds[i].yaw = last_yaw;
                pos_cmd.cmds[i].yaw_dot = 0;
            }
        }
        cmd_traj_info_.sample_traj_mutex_.unlock();
    }

    void SuperPlanner::GetModuleTimeConsuming(vector<double> &time) {
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }

    bool SuperPlanner::shiftPointToNearestNoneOcc(const Vec3f &start_pt, Vec3f &shifted_point, bool use_inf_map) {
        GridType start_pt_type = map_ptr_->getInfGridType(start_pt);
        if (start_pt_type != OCCUPIED) {
            shifted_point = start_pt;
            return true;
        }

        for (auto nei: sphereical_neighbor_list_) {
            Vec3f nei_pt = start_pt + nei;
            if (map_ptr_->getInfGridType(nei_pt) != OCCUPIED) {
                shifted_point = nei_pt;
                return true;
            }
        }
        return false;
    }

    RET_CODE SuperPlanner::GenerateRestToRestExpTraj(const Vec3f &start_position, ExpTrajInfo &exp_traj_info) {
        exp_traj_info.optimized_exp_traj.clear();
        exp_traj_info.exp_yaw_traj.clear();
        /// 1) PathSearch connect all unfinished waypoints within planning horizon.
        vec_Vec3f path;
        bool bool_ret_code = PathSearch(start_position, gi_.goal_p, cfg_.planning_horizon, path);
        if (!bool_ret_code) {
            ROS_WARN(" -- [SVC] SafePathSearch failed, force return");
            exp_traj_info.empty = true;
            return FAILED;
        }

        /// 2) Generate SFCs cover the frontend path.
        PolytopeVec sfcs;

        {
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeGuidePath(path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        path.insert(path.begin(), robot_state_.p);

        bool_ret_code = cg_ptr_->SearchPolytopeOnPath(path, sfcs, cfg_.use_fov_cut);
        if (!bool_ret_code) {
            ROS_WARN(" [SVC] SearchPolytopeCoverPath failed, force return");
            exp_traj_info.empty = true;
            return FAILED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeExpSfc(sfcs);
            time_consuming_[VISUALIZATION] += t_viz.stop();

        }

        exp_traj_info.sfcs = sfcs;
        StatePVAJ init_state, fina_state;
        init_state.setZero();
        fina_state.setZero();
        init_state.col(0) = robot_state_.p;
        init_state.col(1) = exp_traj_info.cur_vel;
        fina_state.col(0) = path.back();
        if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
            fina_state.col(1) = (gi_.goal_p - robot_state_.p).normalized() * cfg_.exp_traj_cfg.max_vel / 2;
        }
        /// 5) Trajectory optimization
        bool_ret_code = exp_traj_opt_->optimize(init_state, fina_state, sfcs, exp_traj_info.optimized_exp_traj);
        if (!bool_ret_code) {
            ROS_WARN(" -- [SVC] Trajectory optimization failed, force return");
            return FAILED;
        }
        exp_traj_info.empty = false;
        Vec4f init_yaw{robot_state_.yaw, 0, 0, 0};
        Vec4f fina_yaw{0, 0, 0, 0};
        bool free_end{true};
        if (cfg_.goal_yaw_en) {
            if (!isnan(gi_.goal_yaw)) {
                free_end = false;
                fina_yaw[0] = gi_.goal_yaw;
            }
        }
        if (!yaw_traj_opt_->optimize(init_yaw, fina_yaw, exp_traj_info.optimized_exp_traj,
                                     exp_traj_info.exp_yaw_traj, 3, false, free_end)) {
            ROS_ERROR(" -- [SUPER] in [GenerateReplanExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        return SUCCESS;
    }

    RET_CODE SuperPlanner::GenerateReplanExpTraj(ExpTrajInfo &last_exp_traj_info, ExpTrajInfo &out_exp_traj_info) {
        TimeConsuming t_exp_frontend("t_exp_frontend", false);
        vector<TimePosPair> last_exp_traj_time_pos;
        const Trajectory &last_exp_traj = last_exp_traj_info.optimized_exp_traj;
        // The trajectory time when current replan process begin.
        double replan_process_start_WT = ros::Time::now().toSec();
        double replan_process_start_TT = replan_process_start_WT - last_exp_traj.start_WT;
        // The replan start state trajectory time in the last exp traj.
        double replan_state_TT = replan_process_start_TT + cfg_.replan_forward_dt;
        if (replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration()) {
            out_exp_traj_info = last_exp_traj_info;
            if (cfg_.print_log) {
                cout << YELLOW
                     << " -- [GenerateReplanExpTraj] replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration(), return FAILED and wait for plan form rest."
                     << RESET << endl;
                ROS_WARN(
                        " -- [GenerateReplanExpTraj] replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
            }
            if (emer_stop) {
                ROS_WARN(" -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                return FAILED;
            } else {
                return type_utils::NO_NEED;
            }

        }
        if (!last_exp_traj_info_.empty) {
            if (replan_state_TT >= last_exp_traj.getTotalDuration()) {
                out_exp_traj_info = last_exp_traj_info;
                ROS_WARN(
                        " -- [GenerateReplanExpTraj] replan_state_TT >= last_exp_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                if (emer_stop) {
                    ROS_WARN(" -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                } else {
                    return type_utils::NO_NEED;
                }
            }

            double last_exp_traj_total_duration = last_exp_traj.getTotalDuration();

            /// 1) Check a series of early termination conditions.
            if (!gi_.new_goal && last_exp_traj_info.sfcs.size() == 1 && last_exp_traj_info.connected_to_goal) {
                ROS_WARN(
                        " -- [SUPER] Replan, last exp have only one corridor and connected to goal return NONEED.");
                out_exp_traj_info = last_exp_traj_info;
                if (emer_stop) {
                    ROS_WARN(" -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                } else {
                    return type_utils::NO_NEED;
                }
            }

            if (!gi_.new_goal &&
                (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3) {
                // Return if the traj close to goal
                out_exp_traj_info = last_exp_traj_info;
                out_exp_traj_info.connected_to_goal = true;

                ROS_WARN(" -- [SUPER] Replan, close to goal and return NONEED.");
                if (emer_stop) {
                    ROS_WARN(" -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                } else {
                    return type_utils::NO_NEED;
                }
            }
        }

        /// Ready for replan.
        out_exp_traj_info.connected_to_goal = false;

        // * 2) Check if in emergency stop. While in emergency stop,
        // *    the guide trajectory should be a part of backup trajectory.
        Trajectory guide_pos_traj = last_exp_traj;
        Trajectory guide_yaw_traj = last_exp_traj_info.exp_yaw_traj;
        if (replan_state_TT >= cmd_traj_info_.backup_traj_start_TT || last_exp_traj_info_.empty) {
            guide_pos_traj = cmd_traj_info_.pos_traj;
            guide_yaw_traj = cmd_traj_info_.yaw_traj;
        }

        // * 3) Perform collision check on the guide trajectory.

        // TODO 0929 critical change for hot init.
        double eval_t = replan_state_TT;//replan_process_start_TT;
        double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();

        Vec3f temp_pt, last_sample_pt;
        last_exp_traj_time_pos.clear();
        last_exp_traj_info.all_traj_free = true;
        last_sample_pt = guide_pos_traj.getPos(eval_t);
        eval_t += cfg_.sample_traj_dt;
        // * 4) 记录replan点在evaluated_pts上的id
        int replan_id = -1;
        for (; eval_t < guide_pos_traj_total_time; eval_t += cfg_.sample_traj_dt) {
            temp_pt = guide_pos_traj.getPos(eval_t);
            if ((temp_pt - last_sample_pt).norm() < cfg_.resolution * 0.8) {
                continue;
            }

            GridType temp_grid = map_ptr_->getInfGridType(temp_pt);

            if (temp_grid == GridType::OCCUPIED || temp_grid == GridType::OUT_OF_MAP) {
                last_exp_traj_info.all_traj_free = false;
                break;
            }
            if (eval_t > replan_state_TT && replan_id == -1) {
                replan_id = last_exp_traj_time_pos.size();
            }
            last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
            last_sample_pt = temp_pt;
        }


        // * 6) Decide where to split the original exp trajecory and re-plan a new one with an A*,
        // *    If the whole trajectory if free,  the whole trajectory should be receding and if not, or a new goal
        // *    is given, we should only receiding a small distance and replan new trajectory ASAP
        double split_dis = cfg_.receding_dis;
        if (last_exp_traj_info.all_traj_free && !gi_.new_goal && cfg_.receding_dis > 0.0) {
            split_dis = std::numeric_limits<double>::max();
        } else {
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeObstaclePoint(temp_pt);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }



        // * 7）Begin replan process, first get the replan state from the commited trajectory.
        StatePVAJ replan_state;
        if (!GetStateFromTraj(guide_pos_traj, replan_state_TT, replan_state)) {
            ROS_WARN(" -- [SVC] Invalid traj or eval t");
            return FAILED;
        }
        // * Generate guide path with time stampe, for hot trajectory initialization
        vec_E<Vec3f> guide_path;
        // * the guide stamp is time from the replan start t
        vector<double> guide_stamp;
        if (split_dis <= 0 || last_exp_traj_time_pos.empty()) {
            /// No need receding, just path search.
            guide_path.push_back(replan_state.col(0));
            guide_stamp.push_back(0.0);
            last_exp_traj_time_pos.clear();
            last_exp_traj_time_pos.emplace_back(replan_state_TT, replan_state.col(0));
        } else {
            temp_pt = last_exp_traj_time_pos.back().second;
            // * 8) Pop all evaluated pts after the sampled point.
            while (map_ptr_->isOccupiedInflate(temp_pt) ||
                   (temp_pt - replan_state.col(0)).norm() > split_dis) {
                last_exp_traj_time_pos.pop_back();
                if (last_exp_traj_time_pos.empty()) {
                    ROS_WARN(" -- [SVC] WARN, all traj is collide in INF2");
                    break;
                }
                temp_pt = last_exp_traj_time_pos.back().second;
            }
            if (!last_exp_traj_time_pos.empty()) {
                Vec3f split_pt = last_exp_traj_time_pos.back().second;
                {
                    TimeConsuming t_viz("tviz", false);
                    viz_.VisualizeSplitPoint(split_pt);
                    viz_.VisualizeRecedingTrajectory(last_exp_traj_time_pos);
                    time_consuming_[VISUALIZATION] += t_viz.stop();

                }
                for (long unsigned int i = 0; i < last_exp_traj_time_pos.size(); i++) {
                    guide_path.push_back(last_exp_traj_time_pos[i].second);
                    guide_stamp.push_back(last_exp_traj_time_pos[i].first - last_exp_traj_time_pos.front().first);
                }
            } else {
                guide_path.push_back(replan_state.col(0));
                guide_stamp.push_back(0.0);
                last_exp_traj_time_pos.emplace_back(replan_state_TT, replan_state.col(0));
            }
        }

        ///=================The Second Part of Guide Path ================================================

        double guide_path_length = geometry_utils::computePathLength(guide_path);
        double temp_horizon = cfg_.planning_horizon - guide_path_length;

        vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        vector<int> sfc_waypoint_ids;

        if ((guide_path.front() - replan_state.col(0)).norm() > 1e-2) {
            guide_path.insert(guide_path.begin(), replan_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }


        if (temp_horizon > cfg_.resolution * 2) {
            /// start point TT + exp_traj start_WT
            double path_search_start_point_WT = last_exp_traj_time_pos.back().first + guide_pos_traj.start_WT;
            if ((guide_path.back() - gi_.goal_p).norm() < cfg_.resolution * 5) {
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - gi_.goal_p).norm() / cfg_.exp_traj_cfg.max_vel);
                guide_path.push_back(gi_.goal_p);
                // NO NEED
            } else {
                vec_Vec3f new_path;
                if (!PathSearch(guide_path.back(), gi_.goal_p, temp_horizon, new_path)) {
                    VisualUtils::VisualizePoint(viz_.point_pub_, guide_path.back());
                    ROS_WARN(" -- [SVC] PathSearch for new path failed");
                    return FAILED;
                }
                if (new_path.empty()) {
                    ROS_WARN(" -- [SVC] PathSearch for new path failed");
                    return FAILED;
                }
                Vec3f last_pt = new_path.front();
                double time_stamp = guide_stamp.back();
                for (long unsigned int i = 1; i < new_path.size(); i++) {
                    double t = (last_pt - new_path[i]).norm() / cfg_.exp_traj_cfg.max_vel;
                    time_stamp += t;
                    guide_path.push_back(new_path[i]);
                    guide_stamp.push_back(time_stamp);
                    last_pt = new_path[i];
                }
            }
        }
        out_exp_traj_info.connected_to_goal =
                (guide_path.back().head(2) - gi_.goal_p.head(2)).norm() < cfg_.resolution * 2;

        PolytopeVec sfcs;
        {
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeGuidePath(guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }
        bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(guide_path, sfcs, cfg_.use_fov_cut);
        if (!bool_ret_code) {
            ROS_WARN(" -- [SVC] SearchPolytopeOnPath for new path failed");
            return FAILED;
        }
        {
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeExpSfc(sfcs);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        time_consuming_[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();

        Trajectory out_traj;
        StatePVAJ f_state;
        f_state.setZero();
        f_state.col(0) = guide_path.back();
        if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
            f_state.col(1) = (gi_.goal_p - robot_state_.p).normalized() * cfg_.exp_traj_cfg.max_vel / 2;
        }
        if ((f_state.col(0) - gi_.goal_p).norm() < cfg_.resolution * 2) {
            f_state.col(1).setZero();
            f_state.col(0) = gi_.goal_p;
        }
        bool temp_ret;

//        // Add in dec 17
//        static const double no_need_dis = cfg_.rog_map_cfg.inflation_resolution * cfg_.rog_map_cfg.inflation_step;
//        if(sfcs.size() == 1 && (replan_state.col(0) - f_state.col(0)).norm() < no_need_dis){
//            out_exp_traj_info = last_exp_traj_info;
//            out_exp_traj_info.connected_to_goal = true;
//            return NO_NEED;
//        }

        TimeConsuming t_exp_opt("t_exp_opt", false);
        temp_ret = exp_traj_opt_->optimize(replan_state, f_state, guide_path, guide_stamp, sfcs, out_traj);
        time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();

        if (!temp_ret) {
            ROS_WARN(" -- [SVC] OptimizationExpTrajInPolytopes for new path failed");
            return FAILED;
        }
        double replan_total_t = (ros::Time::now().toSec() - replan_process_start_WT);
        if (replan_total_t > cfg_.replan_forward_dt) {
            ROS_ERROR(" -- [SVC] Replan over time(%lf)!!!! Return FAILED", replan_total_t);
            return FAILED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeExpTrajInColor(out_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        double new_traj_WT = replan_process_start_WT;
        replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;
//        if(replan_process_start_TT >= 0.5){
//            new_traj_WT -= 0.5;
//            replan_process_start_TT-=0.5;
//        }else{
//            new_traj_WT -= replan_process_start_TT;
//            replan_process_start_TT = 0;
//        }

        if (!guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       out_exp_traj_info.optimized_exp_traj)) {
            ROS_ERROR(" -- [SUPER] in [GenerateReplanExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.sfcs = sfcs;
        out_exp_traj_info.optimized_exp_traj = out_exp_traj_info.optimized_exp_traj + out_traj;
        out_exp_traj_info.optimized_exp_traj.start_WT = new_traj_WT;//last_exp_traj_info.replan_start_WT ;

        if (!GetStateFromTraj(guide_yaw_traj, replan_state_TT, replan_state)) {
            ROS_WARN(" -- [SVC] Invalid traj or eval t");
            return FAILED;
        }
        Vec4f init_yaw = replan_state.row(0);
        Vec4f fina_yaw{0, 0, 0, 0};
        bool free_end{true};
        if (cfg_.goal_yaw_en) {
            if (!isnan(gi_.goal_yaw)) {
                free_end = false;
                fina_yaw[0] = gi_.goal_yaw;
            }
        }
        Trajectory new_traj, old_traj;
        if (!yaw_traj_opt_->optimize(init_yaw, fina_yaw, out_traj, new_traj, 3, false, free_end)) {
            ROS_ERROR(" -- [SUPER] in [GenerateReplanExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       old_traj)) {
            ROS_ERROR(" -- [SUPER] in [GenerateReplanExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.exp_yaw_traj = old_traj + new_traj;
        out_exp_traj_info.exp_yaw_traj.start_WT = new_traj_WT;
        return SUCCESS;
    }

    RET_CODE SuperPlanner::GenerateBackupTrajectory(BackTrajInfo &back_traj_info) {

        // 1) 首先根据当前fov和exp轨迹，生成一个free的走廊
        // TODO 根据飞机当前FOV，切割free的走廊，目前只利用了position信息
        drone_state_mutex_.lock();
        back_traj_info.robot_state = robot_state_;
        drone_state_mutex_.unlock();

        TimeConsuming t_back_frontend("t_back_frontend", false);
        double total_dur = back_traj_info.ref_exp_traj->optimized_exp_traj.getTotalDuration();
        double start_t;
        // 判断是否为rest2rest的轨迹
        if (back_traj_info.ref_exp_traj->is_rest2rest) {
            start_t = 0;
        } else {
            start_t = ros::Time::now().toSec() - back_traj_info.ref_exp_traj->optimized_exp_traj.start_WT;
        }

        if (start_t > total_dur - 0.01) {
            return NO_NEED;
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        // 同时记录每一个点的刹车时间和刹车距离
        vector<double> min_stop_dis;
        vector<TimePosPair> eval_ps;
        Vec3f temp_vel;

        // 记录当前时刻到最远时刻的所有可视部分
        Vec3f last_pos = back_traj_info.ref_exp_traj->optimized_exp_traj.getPos(start_t);
        for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt) {
            temp_point = back_traj_info.ref_exp_traj->optimized_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) {
                continue;
            }
            last_pos = temp_point;
            temp_vel = back_traj_info.ref_exp_traj->optimized_exp_traj.getVel(out_t);

            // 计算最小刹车距离，并且把z置零
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.push_back(std::pair<double, Vec3f>(out_t, temp_point));

            if (!map_ptr_->isLineFree(back_traj_info.robot_state.p,
                                      temp_point,
                                      cfg_.safe_corridor_line_max_length,
//                                      cfg_.corridor_line_max_length * 2,
                                      cfg_.seed_line_neighbour)) {
                all_traj_visible = false;
                break;
            }
        }

        if (all_traj_visible) {
            back_traj_info.empty = true;
            {
                double dur = back_traj_info.ref_exp_traj->optimized_exp_traj.getTotalDuration();
                Vec3f seed_pt = back_traj_info.ref_exp_traj->optimized_exp_traj.getPos(dur);
                Line line{back_traj_info.robot_state.p, seed_pt};
                if (cg_ptr_->GeneratePolytopeFromLine(line, back_traj_info.free_poly)) {
                    back_traj_info.free_poly.SetKnownFree(true);

                    {
                        TimeConsuming t_viz("tviz", false);
                        viz_.VisualizeFreePoly(back_traj_info.free_poly);
                        time_consuming_[VISUALIZATION] += t_viz.stop();
                    }

                }
            }
            return FINISH;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= cfg_.sample_traj_dt;
            Vec3f out_p = back_traj_info.ref_exp_traj->optimized_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > cfg_.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);
        if (back_traj_info.ref_exp_traj->back_traj_start_TT > 0 &&
            seed_point_t < back_traj_info.ref_exp_traj->back_traj_start_TT) {
            return NO_NEED;
        }
        Vec3f seed_point = back_traj_info.ref_exp_traj->optimized_exp_traj.getPos(seed_point_t);

        Vec3f shifted_robot_p;
        shiftPointToNearestNoneOcc(back_traj_info.robot_state.p, shifted_robot_p);

        Line line{shifted_robot_p, seed_point};
        if (!cg_ptr_->GeneratePolytopeFromLine(line, back_traj_info.free_poly)) {
            ROS_WARN(" -- [SVC] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        Eigen::Matrix3Xd vPoly;
        if (!geometry_utils::findInterior(back_traj_info.free_poly.GetPlanes(), inner)) {
            ROS_WARN(" -- [SVC] Cannot generate feasible backup sfc, force return");
            vec_Vec3f seed{back_traj_info.robot_state.p, seed_point};

            {
                TimeConsuming t_viz("tviz", false);
                viz_.VisualizeSafeCorridorSeed(seed);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            return FAILED;
        }


        if (cfg_.use_fov_cut) {
            if (!CutPolytopeByFov(back_traj_info.free_poly, seed_point)) {
                ROS_WARN(" -- [SVC] CutPolytopeByFov failed, force return");
                return FAILED;
            }
        }
        back_traj_info.free_poly.SetKnownFree(true);

        {
            TimeConsuming t_viz("tviz", false);
            viz_.VisualizeFreePoly(back_traj_info.free_poly);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }
        // 2) 在free走廊里生成back轨迹
        Vec3f out_p = temp_point;
        double t_R = 0.0;
        double eval_t = eval_ps.back().first + cfg_.sample_traj_dt;
        last_pos = eval_ps.back().second;
        while (back_traj_info.free_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = back_traj_info.ref_exp_traj->optimized_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
                eval_t += cfg_.sample_traj_dt;
                continue;
            }
            temp_vel = back_traj_info.ref_exp_traj->optimized_exp_traj.getVel(out_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt;
        }
        eval_ps.pop_back();
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

//        bool use_new{true};
//        if (use_new) {
        double t0 = ros::Time::now().toSec() -
                    back_traj_info.ref_exp_traj->optimized_exp_traj.start_WT + 0.01;
        double te = seed_point_t;
//            cout << "t0: " << t0 << endl;
//            cout << "te: " << te << endl;
//            cout << "exp_traj_dur: " << back_traj_info.ref_exp_traj->optimized_exp_traj.getTotalDuration() << endl;
        double vel_e_n = back_traj_info.ref_exp_traj->optimized_exp_traj.getVel(te).norm();
        double heu_ts = max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
        double heu_dur = te - heu_ts;
        Vec3f heu_p = seed_point;
        time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        bool temp_ret = back_traj_opt_->optimize(back_traj_info.ref_exp_traj->optimized_exp_traj,
                                                 t0,
                                                 te,
                                                 heu_ts,
                                                 heu_p,
                                                 heu_ts,
                                                 back_traj_info.free_poly,
                                                 back_traj_info.optimized_back_traj,
                                                 opt_ts);
        time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();
        if (!temp_ret) {
            ROS_WARN(" -- [SVC] OptimizationBakTrajInPolytopes failed, force return");
            back_traj_info.empty = true;
            return OPT_FAILED;
        } else {
            back_traj_info.back_traj_start_TT = opt_ts;
            {
                TimeConsuming t_viz("tviz", false);
                viz_.VisualizeBackupTraj(back_traj_info.optimized_back_traj);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            Vec4f yaw_init_vec = back_traj_info.ref_exp_traj->exp_yaw_traj.getState(opt_ts).row(0);
            Vec4f yaw_goal{0, 0, 0, 0};
            bool free_end{true};
            if (cfg_.goal_yaw_en) {
                if (!isnan(gi_.goal_yaw)) {
                    free_end = false;
                    yaw_goal[0] = gi_.goal_yaw;
                }
            }
            Trajectory yaw_traj;
            if (!yaw_traj_opt_->optimize(yaw_init_vec, yaw_goal, back_traj_info.optimized_back_traj,
                                         back_traj_info.back_yaw_traj, 3, false, free_end)) {
                ROS_ERROR(" -- [SVC] in [GenerateBackupTrajectory] YawTrajOpt FAILD.");
                return OPT_FAILED;
            }


            if (opt_ts < t0) {
                ROS_ERROR(" -- [SVC] opt_ts %lf < t0 %lf", opt_ts, t0);
                return OPT_FAILED;
            }
            double new_ts_WT = back_traj_info.ref_exp_traj->optimized_exp_traj.start_WT + opt_ts;
            double commited_ts_WT = cmd_traj_info_.backup_traj_start_WT.toSec();
            if (new_ts_WT < commited_ts_WT) {
                ROS_ERROR(" -- [SVC] new_ts_WT %lf < commited_ts_WT %lf", new_ts_WT, commited_ts_WT);
                return OPT_FAILED;
            }
            return SUCCESS;
        }
        ROS_WARN(" -- [SVC] Cannot find backup traj start point.");
        return FAILED;
    }

    bool SuperPlanner::GetStateFromTraj(const Trajectory &traj, double eval_t, StatePVAJ &out_state) {
        double dur = traj.getTotalDuration();
        if (eval_t < 0) {
            return false;
        }
        if (eval_t > dur) eval_t = dur;
        out_state.col(0) = traj.getPos(eval_t);
        out_state.col(1) = traj.getVel(eval_t);
        out_state.col(2) = traj.getAcc(eval_t);
        out_state.col(3) = traj.getJer(eval_t);
        return true;
    }

    int SuperPlanner::getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt) {
        if (goals.size() == 1) {
            return 0;
        }
        Vec3f a = start_pt, b;
        int min_id = 0;
        double min_dis = 1e10;
        for (long unsigned int i = 0; i < goals.size() - 1; i++) {
            b = goals[i];
            double dis = geometry_utils::pointLineSegmentDistance(start_pt, a, b);
            if (dis < min_dis) {
                min_dis = dis;
                min_id = i;
            }
            a = b;
        }
        return min_id;
    }

    bool
    SuperPlanner::PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                             const double &searching_horizon,
                             vec_Vec3f &path) {
        std::lock_guard<std::mutex> lock(path_search_mutex_);
        if (searching_horizon <= 0.0) {
            ROS_ERROR(" -- [SVC] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        GridType start_type;
        start_type = map_ptr_->getGridType(start_pt);

        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (start_type == GridType::OCCUPIED ||
            start_type == GridType::OUT_OF_MAP) {
            ROS_WARN(
                    " -- [SVC] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE;
        vec_Vec3f out_path;
        RET_CODE ret_es = astar_ptr_->EscapePathSearch(start_pt, flag_es, out_path);
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                ROS_ERROR(
                        " -- [SVC] Escape path search failed with [%s], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
//            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);

        RET_CODE ret_code = astar_ptr_->PointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                               path);

        //add may23, if failed on inf map, use prob map try again
 
        if (ret_code == NO_PATH) {
            flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = astar_ptr_->PointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                          path);
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        }
        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            ROS_ERROR(
                    " -- [SVC] Path search failed with [%s], force return.\n", RET_CODE_STR[ret_code].c_str());
            return false;
        }
        if (!start_point_escape_path.empty()) {
            path.insert(path.begin(), start_point_escape_path.begin(),
                        start_point_escape_path.end());
        }

        if (path.empty()) {
            ROS_WARN(
                    " -- [SVC] Path search failed with empty segments, force return.");
            return false;
        }
        path.insert(path.begin(), start_pt);
        if (ret_code == REACH_GOAL) {
            path.push_back(goal);
        }

        TimeConsuming t_viz("tviz", false);
        viz_.VisualizeGuidePath(path);
        time_consuming_[VISUALIZATION] += t_viz.stop();
        return true;
    }

    bool SuperPlanner::CutPolytopeByFov(Polytope &poly, const Vec3f &guide_point) {
        Vec3f guide_point_B = robot_state_.q.matrix().transpose() * (guide_point - robot_state_.p);
        double yaw_in_body_frame = atan2(guide_point_B.y(), guide_point_B.x());
        Mat3f rotation_matrix3;
        rotation_matrix3 = Eigen::AngleAxisd(yaw_in_body_frame, Eigen::Vector3d::UnitZ());
        Mat3f R = robot_state_.q.matrix() * rotation_matrix3;
        MatD4f fov_plane, temp_plane;
        vec_E<Mat3f> fov_plane_pt;
        Vec3f dir = (robot_state_.p - guide_point).normalized();
        fov_checker_->GetFovCheckPlane(R, robot_state_.p + dir * cfg_.resolution * 2, fov_plane,
                                       fov_plane_pt);
        temp_plane.resize(poly.SurfNum() + fov_plane.rows(), 4);
        temp_plane << poly.GetPlanes(), fov_plane;
        Vec3f interior;
        if (geometry_utils::findInteriorDist(poly.GetPlanes(), interior) < cfg_.resolution) {
            return false;
        }
        poly.SetPlanes(temp_plane);
        return true;
    }

    bool SuperPlanner::ReachEnd() {
        return ((ros::Time::now().toSec() - cmd_traj_info_.pos_traj.start_WT) >
                cmd_traj_info_.pos_traj.getTotalDuration());
    }

    void SuperPlanner::getRobotState(RobotState &out) {
        robot_state_ = map_ptr_->getRobotState();
        out = robot_state_;
    }

    bool SuperPlanner::isEasyGoal(const Vec3f &goal_position) {
        vec_Vec3f path;
        if (!PathSearch(robot_state_.p, goal_position, cfg_.planning_horizon, path)) {
            return false;
        }
        if (path.empty()) {
            cout << RED << "[SVC] path_segs is empty." << RESET << endl;
            return false;
        }
        if ((path.back() - goal_position).norm() < 0.5) {
            return true;
        }
        return false;
    }

    void
    SuperPlanner::findNearestKnownFreeGoal(const Vec3f &goal_position, Vec3f &free_local_goal,
                                           const double &safe_dis) {
        map_ptr_->isLineFree(robot_state_.p, goal_position, free_local_goal, cfg_.planning_horizon,
                             cfg_.seed_line_neighbour);
        if (safe_dis > 0) {
            if ((goal_position - free_local_goal).norm() > 1e-3) {
                free_local_goal = (free_local_goal - goal_position).normalized() * safe_dis + free_local_goal;
            }
        }
    }


}
