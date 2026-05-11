#include <planner_fsm/fsm.h>

namespace planner_fsm {

    FSM::~FSM() {
        write_time_.close();
    }

    void FSM::setGoal(const Vec3f &goal) {
        Vec3f click_point = goal;
        if (cfg_.click_height > -5) {
            click_point.z() = cfg_.click_height;
        }
        if (cfg_.swarm_en) {
            if (!been_selected_) {
                cout << RED << "Drone <" << cfg_.drone_id << "> is not selected, continue.";
                return;
            } else {
                cout << GREEN << "Drone <" << cfg_.drone_id << "> rcv new goal at" << click_point.transpose() << RESET
                     << endl;
            }
        }

        Quatf q;
        q.w() = 1;

        if (cfg_.click_yaw_en) {
            gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
        } else {
            gi_.goal_yaw = NAN;
        }

        planner_ptr_->shiftPointToNearestNoneOcc(click_point, gi_.goal_p, true);
        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
//                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return;
        } else {
            cout << GREEN << " -- [Rviz] Get click at " << RESET << click_point.transpose() << endl;
        }
        started_ = true;
        gi_.new_goal = true;
    }

    void FSM::init(const ros::NodeHandle &nh) {
        // 初始化参数读取
        nh_ = nh;
        cfg_ = FsmConfig(nh_);
        // 初始化Planner
        planner_ptr_.reset(new SuperPlanner(nh, cfg_.planner_cfg));
        cmd_pub = nh_.advertise<quadrotor_msgs::PositionCommand>(cfg_.cmd_topic, 10);
        if (cfg_.mpc_cmd_type == MPC_PVAJ_MODE) {
            mpc_cmd_pub_ = nh_.advertise<quadrotor_msgs::MpcPositionCommand>(cfg_.mpc_cmd_topic, 10);
        } else if (cfg_.mpc_cmd_type == MPC_POLYTRAJ_MODE) {
            mpc_cmd_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>(cfg_.mpc_cmd_topic, 10);
        }
        mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("fsm/mkr_arr", 100);
        path_pub_ = nh_.advertise<nav_msgs::Path>("fsm/path", 100);
        select_sub_ = nh_.subscribe("/rviz_selected_points", 10, &FSM::SelectedPatchCallback, this); 

        current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("fsm/current_goal", 10);
        current_goalpath_pub_ = nh_.advertise<nav_msgs::Path>("fsm/current_goalpath", 10);
        current_goal_timer_ = nh_.createTimer(ros::Duration(0.1), &FSM::PubCurrentGoalCallback, this);

        int cmd_cnt = 0;
        if (cfg_.auto_pilot_en) {
            rc_sub_ = nh_.subscribe("/mavros/rc/in", 1, &FSM::RcCallback, this);
            cout << YELLOW << " -- [FSM] AUTOPILOT ENABLE." << RESET << endl;
            cmd_cnt++;
        }
        if (cfg_.click_goal_en) {
            goal_sub_ = nh_.subscribe(cfg_.click_goal_topic, 1, &FSM::GoalCallback, this);
            cout << YELLOW << " -- [FSM] CLICKGOAL ENABLE." << RESET << endl;
            cmd_cnt++;
        }
        if (cfg_.path_goal_en) {
            path_goal_sub_ = nh_.subscribe(cfg_.path_goal_topic, 1, &FSM::PathGoalCallback, this);
            cout << YELLOW << " -- [FSM] PATHGOAL ENABLE." << RESET << endl;
            cmd_cnt++;
        }
        if (cmd_cnt != 1) {
            cout << RED << " -- [FSM] CMD INPUT ERROR." << RESET << endl;
            exit(0);
        }

        mpc_cmd_.cmds.resize(8);

        execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FSM::MainFsmCallback, this); // 100Hz
        cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &FSM::PubCmdCallback, this); // 100Hz
        replan_timer_ = nh_.createTimer(ros::Duration(1.0 / cfg_.replan_rate), &FSM::ReplanTimerCallback,
                                        this); // 10Hz
        write_time_.open(DEBUG_FILE_DIR("time_consuming.csv"), std::ios::out | std::ios::trunc);
        log_module_time.resize(9);
        for (int i = 0; i < 9; i++) {
            write_time_ << log_time_str[i];
            if (i != 8) {
                write_time_ << ",";
            }
        }
        write_time_ << endl;
        machine_state_ = INIT;
        system_start_time_ = ros::Time::now();

        pid_cmd_.kx[0] = 5.7;
        pid_cmd_.kx[1] = 5.7;
        pid_cmd_.kx[2] = 4.2;

        pid_cmd_.kv[0] = 3.4;
        pid_cmd_.kv[1] = 3.4;
        pid_cmd_.kv[2] = 4.0;
    }

    void FSM::WriteTimeToLog() {
        write_time_ << (ros::Time::now().toSec() - system_start_time_.toSec()) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void FSM::PubCmdCallback(const ros::TimerEvent &event) {
        if (stop) {
            return;
        }
        if (machine_state_ != FOLLOW_TRAJ && machine_state_ != EMER_STOP) {
            return;
        }

        if (cfg_.mpc_cmd_type == MPC_POLYTRAJ_MODE && !cfg_.auto_pilot_en) {
            quadrotor_msgs::PolynomialTrajectory heartbeat;
            planner_ptr_->GetOneHeartbeat(heartbeat, traj_finish_);
            planner_ptr_->GetOneCommandFromTraj(pid_cmd_, traj_finish_);
            mpc_cmd_pub_.publish(heartbeat);
            cmd_pub.publish(pid_cmd_);
            if (traj_finish_) {
                cout << GREEN << " -- [FSM] Traj finish." << RESET << endl;
                if (closeToGoal(0.1)) {
                    ChangeState("PubCmdCallback", WAIT_GOAL);
                } else {
                    ChangeState("PubCmdCallback", GENERATE_TRAJ);
                }
            }
        } else {
            planner_ptr_->GetOneCommandFromTraj(pid_cmd_, traj_finish_);
            planner_ptr_->GetOneMpcCommandFromTraj(mpc_cmd_, traj_finish_);
            if (traj_finish_) {
                cout << GREEN << " -- [FSM] Traj finish." << RESET << endl;
                if (closeToGoal(0.1)) {
                    ChangeState("PubCmdCallback", WAIT_GOAL);
                } else {
                    ChangeState("PubCmdCallback", GENERATE_TRAJ);
                }
            }
            if (cfg_.auto_pilot_en) {
                pid_cmd_.yaw = yaw_;
                pid_cmd_.yaw_dot = yaw_dot_;
                for (long unsigned int i = 0; i < mpc_cmd_.mpc_horizon; i++) {
                    mpc_cmd_.cmds[i].yaw = yaw_;
                    mpc_cmd_.cmds[i].yaw_dot = yaw_dot_;
                }
            } else if (cfg_.click_goal_en) {
                static double last_yaw = robot_state_.yaw;
                pid_cmd_.yaw = gi_.goal_yaw * 0.5 + last_yaw * 0.5;
                pid_cmd_.yaw_dot = 0;//(pid_cmd_.yaw - last_yaw) / 0.02 ;
                last_yaw = pid_cmd_.yaw;
                if (pid_cmd_.yaw_dot > cfg_.planner_cfg.yaw_dot_max) {
                    pid_cmd_.yaw_dot = cfg_.planner_cfg.yaw_dot_max;
                    pid_cmd_.yaw = last_yaw + pid_cmd_.yaw_dot * 0.01;
                }
                if (pid_cmd_.yaw_dot < -cfg_.planner_cfg.yaw_dot_max) {
                    pid_cmd_.yaw_dot = -cfg_.planner_cfg.yaw_dot_max;
                    pid_cmd_.yaw = last_yaw + pid_cmd_.yaw_dot * 0.01;
                }
                for (int i = 0; i < mpc_cmd_.mpc_horizon; i++) {
                    mpc_cmd_.cmds[i].yaw = pid_cmd_.yaw;
                    mpc_cmd_.cmds[i].yaw_dot = pid_cmd_.yaw_dot;
                }
            }

            cmd_pub.publish(pid_cmd_);
            mpc_cmd_pub_.publish(mpc_cmd_);
        }
    }

    void FSM::ReplanTimerCallback(const ros::TimerEvent &event) {
        if (stop) {
            return;
        }

        if (machine_state_ != FOLLOW_TRAJ) {
            return;
        }

        if (finish_plan) {
            return;
        }

        if (plan_from_rest_) {
            plan_from_rest_ = false;
            return;
        }

        planner_ptr_->shiftPointToNearestNoneOcc(gi_.goal_p, gi_.goal_p, true);

        TimeConsuming replan_once_time("replan_once_time", false);

        RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (ret_code == FAILED) {
            cout << YELLOW << " -- [FSM] ReplanOnce failed." << RESET << endl;
        } else if (ret_code == EMER) {
            ChangeState("ReplanTimerCallback", EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {
            ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            gi_.new_goal = false;
            if (cfg_.mpc_cmd_type == MPC_POLYTRAJ_MODE) {
                PubCmdPolyTraj();
            }
        }

        planner_ptr_->GetModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        WriteTimeToLog();
    }

    void FSM::MainFsmCallback(const ros::TimerEvent &event) {
        if (stop) {
            return;
        }
        static ros::Time start_follow_time;
        static ros::Time fsm_start_time = ros::Time::now();
        double cur_t = (ros::Time::now() - fsm_start_time).toSec();
        static double last_print_t = 0.0;
        planner_ptr_->getRobotState(robot_state_);

        static nav_msgs::Path path;

        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros::Time::now().toSec() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [FSM] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [FSM] Wait for goal." << RESET << endl;
            }
            cout<<std::fixed<<std::setprecision(3);
            cout << GREEN << " -- [FSM " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        if (cfg_.path_goal_en && machine_state_ != INIT && machine_state_ != WAIT_GOAL && started_ &&
            gi_.new_goal) {
            ROS_ERROR("Path mission is not implemented in this version.");
            return;
        }

        switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros::Time::now().toSec() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [FSM] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                if (!gi_.new_goal) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                path.poses.clear();
                break;
            }
            case GENERATE_TRAJ: {
                if (closeToGoal(0.3)) {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    finish_plan = true;
                    return;
                }
                int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
                if (retcode == SUCCESS || retcode == FINISH) {
                    gi_.new_goal = false;
                    start_follow_time = ros::Time::now();
                    plan_from_rest_ = true;
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }
                    if (cfg_.mpc_cmd_type == MPC_POLYTRAJ_MODE) {
                        PubCmdPolyTraj();
                    }
                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                } else {
                    cout << YELLOW << " -- [FSM] PlanFromRest failed, try replan." << RESET << endl;
                    ros::Duration(0.1).sleep();
                }
                break;
            }
            case FOLLOW_TRAJ: {
                path.header.frame_id = "world";
                path.header.stamp = ros::Time::now();
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = robot_state_.p(0);
                pose.pose.position.y = robot_state_.p(1);
                pose.pose.position.z = robot_state_.p(2);
                pose.pose.orientation.x = robot_state_.q.x();
                pose.pose.orientation.y = robot_state_.q.y();
                pose.pose.orientation.z = robot_state_.q.z();
                pose.pose.orientation.w = robot_state_.q.w();
                path.poses.push_back(pose);
                path_pub_.publish(path);
                break;
            }
            case EMER_STOP: {
                if (planner_ptr_->ReachEnd()) {
                    ChangeState("MainFsmCallback", YAWING);
                }
                break;
            }
            default:
                break;
        }
    }

    bool FSM::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    void FSM::GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
        current_goal = *msg;
        current_goal_pub_.publish(msg);

        Vec3f click_point(msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z);
        // if (cfg_.click_height > -5) {
        //     click_point.z() = cfg_.click_height;
        // }
        if (cfg_.swarm_en) {
            if (!been_selected_) {
                cout << RED << "Drone <" << cfg_.drone_id << "> is not selected, continue.";
                return;
            } else {
                cout << GREEN << "Drone <" << cfg_.drone_id << "> rcv new goal at" << click_point.transpose() << RESET
                     << endl;
            }
        }

        // cout << GREEN << "SUPER recieve goal" << endl;

        Quatf q;
        q.w() = msg->pose.orientation.w;
        q.x() = msg->pose.orientation.x;
        q.y() = msg->pose.orientation.y;
        q.z() = msg->pose.orientation.z;

        planner_ptr_->shiftPointToNearestNoneOcc(click_point, gi_.goal_p, true);
        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
//                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return;
        }
        if(isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())){
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [FSM] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }else if (cfg_.click_yaw_en) {
            gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
            cout << GREEN << " -- [FSM] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw: "
                 << gi_.goal_yaw * 57.3 <<" deg" << RESET << endl;
        } else {
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [FSM] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }

        started_ = true;
        gi_.new_goal = true;
    }

    void FSM::SetGoalPosiAndYaw(const Vec3f &g_p, const double &g_y) {
        gi_.goal_yaw = g_y;
        gi_.goal_p = g_p;
        if (g_p.norm() < 1e-3) {
            ROS_ERROR("Set goal to zero, exit.");
            // exit(0);
        }
    }

    void FSM::RcCallback(const mavros_msgs::RCInConstPtr &msg) {
        static double last_call = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cur_t - last_call < 0.2) {
            return;
        }
        last_call = cur_t;
        double axes[4];
        constexpr int L_UD = 2, L_LR = 3, R_UD = 1, R_LR = 0;
        constexpr double DEAD_ZONE = 0.05;
        int dead_cnt = 0;
        static bool first_yaw_dead{true};
        for (size_t i = 0; i < 4; i++) {
            axes[i] = (static_cast<double>(msg->channels[i]) - 1500.0) / 500.0;
            if (fabs(axes[i]) < DEAD_ZONE) {
                axes[i] = 0;
                dead_cnt++;
            }
        }
        if (msg->channels[6] < 1500 || dead_cnt == 4) {
            if (first_yaw_dead) {
                first_yaw_dead = false;
                yaw_ = robot_state_.yaw;
            }
            auto_pilot_vel_w_ = Eigen::Vector3d::Zero();
            gi_.new_goal = false;
            return;
        }

        first_yaw_dead = true;
        static ros::Time last_compute_time = ros::Time::now();
        if (fabs(axes[L_LR]) >= DEAD_ZONE) {
            yaw_dot_ = -axes[L_LR] * cfg_.auto_pilot_yaw_speed;
            double dt = (ros::Time::now() - last_compute_time).toSec();
            yaw_ += dt * yaw_dot_;
        } else {
//            dead_cnt++;
            yaw_dot_ = 0.0;
        }
        last_compute_time = ros::Time::now();

        Eigen::Vector3d vel_b(axes[R_UD], axes[R_LR],
                              axes[L_UD]);

        Eigen::Matrix3d rotation_matrix_4;
        rotation_matrix_4 = Eigen::AngleAxisd(robot_state_.yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        auto_pilot_vel_w_ = -rotation_matrix_4 * vel_b * cfg_.auto_pilot_speed;
        auto_pilot_vel_w_.z() *= 0.2;
        Vec3f click_point = robot_state_.p + auto_pilot_vel_w_;
        VisualUtils::VisualizePoint(mkr_pub_, click_point, Color::Blue(), "rc_goal", 0.2, 0);
        Vec3f local_goal;
        //=====================================================
        // 1. check if it is an easy goal
        if (cfg_.safe_auto_pilot) {
            if (!planner_ptr_->isEasyGoal(click_point)) {
                cout << RED << " -- [RcCallback] Not an easy goal, try to find nearest known free." << RESET
                     << endl;
                // 1.1 try to find a point in the known free space
                Vec3f easy_goal = click_point;
                planner_ptr_->findNearestKnownFreeGoal(click_point, easy_goal, cfg_.auto_pilot_safe_dis);
                VisualUtils::VisualizePoint(mkr_pub_, easy_goal, Color::Purple(), "easy_goal", 0.3, 0);
                // just run to the easy goal
//                    planner_ptr_->shiftPointToNearestNoneOcc(easy_goal, local_goal, true);
                local_goal = easy_goal;
                if ((local_goal - robot_state_.p).norm() < 0.5) {
                    if (machine_state_ == WAIT_GOAL) {
                        return;
                    }
                    ROS_WARN("Close to goal, abort this goal.");
                    gi_.new_goal = false;
//                        ChangeState("AutoPilot RC Callback", WAIT_GOAL);
                    return;
                }
                SetGoalPosiAndYaw(local_goal, yaw_);
                VisualUtils::VisualizePoint(mkr_pub_, local_goal, Color::Purple(), "rc_goal", 0.3, 0);
                started_ = true;
                gi_.new_goal = true;
                return;
            }
//                else {
//                    print(fg(color::green), " -- [RcCallback] Easy goal, just run.\n");
//                }
        }

        // just run to the easy goal
        planner_ptr_->shiftPointToNearestNoneOcc(click_point, local_goal, true);
        if ((local_goal - robot_state_.p).norm() < cfg_.planner_cfg.resolution * 2) {
            if (machine_state_ == WAIT_GOAL) {
                return;
            }
            ROS_WARN("Close to goal, abort this goal.");
            gi_.new_goal = false;
            return;
        }
        VisualUtils::VisualizePoint(mkr_pub_, click_point, Color::Purple(), "rc_goal", 0.3, 0);
        SetGoalPosiAndYaw(local_goal, yaw_);
        started_ = true;
        gi_.new_goal = true;
    }

    void FSM::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        ROS_INFO(" -- [FSM]: [%s] change state from [%s] to [%s].", call_func.c_str(),
                 MACHINE_STATE_STR[int(machine_state_)].c_str(), MACHINE_STATE_STR[int(new_state)].c_str());
        machine_state_ = new_state;
    }

    void FSM::PathGoalCallback(const nav_msgs::PathConstPtr &msg) {
        ROS_ERROR("Path goal is not implemented yet.");
    }

    void FSM::PubCurrentGoalCallback(const ros::TimerEvent &event)
    {
        // cout << YELLOW << "Timer pub current goal triggered, current goal:" << current_goal.pose.position.x << ", " \
        // << current_goal.pose.position.y << ", "<< current_goal.pose.position.z << " " << endl;
        current_goal_pub_.publish(current_goal);
    }

}