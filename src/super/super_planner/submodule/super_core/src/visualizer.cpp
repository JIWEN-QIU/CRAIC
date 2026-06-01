#include <super_core/visualizer.h>

namespace super_planner {


    Visualizer::Visualizer(const ros::NodeHandle &nh, const PlannerConfig cfg) {
        Init(nh, cfg);
    }

    void Visualizer::Init(const ros::NodeHandle &nh) {
        nh_ = nh;

        backup_sfc_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/backup_sfc", 100, true);
        goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/goal", 100, true);
        backup_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/backup_traj", 100, true);
        exp_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/exp_traj", 100, true);
        commit_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/commit_traj", 100, true);
        receding_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/receding_traj", 100, true);
        exp_sfcs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/exp_sfcs", 100, true);
        point_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/points", 100, true);
        fov_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/fov", 100, true);
        astar_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/astar", 100, true);
        guide_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/sfc_guide_path", 100, true);

        exp_traj_data_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("visualization/exp_traj_data",
                                                                                 100);
        exp_sfc_data_pub_ = nh_.advertise<quadrotor_msgs::PolyhedronVector>("visualization/exp_sfc_data", 100);
        back_traj_data_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("visualization/back_traj_data",
                                                                                  100);
        back_sfc_data_pub_ = nh_.advertise<quadrotor_msgs::PolyhedronVector>("visualization/back_sfc_data", 100);

        receding_sfc_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/receding_sfc", 100, true);
        backup_traj_star_point_ = nh_.advertise<visualization_msgs::MarkerArray>(
                "visualization/backup_traj_start_pt", 100, true);
        yaw_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/yaw_traj", 100, true);
    }

    void Visualizer::Init(const ros::NodeHandle &nh, const PlannerConfig cfg) {
        cfg_ = cfg;
        Init(nh);
    }

    void Visualizer::ClearMarker(const PublisherName &name) {
        switch (name) {
            case GOAL_PUB: {
                VisualUtils::DeleteMkrArr(goal_pub_);
                break;
            }
            case BACKUP_SFC_PUB: {
                VisualUtils::DeleteMkrArr(backup_sfc_pub_);
                break;
            }
            case BACKUP_TRAJ_PUB: {
                VisualUtils::DeleteMkrArr(backup_traj_pub_);
                break;
            }
            case EXP_TRAJ_PUB: {
                VisualUtils::DeleteMkrArr(exp_traj_pub_);
                break;
            }
            case COMMIT_TRAJ_PUB: {
                VisualUtils::DeleteMkrArr(commit_traj_pub_);
                break;
            }
            case RECEDING_TRAJ_PUB: {
                VisualUtils::DeleteMkrArr(receding_traj_pub_);
                break;
            }
            case EXP_SFCS_PUB: {
                VisualUtils::DeleteMkrArr(exp_sfcs_pub_);
                break;
            }
            case POINT_PUB: {
                VisualUtils::DeleteMkrArr(point_pub_);
                break;
            }
            case FOV_PUB: {
                VisualUtils::DeleteMkrArr(fov_pub_);
                break;
            }
            case ASTAR_PUB: {
                VisualUtils::DeleteMkrArr(astar_pub_);
                break;
            }

        }
    }

    void Visualizer::VisualizeCommitTrajectory(const Trajectory &commit_traj, const double &back_start_TT) {
        double traj_dur = commit_traj.getTotalDuration();
        ClearMarker(COMMIT_TRAJ_PUB);
        if (back_start_TT > 0 && back_start_TT < traj_dur) {
            Trajectory exp_traj;
            if (!commit_traj.getPartialTrajectoryByTime(0, back_start_TT, exp_traj)) {
                ROS_ERROR("Failed to get partial trajectory");
                return;
            }
            Trajectory backup_traj;
            if (!commit_traj.getPartialTrajectoryByTime(back_start_TT, traj_dur, backup_traj)) {
                ROS_ERROR("Failed to get partial trajectory");
                return;
            }
            exp_traj.Visualize(commit_traj_pub_, "commit_exp", Color::SteelBlue(), 0.08, true, false);
            backup_traj.Visualize(commit_traj_pub_, "commit_backup", Color::Green(), 0.1, false, false);
        } else {
            commit_traj.Visualize(commit_traj_pub_, "commit_exp", Color::Green(), 0.1, true, false);
        }
    }


    void Visualizer::VisualizeSafeCorridorSeed(const vec_Vec3f &seed) {
        VisualUtils::VisualizePath(backup_sfc_pub_, seed, Color::Red(), "backup_sfc_seed", 0.3, 0.15);
    }

    void Visualizer::VisualizeGuidePath(const vec_E<Vec3f> &path) {
        VisualUtils::DeleteMkrArr(guide_path_pub_);
        VisualUtils::VisualizePath(guide_path_pub_, path, Color::Pink(), "guide_path", 0.3, 0.15);
    }

    void Visualizer::VisualizeGuidePathSegs(const vector<vec_E<Vec3f>> &path_segs) {
        VisualUtils::DeleteMkrArr(astar_pub_);
        static std::random_device rd;
        static std::mt19937 mt(rd());
        static std::uniform_real_distribution<double> dist(0.0, 1.0);

        for (auto &path: path_segs) {
            Color random_color(dist(mt), dist(mt), dist(mt));
            VisualUtils::VisualizePath(astar_pub_, path, random_color, "astar", 0.1, 0.05);
        }
    }

    void Visualizer::VisualizeGoalPath(const vec_E<Vec3f> &path) {
        VisualUtils::DeleteMkrArr(goal_pub_);
        VisualUtils::VisualizePath(goal_pub_, path, Color::Pink(), "goal", 0.3, 0.15);
    }

    void Visualizer::VisualizeGoal(const Vec3f &local_goal, const Vec3f &global_goal) {
        VisualUtils::DeleteMkrArr(goal_pub_);
        VisualUtils::VisualizePoint(goal_pub_, local_goal, Color::Pink(), "local_goal", 0.3, 0);
        VisualUtils::VisualizePoint(goal_pub_, global_goal, Color::Green(), "global_goal", 0.3, 0);
    }

    void Visualizer::VisualizeBackupTrajSeedPointAndStartPoint(const Vec3f &seed_pt, const Vec3f &pt) {
        VisualUtils::VisualizePoint(backup_traj_star_point_, seed_pt, Color::Orange(), "back_traj_seed_pt", 0.2, 0);
        VisualUtils::VisualizePoint(backup_traj_star_point_, pt, Color::Chartreuse(), "back_traj_start_pt", 0.2, 0);
    }

    void Visualizer::VisualizeDroneFOV(vec_E<Mat3f> &meshes) {
        VisualUtils::VisualizeMeshes(fov_pub_, meshes, Color::SteelBlue(), "fov");
    }

    void Visualizer::VisualizeRecedingTrajectory(const vector<TimePosPair> &evaluated_traj) {
        if (receding_traj_pub_.getNumSubscribers() <= 0) {
            return;
        }
        if (evaluated_traj.empty()) {
            return;
        }
        vec_E<Vec3f> receding_traj;
        receding_traj.reserve(evaluated_traj.size());
        for (auto p: evaluated_traj) {
            receding_traj.push_back(p.second);
        }
        VisualUtils::DeleteMkrArr(receding_traj_pub_);
        VisualUtils::VisualizeTrajectory(receding_traj_pub_, receding_traj, Color::SteelBlue(), 0.03,
                                         "receding_traj", 0.1);
    }

    void Visualizer::VisualizeSplitPoint(const Vec3f &split_pt) {
        VisualUtils::VisualizePoint(point_pub_, split_pt, Color::SteelBlue(), "split_pt", 0.3, 0);
    }

    void Visualizer::VisualizeReplanPoint(const Vec3f &split_pt) {
        VisualUtils::VisualizePoint(point_pub_, split_pt, Color::Green(), "replan_pt", 0.3, 0);
    }

    void Visualizer::VisualizeObstaclePoint(const Vec3f &split_pt) {
        VisualUtils::VisualizePoint(point_pub_, split_pt, Color::Red(), "obs_pt", 0.3, 0);
    }

    void Visualizer::VisualizeBackupTraj(const Trajectory &backup_traj) {
        if (backup_traj_pub_.getNumSubscribers() <= 0) {
            return;
        }
        VisualUtils::DeleteMkrArr(backup_traj_pub_);
        backup_traj.Visualize(backup_traj_pub_, "backup_traj", Color::Green(), 0.08, true, false);
    }

    void Visualizer::VisualizeExpTrajInColor(const Trajectory &color_traj) {
        if (exp_traj_pub_.getNumSubscribers() <= 0) {
            return;
        }
        VisualUtils::DeleteMkrArr(exp_traj_pub_);
        color_traj.Visualize(exp_traj_pub_, "exp_traj", Color::Green(), 0.05, true, true);
    }

    void Visualizer::VisualizeExpSfc(const PolytopeVec &sfcs) {
        if (exp_sfcs_pub_.getNumSubscribers() <= 0) {
            return;
        }
        VisualUtils::DeleteMkrArr(exp_sfcs_pub_);
        int color_num = sfcs.size();

        int color_id = 0;
        quadrotor_msgs::Polyhedron poly;
        quadrotor_msgs::PolyhedronVector vec_poly;
        for (auto p: sfcs) {
            double color_ratio = 1.0 - (double) color_id / color_num;
            Vec3f color_mag = tinycolormap::GetColor(color_ratio, tinycolormap::ColormapType::Jet).ConvertToEigen();
            color_id++;
            Color c(color_mag[0], color_mag[1], color_mag[2]);
            p.Visualize(exp_sfcs_pub_, "exp_sfc", false, Color::SteelBlue(), c, Color::Orange(), 0.15,
                        cfg_.resolution / 2);
            auto planes = p.GetPlanes();
            for (int k = 0; k < planes.rows(); k++) {
                auto face = planes.row(k);
                for (int i = 0; i < 4; i++)
                    poly.data.push_back(face[i]);
            }
            poly.face_num = planes.rows();
            vec_poly.polyhedrons.push_back(poly);
        }
        vec_poly.header.stamp = ros::Time::now();
        exp_sfc_data_pub_.publish(vec_poly);
    }

    void Visualizer::YawTrajDebugViz(const vec_Vec3f &pts1, const vec_Vec3f &pts2) {
        for (long unsigned int i = 0; i < pts1.size(); i++) {
            VisualUtils::VisualizePoint(yaw_pub_, pts1[i], Color::Green(), "yaw_traj_waypoint_a", 0.3);
            VisualUtils::VisualizePoint(yaw_pub_, pts2[i], Color::Red(), "yaw_traj_waypoint_b", 0.3);
        }
    }

    void Visualizer::VisualizeRecedingCorridor(PolytopeVec &sfcs) {
        VisualUtils::DeleteMkrArr(receding_sfc_pub_);
        for (auto p: sfcs) {
            p.Visualize(receding_sfc_pub_, "receding_sfc", false, Color::SteelBlue(), Color::Black(), Color::Blue(),
                        0.15,
                        cfg_.resolution / 2);
        }
    }

    void Visualizer::VisualizeFreePoly(Polytope &backup_sfc) {
        if (backup_sfc_pub_.getNumSubscribers() <= 0) {
            return;
        }
        VisualUtils::DeleteMkrArr(backup_sfc_pub_);
        backup_sfc.Visualize(backup_sfc_pub_, "backup_sfc", false, Color::Chartreuse(), Color::Green(),
                             Color::Green(),
                             0.15,
                             cfg_.resolution / 2);
        quadrotor_msgs::Polyhedron poly;
        quadrotor_msgs::PolyhedronVector vec_poly;
        auto planes = backup_sfc.GetPlanes();
        for (int k = 0; k < planes.rows(); k++) {
            auto face = planes.row(k);
            for (int i = 0; i < 4; i++)
                poly.data.push_back(face[i]);
        }
        poly.face_num = planes.rows();
        vec_poly.polyhedrons.push_back(poly);
        vec_poly.header.stamp = ros::Time::now();
        back_sfc_data_pub_.publish(vec_poly);
    }

    void Visualizer::VisualizeYawTraj(const Trajectory &pos_traj, const Trajectory &yaw_traj) {
        VisualUtils::DeleteMkrArr(yaw_pub_);
        VisualUtils::VisualizeYawTrajectory(yaw_pub_, pos_traj, yaw_traj);
    }
}
