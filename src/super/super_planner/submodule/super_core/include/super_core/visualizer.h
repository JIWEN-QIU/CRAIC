#pragma once

#include <visualization_utils/visualization_utils.h>

#define TINYCOLORMAP_WITH_EIGEN

#include <visualization_utils/tinycolormap.hpp>
#include <geometry_utils/polytope.h>
#include <geometry_utils/trajectory.h>
#include <super_core/config.h>
#include <quadrotor_msgs/PolyhedronVector.h>
#include <quadrotor_msgs/Polyhedron.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>

namespace super_planner {
    using namespace type_utils;
    using namespace geometry_utils;
    using namespace visualization_utils;

    class Visualizer {
    private:
        ros::NodeHandle nh_;
        PlannerConfig cfg_;
    public:
        ros::Publisher goal_pub_, backup_sfc_pub_, backup_traj_pub_, commit_traj_pub_,
                receding_traj_pub_, exp_sfcs_pub_, point_pub_, fov_pub_,
                exp_traj_pub_, astar_pub_, receding_sfc_pub_, backup_traj_star_point_, yaw_pub_, guide_path_pub_;


        ros::Publisher exp_sfc_data_pub_, back_sfc_data_pub_, exp_traj_data_pub_, back_traj_data_pub_;

        enum PublisherName{
            GOAL_PUB,
            BACKUP_SFC_PUB,
            BACKUP_TRAJ_PUB,
            COMMIT_TRAJ_PUB,
            RECEDING_TRAJ_PUB,
            EXP_SFCS_PUB,
            POINT_PUB,
            FOV_PUB,
            EXP_TRAJ_PUB,
            ASTAR_PUB,
            RECEDING_SFC_PUB,
            BACKUP_TRAJ_STAR_POINT,
            YAW_PUB,
            GUIDE_PATH_PUB,
            EXP_SFC_DATA_PUB,
            BACK_SFC_DATA_PUB,
            EXP_TRAJ_DATA_PUB,
            BACK_TRAJ_DATA_PUB
        };

    public:


        Visualizer() = default;

        ~Visualizer() = default;

        std::shared_ptr<Visualizer> Ptr;

        Visualizer(const ros::NodeHandle &nh, const PlannerConfig cfg);

        void Init(const ros::NodeHandle &nh);

        void Init(const ros::NodeHandle &nh, const PlannerConfig cfg);

        void ClearMarker(const PublisherName & name);

        void VisualizeSafeCorridorSeed(const vec_Vec3f &seed);

        void VisualizeGuidePath(const vec_E<Vec3f> &path);

        void VisualizeCommitTrajectory(const Trajectory &commit_traj, const double & back_start_TT);

        void VisualizeGuidePathSegs(const vector <vec_E<Vec3f>> &path_segs);

        void VisualizeGoalPath(const vec_E<Vec3f> &path);

        void VisualizeGoal(const Vec3f &local_goal, const Vec3f &global_goal);

        void VisualizeBackupTrajSeedPointAndStartPoint(const Vec3f &seed_pt, const Vec3f &pt);

        void VisualizeDroneFOV(vec_E<Mat3f> &meshes);

        void VisualizeRecedingTrajectory(const vector <TimePosPair> &evaluated_traj);

        void VisualizeSplitPoint(const Vec3f &split_pt);

        void VisualizeReplanPoint(const Vec3f &split_pt);

        void VisualizeObstaclePoint(const Vec3f &split_pt);

        void VisualizeBackupTraj(const Trajectory &backup_traj);

        void VisualizeExpTrajInColor(const Trajectory &color_traj);

        void VisualizeExpSfc(const PolytopeVec &sfcs);

        void YawTrajDebugViz(const vec_Vec3f &pts1, const vec_Vec3f &pts2);

        void VisualizeRecedingCorridor(PolytopeVec &sfcs);

        void VisualizeFreePoly(Polytope &backup_sfc);

        void VisualizeYawTraj(const Trajectory &pos_traj, const Trajectory &yaw_traj);

    };
}