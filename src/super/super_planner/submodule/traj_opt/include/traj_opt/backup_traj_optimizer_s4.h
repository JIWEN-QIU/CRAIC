#pragma once

#include <traj_opt/config.h>
#include <traj_opt/minco.h>
#include <math_utils/lbfgs.h>
#include <benchmark_utils/scope_timer.h>
#include <geometry_utils/geometry_utils.h>
#include <type_utils/common_type_name.h>
#include <geometry_utils/polytope.h>
#include <visualization_utils/visualization_utils.h>
#include <memory>
#include <optimization_utils/optimization_utils.h>


namespace traj_opt {
    using namespace type_utils;
    using namespace geometry_utils;
    using namespace math_utils;
    using namespace visualization_utils;
    using namespace optimization_utils;

    class BackupTrajOpt {
    private:
        TrajOptConfig cfg_;
        ros::Publisher mkr_pub_;
        ros::NodeHandle nh_;
        std::ofstream failed_traj_log;
        std::ofstream penalty_log;

        struct OptimizationVariables {
            double rho;
            int iter_num{0};
            int pos_constraint_type;
            bool block_energy_cost;
            double smooth_eps;
            int integral_res;
            flatness::FlatnessMap quadrotor_flatness;

            Eigen::Matrix3Xd gradByPoints;
            Eigen::VectorXd gradByTimes;
            Eigen::MatrixX3d partialGradByCoeffs;
            Eigen::VectorXd partialGradByTimes;
            bool default_init{true};
            int piece_num;
            Eigen::Matrix3Xd points;
            Eigen::VectorXd times;
            Eigen::VectorXd magnitudeBounds, penaltyWeights;

            Eigen::Matrix3Xd init_path;
            Eigen::Matrix3Xd waypoint_attractor;
            Eigen::VectorXd waypoint_attractor_dead_d;

            PolyhedronH hPolytope;
            PolyhedronV vPolytope;

            MINCO_S4NU minco;

            StatePVAJ headPVAJ;
            StatePVAJ tailPVAJ;
            vec_E<Vec3f> guide_path;
            vector<double> guide_t;

            int temporalDim, spatialDim;

            VectorXd penalty_log;

            bool debug_en;
            double ts;
            double gradTs;
            ros::Publisher debug_pub;
            Trajectory exp_traj;
            double min_ts, max_ts;
            double weight_ts;

        } opt_vars{};

    private:
        /// Optimization functions
        static double costFunctional(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g);

        static void constraintsFunctional(const Eigen::VectorXd &T,
                                          const Eigen::MatrixX3d &coeffs,
                                          const PolyhedronH &hPoly,
                                          const double &smoothFactor,
                                          const int &integralResolution,
                                          const Eigen::VectorXd &magnitudeBounds,
                                          const Eigen::VectorXd &penaltyWeights,
                                          flatness::FlatnessMap &flatMap,
                                          double &cost,
                                          Eigen::VectorXd &gradT,
                                          Eigen::MatrixX3d &gradC,
                                          VectorXd &pena_log);

        bool processCorridor();

        static int
        visualizeProgress(void *instance, const Eigen::VectorXd &x, const Eigen::VectorXd &g, const double fx,
                          const double step, const int k, const int ls);

        void defaultInitialization();

        bool setupProblemAndCheck();

        static bool SimplifySFC(const Vec3f &head_p, const Vec3f &tail_p, PolytopeVec &sfcs);


        double optimize(Trajectory &traj, const double &relCostTol);

    public:

        explicit BackupTrajOpt(const ros::NodeHandle &nh, TrajOptConfig &cfg);

        ~BackupTrajOpt() {
            penalty_log.close();
        }

        typedef std::shared_ptr<BackupTrajOpt> Ptr;

        bool checkTrajMagnitudeBound(Trajectory &out_traj);

        bool optimize(const Trajectory &exp_traj,
                      const double &t_0,
                      const double &t_e,
                      const double &heu_ts,
                      const Vector3d &heu_end_pt,
                      double &heu_dur,
                      Polytope &sfc,
                      Trajectory &out_traj,
                      double &out_ts,
                      const bool &debug = false);

    };
}
