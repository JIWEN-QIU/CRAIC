#pragma once

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <traj_opt/config.h>
#include <traj_opt/minco.h>
#include <math_utils/lbfgs.h>
#include <benchmark_utils/scope_timer.h>
#include <geometry_utils/geometry_utils.h>
#include <type_utils/common_type_name.h>
#include <geometry_utils/polytope.h>
#include <visualization_utils/visualization_utils.h>
#include <optimization_utils/optimization_utils.h>


namespace traj_opt {
    using namespace benchmark_utils;
    using namespace visualization_utils;
    using namespace optimization_utils;
    using std::cout;
    using std::endl;
    using std::string;

    class ExpTrajOpt {
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

            PolyhedraV vPolytopes; // 原始走廊+相交走廊
            PolyhedraH hPolytopes; // 原始走廊
            PolyhedraH hOverlapPolytopes;
            Eigen::Matrix3Xd init_path;
            Eigen::Matrix3Xd waypoint_attractor;
            Eigen::VectorXd waypoint_attractor_dead_d;

            Eigen::VectorXi pieceIdx;
            Eigen::VectorXi vPolyIdx;
            Eigen::VectorXi hPolyIdx;

            MINCO_S4NU minco;

            StatePVAJ headPVAJ;
            StatePVAJ tailPVAJ;
            vec_E<Vec3f> guide_path;
            vector<double> guide_t;

            int temporalDim, spatialDim;

            VectorXd penalty_log;
        } opt_vars;

    private:
        /// Optimization functions
        static double costFunctional(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g);

        static void constraintsFunctional(const Eigen::VectorXd &T,
                                            const Eigen::MatrixX3d &coeffs,
                                            const Eigen::VectorXi &hIdx,
                                            const PolyhedraH &hPolys,
                                            const Eigen::Matrix3Xd &waypoint_attractor,
                                            const Eigen::VectorXd &waypoint_attractor_dead_d,
                                            const double &smoothFactor,
                                            const int &integralResolution,
                                            const Eigen::VectorXd &magnitudeBounds,
                                            const Eigen::VectorXd &penaltyWeights,
                                            flatness::FlatnessMap &flatMap,
                                            double &cost,
                                            Eigen::VectorXd &gradT,
                                            Eigen::MatrixX3d &gradC,
                                            VectorXd &penalty_log);

        bool processCorridor();

        bool processCorridorWithGuideTraj();

        void defaultInitialization();

        bool setupProblemAndCheck();

        bool setInitPsAndTs(vec_E<Eigen::Vector3d> &init_ps, vector<double> &init_ts);

        static bool SimplifySFC(const Vec3f &head_p, const Vec3f &tail_p, PolytopeVec &sfcs);


        double optimize(Trajectory &traj, const double &relCostTol);

    public:
        typedef std::shared_ptr<ExpTrajOpt> Ptr;

        ExpTrajOpt(const ros::NodeHandle &nh, const TrajOptConfig &cfg);

        ~ExpTrajOpt();

        bool checkTrajMagnituteBound(Trajectory &out_traj);

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      const vec_E<Vec3f> & guide_path, const vector<double> & guide_t,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);
    };
}
