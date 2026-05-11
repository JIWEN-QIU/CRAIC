#pragma once

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include "math_utils/lbfgs.h"
#include "benchmark_utils/scope_timer.h"
#include "geometry_utils/geometry_utils.h"
#include "type_utils/common_type_name.h"
#include "geometry_utils/polytope.h"
#include "visualization_utils/visualization_utils.h"
#include "traj_opt/config.h"
#include <optimization_utils/optimization_utils.h>
#include <optimization_utils/minco.h>
#include "geometry_utils/polynomial_interpolation.h"

namespace traj_opt {
    using namespace geometry_utils;
    using namespace visualization_utils;
    using namespace type_utils;
    using namespace benchmark_utils;
    using namespace optimization_utils;
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class YawTrajOpt {
    private:
        ros::NodeHandle nh_;
        bool free_goal_{false};
        double yaw_dot_max_{10};

    public:

        YawTrajOpt(const double &_yaw_dot_max);

        typedef std::shared_ptr<YawTrajOpt> Ptr;

        void getYawTimeAllocation(const double &duration, VecDf &times);

        void getYawWaypointAllocation(const Vec4f &init_state, Vec4f &goal_state, VecDf &way_pts, VecDf &times,
                                      const Trajectory &pos_traj);

        bool optimize(const Vec4f &istate_in,
                      const Vec4f &gstate_in,
                      const Trajectory &pos_traj,
                      Trajectory &out_traj,
                      const int order = 3,
                      const bool &free_start = false,
                      const bool &free_goal = true);

    };


}