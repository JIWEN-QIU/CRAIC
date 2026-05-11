#pragma once

#include <memory>
#include <quadrotor_msgs/MincoTrajectory.h>
#include <quadrotor_msgs/MpcPositionCommand.h>
#include <geometry_utils/trajectory.h>
#include <type_utils/common_type_name.h>
#include <super_core/visualizer.h>
#include <geometry_utils/trajectory.h>



namespace super_planner {
    using namespace geometry_utils;
    using namespace std;
    using namespace type_utils;

    class ControlCommandGenerator {
    public:

        void GetOneCommandFromTraj(Trajectory &traj, const double eval_t,
                                   quadrotor_msgs::PositionCommand &cmd,
                                   bool emer = false);


        void GetOneMpcCommandFromTraj(const Trajectory &traj, const double eval_t,
                                      quadrotor_msgs::MpcPositionCommand &cmd, bool emer = false);
    };
}