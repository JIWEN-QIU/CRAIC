#include <super_core/control_cmd_gene.h>

namespace super_planner {

    void ControlCommandGenerator::GetOneCommandFromTraj(Trajectory &traj, const double eval_t,
                                                        quadrotor_msgs::PositionCommand &cmd, bool emer) {
        std::pair<double, double> yyd;
        Vec3f p = traj.getPos(eval_t);
        Vec3f v = traj.getVel(eval_t);
        Vec3f a = traj.getAcc(eval_t);
        Vec3f j = traj.getJer(eval_t);
        static int traj_id = 0;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        if (emer) {
            cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMER;
        } else {
            cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        }
        cmd.trajectory_id = traj_id++;
        cmd.vel_norm = v.norm();
        cmd.acc_norm = a.norm();
        cmd.position.x = p(0);
        cmd.position.y = p(1);
        cmd.position.z = p(2);
        cmd.velocity.x = v(0);
        cmd.velocity.y = v(1);
        cmd.velocity.z = v(2);
        cmd.acceleration.x = a(0);
        cmd.acceleration.y = a(1);
        cmd.acceleration.z = a(2);
        cmd.jerk.x = j(0);
        cmd.jerk.y = j(1);
        cmd.jerk.z = j(2);
//        cmd.jerk.x = 0.0;
//        cmd.jerk.y = 0.0;
//        cmd.jerk.z = 0.0;
        cmd.yaw = yyd.first;
        cmd.yaw_dot = yyd.second;
        static Vec3f rpy, omg;
        static double aT;
        geometry_utils::convertFlatOutputToAttAndOmg(p, v, a, j, yyd.first, yyd.second, rpy, omg, aT);
        cmd.attitude.x = rpy(0);
        cmd.attitude.y = rpy(1);
        cmd.attitude.z = rpy(2);
        cmd.angular_velocity.x = omg(0);
        cmd.angular_velocity.y = omg(1);
        cmd.angular_velocity.z = omg(2);
        cmd.thrust.z = aT;
    }

    void ControlCommandGenerator::GetOneMpcCommandFromTraj(const Trajectory &traj, const double eval_t,
                                                           quadrotor_msgs::MpcPositionCommand &cmd, bool emer) {
        double total_dur = traj.getTotalDuration();
        cmd.mpc_horizon = 8;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        static int traj_id = 0;
        constexpr double mpc_dt = 0.01;
        for (int i = 0; i < 8; i++) {
            double local_t = eval_t + mpc_dt * i;
            local_t = local_t > total_dur ? total_dur : local_t;
            Vec3f p = traj.getPos(local_t);
            Vec3f v = traj.getVel(local_t);
            Vec3f a = traj.getAcc(local_t);
            Vec3f j = traj.getJer(local_t);
            if (emer) {
                cmd.cmds[i].trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMER;
            } else {
                cmd.cmds[i].trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
            }
            cmd.cmds[i].trajectory_id = traj_id;
            cmd.cmds[i].vel_norm = v.norm();
            cmd.cmds[i].acc_norm = a.norm();
            cmd.cmds[i].position.x = p(0);
            cmd.cmds[i].position.y = p(1);
            cmd.cmds[i].position.z = p(2);
            cmd.cmds[i].velocity.x = v(0);
            cmd.cmds[i].velocity.y = v(1);
            cmd.cmds[i].velocity.z = v(2);
            cmd.cmds[i].acceleration.x = a(0);
            cmd.cmds[i].acceleration.y = a(1);
            cmd.cmds[i].acceleration.z = a(2);
            cmd.cmds[i].jerk.x = j(0);
            cmd.cmds[i].jerk.y = j(1);
            cmd.cmds[i].jerk.z = j(2);
        }
        traj_id++;
    }

}