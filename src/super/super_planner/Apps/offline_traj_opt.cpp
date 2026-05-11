#include <geometry_utils/trajectory.h>
#include <optimization_utils/optimization_utils.h>
#include <optimization_utils/waypoint_trajectory_optimizer.h>
#include <ros/ros.h>
#include <visualization_utils/visualization_utils.h>
#include <super_core/ciri.h>
#include <traj_opt/exp_traj_optimizer_s4.h>
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "traj_opt/yaw_traj_opt.h"

using namespace type_utils;
using namespace geometry_utils;
using namespace visualization_utils;
using namespace optimization_utils;
using namespace std;
using namespace super_planner;
using namespace traj_opt;
ros::Publisher mkrarr_pub, back_traj_pub;

ExpTrajOpt::Ptr optimizer;
BackupTrajOpt::Ptr backup_optimizer;
YawTrajOpt::Ptr yaw_optimizer;
PolytopeVec sfcs;

double str2double(string s) {
    double d;
    stringstream ss;
    ss << s;
    ss >> setprecision(16) >> d;
    ss.clear();
    return d;
}

bool LoadOfflinePlanningProblemTraj(TrajOptConfig &cfg, bool shift_to_oringin = false) {
//    string package_path_ = ros::package::getPath("simple_traj_generator");
    bool have_guide_time{false};
    string file_name = "/home/yunfan/SUPER/Workspace/catkin_ws/src/super/svc_planner/super_planner/log/replan_opt_prob.csv";//package_path_+"/data/aaaa.txt";
    ifstream theFile(file_name);
    cout << "LoadOfflinePlanningProblemTraj" << endl;
    std::string line;
    Trajectory new_traj;
    bool finish = false;
    std::getline(theFile, line);
    std::vector<std::string> result;
    std::istringstream iss(line);
    for (std::string s; iss >> s;) {
        result.push_back(s);
    }
    int first_num = str2double(result[0]);
    cout << "Load first_num: " << first_num << endl;
    if (std::abs(first_num - 990419) < 0.0001) {
        StatePVAJ headPVAJ, tailPVAJ;
        int row_id = 0;
        std::vector<std::string> result;
        while (std::getline(theFile, line)) {
            cout << "Load new data" << endl;
            std::istringstream iss(line);
            result.clear();
            for (std::string s; iss >> s;) {
                result.push_back(s);
            }
            if (row_id < 3) {
                headPVAJ(row_id, 0) = str2double(result[0]);
                headPVAJ(row_id, 1) = str2double(result[1]);
                headPVAJ(row_id, 2) = str2double(result[2]);
                headPVAJ(row_id, 3) = str2double(result[3]);
            } else {
                tailPVAJ(row_id - 3, 0) = str2double(result[0]);
                tailPVAJ(row_id - 3, 1) = str2double(result[1]);
                tailPVAJ(row_id - 3, 2) = str2double(result[2]);
                tailPVAJ(row_id - 3, 3) = str2double(result[3]);
            }
            row_id++;
            if (row_id == 6) {
                break;
            }
        }
        Vec3f shift_offset = headPVAJ.col(0);
        if (shift_to_oringin) {
            headPVAJ.col(0) -= shift_offset;
            tailPVAJ.col(0) -= shift_offset;
        }
//        tailPVAJ.col(0) += Vec3f(-0.2, 0, -0.2);
        VisualUtils::VisualizePoint(mkrarr_pub, headPVAJ.col(0), Color::Orange(), "start_state", 0.2, 1);
        VisualUtils::VisualizePoint(mkrarr_pub, tailPVAJ.col(0), Color::Green(), "goal_state", 0.2, 1);

        // ignore guide line
        std::getline(theFile, line);
        std::istringstream iss(line);
        result.clear();
        for (std::string s; iss >> s;) {
            result.push_back(s);
        }
        vector<double> guide_stamp;
        vec_E<Vec3f> guide_path;
        if (result.size() == 1) {

        } else {
            have_guide_time = true;
            guide_stamp.resize(result.size());
            for (int i = 0; i < result.size(); i++) {
                guide_stamp[i] = str2double(result[i]);
            }
            // ignore guitde pose
            std::getline(theFile, line);
            iss = std::istringstream(line);
            result.clear();
            for (std::string s; iss >> s;) {
                result.push_back(s);
            }
            cout << "Path length: " << result.size() / 3 << endl;
            guide_path.resize(int(result.size() / 3));
            for (int i = 0; i < result.size(); i += 3) {
                guide_path[i / 3](0) = str2double(result[i]);
                guide_path[i / 3](1) = str2double(result[i + 1]);
                guide_path[i / 3](2) = str2double(result[i + 2]);
            }

            for (long unsigned int i = 0; i < guide_stamp.size(); i++) {
                cout << guide_stamp[i] << " ";
            }
            cout << endl;
            for (long unsigned int i = 0; i < guide_path.size(); i++) {
                cout << guide_path[i].transpose() << " ";
            }
            cout << endl;
            // get sfc id
            std::getline(theFile, line);
            iss = std::istringstream(line);
            result.clear();
            for (std::string s; iss >> s;) {
                result.push_back(s);
            }
        }


        cout << "SFC ID: " << result[0] << endl;
        vec_E<Vec4f> sfc_planes;
        while (!finish) {
            if (!std::getline(theFile, line)) {
                if (sfc_planes.empty()) {
                    finish = true;
                    break;
                }
                MatD4f planes(sfc_planes.size(), 4);
                for (int j = 0; j < sfc_planes.size(); j++) {
                    if (!shift_to_oringin) {
                        planes.row(j) = sfc_planes[j];
                    } else {
                        planes.row(j) = geometry_utils::translatePlane(sfc_planes[j], -shift_offset);
                    }
                }
                cout << planes << endl;
                Polytope poly(planes);
                sfcs.push_back(poly);
                finish = true;
                sfc_planes.clear();
                break;
            }
            std::istringstream iss(line);
            result.clear();
            for (std::string s; iss >> s;) {
                result.push_back(s);
            }

            if (result.size() == 1 || result.size() == 0) {
                if (str2double(result[0]) == 990419 || result.size() == 0) {
                    finish = true;
                    break;
                } else {
                    cout << "SFC ID: " << result[0] << endl;
                    cout << "sfcs.size()" << sfcs.size() << endl;
                    MatD4f planes(sfc_planes.size(), 4);
                    for (int j = 0; j < sfc_planes.size(); j++) {
                        if (!shift_to_oringin) {
                            planes.row(j) = sfc_planes[j];
                        } else {
                            planes.row(j) = geometry_utils::translatePlane(sfc_planes[j], -shift_offset);
                        }

                    }
                    cout << planes << endl;
                    Polytope poly(planes);
                    sfcs.push_back(poly);
                    sfc_planes.clear();
                }
            } else if (result.size() == 4) {
                sfc_planes.push_back(Vec4f(str2double(result[0]), str2double(result[1]), str2double(result[2]),
                                           str2double(result[3])));
            }
        }

        int cnt = 0;
        vec_Vec3f path{headPVAJ.col(0), tailPVAJ.col(0)};

        for (auto poly: sfcs) {
            cout << GREEN << "Try visualize:" << cnt++ << RESET << endl;
            cout << poly.GetPlanes() << endl;
            poly.Visualize(mkrarr_pub, "sfc", true, Color::SteelBlue(), Color::Black(), Color::Orange(), 0.15,
                           0.02);
            ros::Duration(0.05).sleep();
        }
        cout << BLUE << "Boundary conditions:" << RESET << endl;
        cout << headPVAJ << endl;
        cout << tailPVAJ << endl;

        if (!sfcs.front().PointIsInside(headPVAJ.col(0))) {
            cout << RED << "Start state is not inside the sfc" << RESET << endl;
        }
        if (!sfcs.back().PointIsInside(tailPVAJ.col(0))) {
            cout << RED << "Goal state is not inside the sfc" << RESET << endl;
        }


        Trajectory out_traj;
        {
            if (have_guide_time) {
                cout << BLUE << "Solve with guide:" << RESET << endl;
                VisualUtils::VisualizePath(mkrarr_pub, guide_path, Color::Red(), "guide_path");
                TimeConsuming opt("Optimization time consuming");
                if (optimizer->optimize(headPVAJ, tailPVAJ, guide_path, guide_stamp, sfcs, out_traj)) {
                    opt.stop();
                    out_traj.Visualize(mkrarr_pub, "out_traj", Color::Yellow(), 0.1, true, true);
                } else {
                    opt.stop();
                    out_traj.Visualize(mkrarr_pub, "out_traj", Color::Yellow(), 0.1, true, false);
                }
                out_traj.printProfile();

                ofstream traj_profile;
                traj_profile.open(DEBUG_FILE_DIR("offline_traj_profile.csv"), std::ios::out | std::ios::trunc);
                double dur = out_traj.getTotalDuration();
                for (double t = 0.0; t < dur; t = t + 0.01) {
                    Vec3f p, v, a, j, s;
                    p = out_traj.getPos(t);
                    v = out_traj.getVel(t);
                    a = out_traj.getAcc(t);
                    j = out_traj.getJer(t);
                    s = out_traj.getSnap(t);
                    Vec3f omg;
                    double thr;
                    Vec4f quat;
                    cfg.quadrotot_flatness.forward(v, a, j, 0, 0, thr, quat, omg);

                    static const Vec3f grav = 9.80f * Vec3f(0, 0, 1);
                    double aT = (grav + a).norm();
                    Vec3f xB, yB, zB;
                    Vec3f xC(cos(0), sin(0), 0);
                    zB = (grav + a).normalized();
                    yB = ((zB).cross(xC)).normalized();
                    xB = yB.cross(zB);
                    Mat3f R;
                    R << xB, yB, zB;
                    Quatf q(R);
                    Vec3f eular_wo_drag = geometry_utils::quaternion_to_ypr(q);
                    Vec3f eular = geometry_utils::quaternion_to_ypr(Quaterniond(quat[0], quat[1], quat[2], quat[3]));

                    traj_profile << t << " " << p.transpose() << " " << v.transpose() << " " << a.transpose() << " "
                                 << j.transpose() << " " << s.transpose() << " " << omg.transpose() << " "
                                 << eular.transpose() << " " << eular_wo_drag.transpose() << " "
                                 << thr / cfg.mass << endl;
                }
                traj_profile.close();
            } else {
                cout << BLUE << "Solve w/o guide:" << RESET << endl;
                TimeConsuming opt("Optimization time consuming");
                if (optimizer->optimize(headPVAJ, tailPVAJ, sfcs, out_traj)) {
                    opt.stop();
                    out_traj.Visualize(mkrarr_pub, "out_traj", Color::Yellow(), 0.1, true, true);
                } else {
                    opt.stop();
                    out_traj.Visualize(mkrarr_pub, "out_traj", Color::Yellow(), 0.1, true, true);
                }
                out_traj.printProfile();
                ofstream traj_profile;
                traj_profile.open(DEBUG_FILE_DIR("offline_traj_profile.csv"), std::ios::out | std::ios::trunc);
                double dur = out_traj.getTotalDuration();
                for (double t = 0.0; t < dur; t = t + 0.01) {
                    Vec3f p, v, a, j, s;
                    p = out_traj.getPos(t);
                    v = out_traj.getVel(t);
                    a = out_traj.getAcc(t);
                    j = out_traj.getJer(t);
                    s = out_traj.getSnap(t);
                    Vec3f omg;
                    double thr;
                    Vec4f quat;
                    cfg.quadrotot_flatness.forward(v, a, j, 0, 0, thr, quat, omg);

                    static const Vec3f grav = 9.80f * Vec3f(0, 0, 1);
                    double aT = (grav + a).norm();
                    Vec3f xB, yB, zB;
                    Vec3f xC(cos(0), sin(0), 0);
                    zB = (grav + a).normalized();
                    yB = ((zB).cross(xC)).normalized();
                    xB = yB.cross(zB);
                    Mat3f R;
                    R << xB, yB, zB;
                    Quatf q(R);
                    Vec3f eular_wo_drag = geometry_utils::quaternion_to_ypr(q);
                    Vec3f eular = geometry_utils::quaternion_to_ypr(Quaterniond(quat[0], quat[1], quat[2], quat[3]));

                    traj_profile << t << " " << p.transpose() << " " << v.transpose() << " " << a.transpose() << " "
                                 << j.transpose() << " " << s.transpose() << " " << omg.transpose() << " "
                                 << eular.transpose() << " " << eular_wo_drag.transpose() << " "
                                 << thr / cfg.mass << endl;
                }
                traj_profile.close();
            }
        }

    } else if (std::abs(first_num - 123321 < 1e-5)) {

        std::vector<std::string> result;
        result.clear();
        std::getline(theFile, line);
        iss = std::istringstream(line);
        for (std::string s; iss >> s;) {
            result.push_back(s);
        }
        double t0 = str2double(result[0]);

        result.clear();
        std::getline(theFile, line);
        iss = std::istringstream(line);
        for (std::string s; iss >> s;) {
            result.push_back(s);
        }
        double te = str2double(result[0]);

        result.clear();
        std::getline(theFile, line);
        iss = std::istringstream(line);
        for (std::string s; iss >> s;) {
            result.push_back(s);
        }
        double heu_ts = str2double(result[0]);

        result.clear();
        std::getline(theFile, line);
        iss = std::istringstream(line);
        for (std::string s; iss >> s;) {
            result.push_back(s);
        }
        Vec3f heu_pt(str2double(result[0]), str2double(result[1]), str2double(result[2]));

        result.clear();
        std::getline(theFile, line);
        iss = std::istringstream(line);
        for (std::string s; iss >> s;) {
            result.push_back(s);
        }
        double heu_dur = str2double(result[0]);

        cout << "t0: " << t0 << endl;
        cout << "te: " << te << endl;
        cout << "heu_ts: " << heu_ts << endl;
        cout << "heu_pt: " << heu_pt.transpose() << endl;
        cout << "heu_dur: " << heu_dur << endl;

        /// Read the corridor
        std::getline(theFile, line);
        vec_E<Vec4f> planes;
        int pieceN;
        while (1) {
            result.clear();
            std::getline(theFile, line);
            iss = std::istringstream(line);
            for (std::string s; iss >> s;) {
                result.push_back(s);
            }
            if (result.size() == 4) {
                planes.push_back(Vec4f(str2double(result[0]), str2double(result[1]), str2double(result[2]),
                                       str2double(result[3])));
            } else {
                pieceN = str2double(result[0]);
                break;
            }
        }


        // Read Trajectory
        cout << "PieceN: " << pieceN << endl;
        VecDf time_vec(pieceN);

        result.clear();
        std::getline(theFile, line);
        iss = std::istringstream(line);
        for (std::string s; iss >> s;) {
            result.push_back(s);
        }

        for (int i = 0; i < pieceN; i++) {
            time_vec(i) = str2double(result[i]);
        }
        cout << "time_vec: " << time_vec.transpose() << endl;
        //
        bool done{false};
        vec_E<Mat3Df> coeff_vec;
        while (!done) {
            Mat3Df coeff(3, 8);
            for (int i = 0; i < 3; i++) {
                result.clear();
                if (!std::getline(theFile, line)) {
                    done = true;
                    break;
                }
                iss = std::istringstream(line);
                for (std::string s; iss >> s;) {
                    result.push_back(s);
                }
                if (result.size() == 6 || result.size() == 8) {
                    if (result.size() == 6) {
                        coeff(i, 0) = str2double(result[0]);
                        coeff(i, 1) = str2double(result[1]);
                        coeff(i, 2) = str2double(result[2]);
                        coeff(i, 3) = str2double(result[3]);
                        coeff(i, 4) = str2double(result[4]);
                        coeff(i, 5) = str2double(result[5]);
                    } else {
                        coeff(i, 0) = str2double(result[0]);
                        coeff(i, 1) = str2double(result[1]);
                        coeff(i, 2) = str2double(result[2]);
                        coeff(i, 3) = str2double(result[3]);
                        coeff(i, 4) = str2double(result[4]);
                        coeff(i, 5) = str2double(result[5]);
                        coeff(i, 6) = str2double(result[6]);
                        coeff(i, 7) = str2double(result[7]);
                    }
                }
            }
            coeff_vec.push_back(coeff);
        }

        Vec3f off_set_pt = coeff_vec[0].block<3, 1>(0, 7);
        Trajectory exp_traj;
        for (int i = 0; i < time_vec.size(); i++) {
            if (shift_to_oringin) {
                coeff_vec[i].block<3, 1>(0, 7) -= off_set_pt;
            }
//            cout << coeff_vec[i] << endl;
            Piece pie(time_vec(i), coeff_vec[i]);
            exp_traj.emplace_back(pie);
        }
        MatD4f planes_E(planes.size(), 4);
        for (int j = 0; j < planes.size(); j++) {
            if (shift_to_oringin) {
                planes_E.row(j) = geometry_utils::translatePlane(planes[j], -off_set_pt);
            } else {
                planes_E.row(j) = planes[j];
            }
        }

        if (shift_to_oringin) {
            heu_pt -= off_set_pt;
        }

        Polytope corridor(planes_E);
        Vec3f init_tsp = exp_traj.getPos(heu_ts);
        ros::Duration(0.5).sleep();
        VisualUtils::VisualizePoint(mkrarr_pub, init_tsp, Color::Blue(), "init_tsp", 0.2);
        VisualUtils::VisualizePoint(mkrarr_pub, heu_pt, Color::Red(), "heu_pt", 0.2);
        corridor.Visualize(mkrarr_pub, "corridor", true);
        exp_traj.Visualize(mkrarr_pub, "exp", Color::Chartreuse(), 0.05, true, true);
        exp_traj.printProfile();

        {
            // Test yaw_traj_planning
            Vec4f istate, fstate;
            istate.setZero();
            fstate.setZero();
            istate[0] = 1.2;
            istate[0] = -1.2;
            Trajectory yaw_traj;
            yaw_optimizer->optimize(istate, fstate, exp_traj, yaw_traj, 3, true);
            yaw_optimizer->optimize(istate, fstate, exp_traj, yaw_traj, 5, true);
            yaw_optimizer->optimize(istate, fstate, exp_traj, yaw_traj, 7, true);
            VisualUtils::VisualizeYawTrajectory(mkrarr_pub, exp_traj, yaw_traj, "yaw_traj");
        }


        double opt_ts;
        Trajectory out_traj;
        {
            TimeConsuming opt("Optimization time consuming");
            bool temp_ret = backup_optimizer->optimize(exp_traj,
                                                       t0,
                                                       te,
                                                       heu_ts, heu_pt, heu_dur,
                                                       corridor,
                                                       out_traj, opt_ts, false);
//            bool temp_ret = nlopt_om->optimize(exp_traj,
//                                               t0,
//                                               te,
//                                               heu_ts, heu_pt, heu_dur,
//                                               corridor,
//                                               out_traj, opt_ts, false);
        }
        out_traj.printProfile();
        out_traj.Visualize(mkrarr_pub, "out", Color::Green(), 0.1, true, false);

        cout << opt_ts << ", ";
        int last_id = 0;
        for (double t = 0; t < out_traj.getTotalDuration(); t += 0.01) {
            double cur_t = t;
            int id = out_traj.locatePieceIdx(cur_t);
            if (id != last_id) {
                cout << 0 << ",";
                last_id = id;
            }
            cout << out_traj.getAcc(t).norm() << ", ";
        }
        cout << 0;
        cout << endl;
        last_id = 0;
        for (double t = 0; t < exp_traj.getTotalDuration(); t += 0.003) {
            double cur_t = t;
            int id = exp_traj.locatePieceIdx(cur_t);
            if (id != last_id) {
                cout << 0 << ",";
                last_id = id;
            }
            cout << exp_traj.getVel(t).norm() << ", ";
        }
        cout << 0;
        cout << endl;
        ofstream traj_profile;
        traj_profile.open(DEBUG_FILE_DIR("offline_back_traj_profile.csv"), std::ios::out | std::ios::trunc);
        double dur = out_traj.getTotalDuration();
        for (double t = 0.0; t < dur; t = t + 0.01) {
            Vec3f p, v, a, j, s;
            p = out_traj.getPos(t);
            v = out_traj.getVel(t);
            a = out_traj.getAcc(t);
            j = out_traj.getJer(t);
            s = out_traj.getSnap(t);
            Vec3f omg;
            double thr;
            Vec4f quat;
            cfg.quadrotot_flatness.forward(v, a, j, 0, 0, thr, quat, omg);

            static const Vec3f grav = 9.80f * Vec3f(0, 0, 1);
            double aT = (grav + a).norm();
            Vec3f xB, yB, zB;
            Vec3f xC(cos(0), sin(0), 0);
            zB = (grav + a).normalized();
            yB = ((zB).cross(xC)).normalized();
            xB = yB.cross(zB);
            Mat3f R;
            R << xB, yB, zB;
            Quatf q(R);
            Vec3f eular_wo_drag = geometry_utils::quaternion_to_ypr(q);
            Vec3f eular = geometry_utils::quaternion_to_ypr(Quaterniond(quat[0], quat[1], quat[2], quat[3]));

            traj_profile << t << " " << p.transpose() << " " << v.transpose() << " " << a.transpose() << " "
                         << j.transpose() << " " << s.transpose() << " " << omg.transpose() << " "
                         << eular.transpose() << " " << eular_wo_drag.transpose() << " "
                         << thr / cfg.mass << endl;
        }
        traj_profile.close();
    }


    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    mkrarr_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1);

    TrajOptConfig cfg(nh, "exp_traj");
    optimizer = make_shared<ExpTrajOpt>(nh, cfg);
    TrajOptConfig bcfg(nh, "backup_traj");
    backup_optimizer = make_shared<BackupTrajOpt>(nh, bcfg);

    TrajOptConfig ycfg(nh, "yaw_traj");
    yaw_optimizer = make_shared<YawTrajOpt>(10.0);

    cout << GREEN << " -- [Traj-Test] Begin." << RESET << endl;
    ros::Duration(1).sleep();
    VisualUtils::DeleteMkrArr(mkrarr_pub);
    LoadOfflinePlanningProblemTraj(cfg, false);
    ros::spin();
    return 0;
}