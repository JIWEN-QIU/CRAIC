//
// Created by yunfan on 2022/2/17.
//
#pragma once

#include <super_core/config.h>
#include <super_core/visualizer.h>
#include <geometry_utils/polytope.h>
#include <geometry_utils/trajectory.h>
#include <type_utils/common_type_name.h>
#include <super_core/control_cmd_gene.h>
#include <super_core/ciri.h>
#include <rog_map/rog_map.h>
#include "super_core/gen_sfc_baselines.h"

namespace super_planner {
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
    using namespace type_utils;
    using namespace geometry_utils;
    using namespace rog_map;


    class CorridorGenerator {
    private:
        double bound_dis_;
        double seed_line_max_length_;
        double min_overlap_threshold_;
        double robot_r_;
        int box_search_skip_num_;
        int iris_iter_num_;
        double virtual_groud_height_ = 0.0;
        double virtual_ceil_height_ = 0.0;
        ROGMap::Ptr map_ptr_;
        vec_E<Vec3i> line_seed_neighbor_list;
        CIRI::Ptr ciri_;
        std::ofstream failed_traj_log;

        double ciri_t{0};
        int ciri_cnt{0};
    public:
        CorridorGenerator(const ros::NodeHandle &nh,
                          const ROGMap::Ptr map_ptr,
                          const double bound_dis,
                          const double seed_line_max_dis,
                          const double min_overlap_threshold,
                          const double virtual_groud_height,
                          const double virtual_ceil_height,
                          const double robot_r,
                          const int box_search_skip_num,
                          const int iris_iter_num);

        ~CorridorGenerator() = default;

        void SetLineNeighborList(const vec_E<Vec3i> &line_seed_neighbor_list);

        typedef shared_ptr<CorridorGenerator> Ptr;

        bool SearchPolytopeOnPath(const vec_Vec3f &path, PolytopeVec &sfcs,
                                  bool cut_first_poly = false);

        void getSeedBBox(const Vec3f &p1, const Vec3f &p2,
                         Vec3f &box_min, Vec3f &box_max);

        bool GeneratePolytopeFromPoint(const Vec3f &pt, Polytope &polytope);

        bool GeneratePolytopeFromLine(Line &line, Polytope &polytope);

        double getCiriComputationTime() {
            if (ciri_cnt == 0) {
                return -1;
            }
            double aver_T = ciri_t / ciri_cnt;
            ciri_t = 0;
            ciri_cnt = 0;
            return aver_T;
        }

        bool GeneratePolyWithFiri(Line &line, Polytope &polytope) {
            Eigen::Vector3d box_max, box_min;
            vec_E<Vec3f> pc, pts{line.first, line.second};
            getSeedBBox(line.first, line.second, box_min, box_max);
            map_ptr_->boundBoxByLocalMap(box_min, box_max);
            map_ptr_->boxSearch(box_min, box_max, OCCUPIED, pc);
            box_min.z()+=robot_r_;
            box_max.z()-=robot_r_;
            MatD4f planes;
            Eigen::Vector3d a = line.first, b = line.second;
            Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
            bd(0, 0) = 1.0;
            bd(1, 0) = -1.0;
            bd(2, 1) = 1.0;
            bd(3, 1) = -1.0;
            bd(4, 2) = 1.0;
            bd(5, 2) = -1.0;
            bd(0, 3) = -box_max.x();
            bd(1, 3) = box_min.x();
            bd(2, 3) = -box_max.y();
            bd(3, 3) = box_min.y();
            bd(4, 3) = -box_max.z();
            bd(5, 3) = box_min.z();
            // 将vector放到Eigen里，准备开始分解
            if (pc.empty()) {
                // 障碍物点云为空，直接返回一个方块
                // Ax + By + Cz + D = 0g
                planes.resize(6, 4);
                planes.row(0) << 1, 0, 0, -box_max.x();
                planes.row(1) << 0, 1, 0, -box_max.y();
                planes.row(2) << 0, 0, 1, -box_max.z();
                planes.row(3) << -1, 0, 0, box_min.x();
                planes.row(4) << 0, -1, 0, box_min.y();
                planes.row(5) << 0, 0, -1, box_min.z();
                polytope.SetPlanes(planes);
                polytope.SetSeedLine(line);
                return true;
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(pc[0].data(), 3, pc.size());
            TimeConsuming tc("ciri", false);

            bool success = firi::convexDecomposition(bd, pp, a, b, polytope, robot_r_, false, iris_iter_num_);
            double dt = tc.stop();
            if (success) {
                ciri_cnt++;
                ciri_t += dt;
                polytope.SetSeedLine(line);
                return true;
            } else {
                polytope.Reset();
                cout << YELLOW << "\t box_min = " << box_min.transpose() << RESET << endl;
                cout << YELLOW << "\t box_max =" << box_max.transpose() << RESET << endl;
                cout << YELLOW << "\t seed line =" << line.first.transpose() << " --> " << line.second.transpose()
                     << RESET << endl;

                failed_traj_log << 889900 << endl;
                failed_traj_log << bd << endl;
                failed_traj_log << 0 << endl;
                failed_traj_log << pp << endl;
                failed_traj_log << 0 << endl;
                failed_traj_log << a.transpose() << endl;
                failed_traj_log << 0 << endl;
                failed_traj_log << b.transpose() << endl;
                failed_traj_log << 0 << endl;
                failed_traj_log << robot_r_ << endl;
                failed_traj_log << 0 << endl;
                failed_traj_log << iris_iter_num_ << endl;
                return false;
            }
        }

        void setIterNum(int iter){
            iris_iter_num_ = iter;
        }

        bool
        SearchWithFiri(const vec_Vec3f &path, PolytopeVec &sfcs) {
            // https://whimsical.com/flow-3TASJFwe1dASYYY2xHEmze
            // password: wtr
            //	TimeConsuming t___("SearchPolytopeOnPath");
            sfcs.clear();
            if (path.empty()) {
                return false;
            }

            vector<Line> seed_lines;
            int first_id, second_id;
            Polytope overlap;
            Vec3f interior_pt;
            double interior_depth;
            Polytope temp_poly, temp_poly_fix_p;
            int max_loop = 1000;
            int cnt_loop = 0;
            first_id = 0;

            while (cnt_loop++ < max_loop) {
                second_id = first_id;
                for (int j = first_id + 1; j < path.size(); j++) {
                    bool reach_segment = false;
                    if (!map_ptr_->isLineFree(path[first_id], path[j], seed_line_max_length_,
                                              line_seed_neighbor_list)) {
                        reach_segment = true;
                    }
                    if (reach_segment) {
                        second_id = j - 1;
                        if (second_id - 1 > first_id) {
                            second_id -= 1;
                        }
                        break;
                    }
                    second_id = j;
                }

                if (second_id == first_id && second_id + 1 < path.size()) {
                    second_id += 1;
                }

                seed_lines.emplace_back(path[first_id], path[second_id]);
                if ((path[first_id] - path[second_id]).norm() > seed_line_max_length_ * 1.5) {
                    cout << RED << " -- [SVC] seed line too long, force return." << RESET << endl;
                    cout << RED << " -- [SVC] seed line too long, force return." << RESET << endl;
                    cout << RED << " -- [SVC] seed line too long, force return." << RESET << endl;
                    cout << RED << " -- [SVC] seed line too long, force return." << RESET << endl;
                    // exit(0);
                }
                if (!GeneratePolyWithFiri(seed_lines.back(), temp_poly)) {
                    cout << YELLOW << " -- [SVC] GeneratePolytopeFromLine failed." << RESET << endl;
                    return false;
                }

                if (!sfcs.empty()) {
                    overlap = sfcs.back().CrossWith(temp_poly);
                    interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
                    temp_poly.overlap_depth_with_last_one = interior_depth;
                    temp_poly.interior_pt_with_last_one = interior_pt;
//                    if (interior_depth < min_overlap_threshold_) {
//                        if (!GeneratePolyWithFiri(Line{path[first_id],path[first_id]}, temp_poly_fix_p)) {
//                            cout << YELLOW << " -- [SVC] GeneratePolytopeFromPoint failed." << RESET << endl;
//                            return false;
//                        }
//                        overlap = sfcs.back().CrossWith(temp_poly_fix_p);
//                        interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
//                        if (interior_depth <= 0.01) {
//                            cout << YELLOW <<
//                                 " -- [SVC] Cannot find continous corridor on path, force return." << RESET << endl;
//                            return false;
//                        }
//                        temp_poly_fix_p.overlap_depth_with_last_one = interior_depth;
//                        temp_poly_fix_p.interior_pt_with_last_one = interior_pt;
//                        sfcs.push_back(temp_poly_fix_p);
//                        overlap = sfcs.back().CrossWith(temp_poly);
//                        interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
//                        if (interior_depth <= 0.01) {
//                            cout << YELLOW <<
//                                 " -- [SVC] Cannot find continous corridor on path, force return." << RESET << endl;
//                            return false;
//                        }
//                    } else
                    {
                        int temp_id = sfcs.size() - 2;
                        if (temp_id > 0) {
                            overlap = sfcs[temp_id].CrossWith(temp_poly);
                            interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
                            if (interior_depth > sfcs[temp_id + 1].overlap_depth_with_last_one * 0.25) {
                                temp_poly.overlap_depth_with_last_one = interior_depth;
                                temp_poly.interior_pt_with_last_one = interior_pt;
                                sfcs.pop_back();
                            }
                        }
                    }
                }

                sfcs.push_back(temp_poly);
                if (second_id == path.size() - 1) {
                    break;
                }
                first_id = second_id;
            }
            // Delete last polytope if the second last one contains the last point
            if (sfcs.size() >= 2 && sfcs[sfcs.size() - 2].PointIsInside(path.back())) {
                sfcs.pop_back();
            }

            if (sfcs.size() >= 2 && sfcs[1].PointIsInside(path.front())) {
                sfcs.erase(sfcs.begin());
            }

            if (cnt_loop >= max_loop) {
                cout << YELLOW << " -- [SVC] Reach max iteration, failed." << RESET << endl;
                return false;
            }

            if (sfcs.empty()) {
                return false;
            }

            return true;
        }

    };
}
