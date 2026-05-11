/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#pragma once

#include <math_utils/lbfgs.h>
#include <math_utils/sdlp.h>
#include <geometry_utils/geometry_utils.h>
#include <geometry_utils/polytope.h>
#include <geometry_utils/ellipsoid.h>
#include <benchmark_utils/scope_timer.h>
#include <optimization_utils/optimization_utils.h>
#include <optimization_utils/mvie.h>
#include <memory>
#include <Eigen/SVD>
#include <Eigen/Dense>

namespace super_planner {
    using namespace optimization_utils;
    using namespace geometry_utils;
    using namespace type_utils;
    using namespace std;


    class firi {
    private:
/**
 * @brief findEllipsoid: find maximum ellipsoid with RILS
 * @param pc the obstacle points
 * @param a the start point of the line segment seed
 * @param b the end point of the line segment seed
 * @param out_ell the output ellipsoid
 * @param r_robot the robot_size, decide if the polytope need to be shrink
 * @param _fix_p decide if the ellipsoid center need to be optimized
 * @param iterations number of the alternating optimization
 */

        static void findEllipsoid(
                const Eigen::Matrix3Xd &pc,
                const Eigen::Vector3d &a,
                const Eigen::Vector3d &b,
                Ellipsoid &out_ell,
                const double &robot_r = 0.0) {

            double f = (a - b).norm() / 2;
            Mat3f C = f * Mat3f::Identity();
            Vec3f r = Vec3f::Constant(f);
            Vec3f center = (a + b) / 2;
            C(0, 0) += robot_r;
            r(0) += robot_r;
            if (r(0) > 0) {
                double ratio = r(1) / r(0);
                r *= ratio;
                C *= ratio;
            }

            Mat3f Ri = Eigen::Quaterniond::FromTwoVectors(Vec3f::UnitX(), (b - a)).toRotationMatrix();
            Ellipsoid E(Ri, r, center);
            Mat3f Rf = Ri;
            Mat3Df obs;
            int min_dis_id;
            Vec3f pw;
            if (E.pointsInside(pc, obs, min_dis_id)) {
                pw = obs.col(min_dis_id);
            } else {
                out_ell = E;
                return;
            }
            Mat3Df obs_inside = obs;
            int max_iter = 100;
            while (max_iter--) {
                Vec3f p_e = Ri.transpose() * (pw - E.d());
                const double roll = atan2(p_e(2), p_e(1));
                Rf = Ri * Eigen::Quaterniond(cos(roll / 2), sin(roll / 2), 0, 0);
                p_e = Rf.transpose() * (pw - E.d());
                if (p_e(0) < r(0)) {
                    r(1) = std::abs(p_e(1)) / std::sqrt(1 - std::pow(p_e(0) / r(0), 2));
                }
                E = Ellipsoid(Rf, r, center);
                if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
                    pw = obs_inside.col(min_dis_id);
                } else {
                    break;
                }
            }
            if (max_iter == 0) {
                cout << YELLOW" -- [CIRI] Find Ellipsoid reach max iteration, may cause error." << endl;
            }
            max_iter = 100;


            if (E.pointsInside(obs, obs_inside, min_dis_id)) {
                pw = obs_inside.col(min_dis_id);
            } else {
                out_ell = E;
                return;
            }

            while (max_iter--) {
                Vec3f p = Rf.transpose() * (pw - E.d());
                double dd = 1 - std::pow(p(0) / r(0), 2) -
                            std::pow(p(1) / r(1), 2);
                if (dd > epsilon_) {
                    r(2) = std::abs(p(2)) / std::sqrt(dd);
                }
                E = Ellipsoid(Rf, r, center);
                if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
                    pw = obs_inside.col(min_dis_id);
                } else {
                    out_ell = E;
                    break;
                }
            }

            if (max_iter == 0) {
                cout << YELLOW " -- [CIRI] Find Ellipsoid reach max iteration, may cause error." << endl;
            }
            E = Ellipsoid(Rf, r, center);
            out_ell = E;
        }

        static void findTangentPlaneOfSphere(const Eigen::Vector3d &center, const double &r,
                                             const Eigen::Vector3d &pass_point,
                                             const Eigen::Vector3d &seed_p,
                                             Eigen::Vector4d &outter_plane) {
            Eigen::Vector3d P = pass_point - center;
            Eigen::Vector3d norm_ = (pass_point - center).cross(seed_p - center).normalized();
            Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(norm_, Vec3f(0, 0, 1)).matrix();
            P = R * P;
            Eigen::Vector3d C = R * (seed_p - center);
            Eigen::Vector3d Q;
            double r2 = r * r;
            double p1p2n = P.head(2).squaredNorm();
            double d = sqrt(p1p2n - r2);
            double rp1p2n = r / p1p2n;
            double q11 = rp1p2n * (P(0) * r - P(1) * d);
            double q21 = rp1p2n * (P(1) * r + P(0) * d);

            double q12 = rp1p2n * (P(0) * r + P(1) * d);
            double q22 = rp1p2n * (P(1) * r - P(0) * d);
            if (q11 * C(0) + q21 * C(1) > 0) {
                Q(0) = q12;
                Q(1) = q22;
            } else {
                Q(0) = q11;
                Q(1) = q21;
            }
            Q(2) = 0;
            // point(Q) + normal (AQ)
            outter_plane.head(3) = R.transpose() * Q;
            Q = outter_plane.head(3) + center;
            outter_plane(3) = -Q.dot(outter_plane.head(3));
            if (outter_plane.head(3).dot(center) + outter_plane(3) > epsilon_) {
                outter_plane = -outter_plane;
            }
        }

    public:

        firi() = default;

        ~firi() = default;

/**
 * @brief embed maximum volume polytope
 * @param bd bounding box with 6 faces
 * @param pc the obstacle points
 * @param a the start point of the line segment seed
 * @param b the end point of the line segment seed
 * @param hPoly the output polytope
 * @param r_robot the robot_size, decide if the polytope need to be shrink
 * @param _fix_p decide if the ellipsoid center need to be optimized
 * @param iterations number of the alternating optimization
 */
        static bool convexDecomposition(const Eigen::MatrixX4d &bd,
                                        const Eigen::Matrix3Xd &pc,
                                        const Eigen::Vector3d &a,
                                        const Eigen::Vector3d &b,
                                        Polytope &out_poly,
                                        const double r_robot = 0.0,
                                        const bool _fix_p = false,
                                        const int iterations = 3) {
            const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
            const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

            /// force return if the seed is not inside the boundary
            if ((bd * ah).maxCoeff() > 0.0 ||
                (bd * bh).maxCoeff() > 0.0) {
                cout << YELLOW << " -- [WARN] ah, bh not in BD, forced return." << endl;
                cout << YELLOW << a.transpose() << endl;
                cout << YELLOW << b.transpose() << endl;
                return false;
            }

            /// Maximum M boundary constraints and N point constraints
            const int M = bd.rows();
            const int N = pc.cols();

            Ellipsoid E(Mat3f::Identity(), (a + b) / 2);
            if ((a - b).norm() > 0.1) {
                /// use line seed
                findEllipsoid(pc, a, b, E, r_robot);
            }

            vector<Eigen::Vector4d> planes;
            MatD4f hPoly;
            for (int loop = 0; loop < iterations; ++loop) {
                // Initialize the boundary in ellipsoid frame
                const Eigen::MatrixX4d bd_e = E.toEllipsoidFrame(bd);
                // Initialize the seed points
                const Eigen::Vector3d fwd_a = E.toEllipsoidFrame(a);
                const Eigen::Vector3d fwd_b = E.toEllipsoidFrame(b);
                const Eigen::VectorXd distDs = bd_e.rightCols<1>().cwiseAbs().cwiseQuotient(
                        bd_e.leftCols<3>().rowwise().norm());
                const Eigen::Matrix3Xd pc_e = E.toEllipsoidFrame(pc);
                Eigen::VectorXd distRs = pc_e.colwise().norm();

                Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
                Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

                bool completed = false;
                int bdMinId, pcMinId;
                double minSqrD = distDs.minCoeff(&bdMinId);
                double minSqrR = distRs.minCoeff(&pcMinId);

                Eigen::Vector3d pt_w, pt_e;
                Eigen::Vector4d temp_tangent, temp_tange_W;

                planes.clear();
                planes.reserve(30);
                for (int i = 0; !completed && i < (M + N); ++i) {
                    // Get the min dis point of this round.
                    pt_w = pc.col(pcMinId);
                    pt_e = pc_e.col(pcMinId);
                    if (minSqrD < minSqrR) {
                        // enable the boundary constrain.
                        temp_tangent = bd_e.row(bdMinId);
                        bdFlags(bdMinId) = 0;
                    } else {
                        // enable the obstacle point constarin.
                        // First generate a plane in E frame
                        temp_tangent(3) = -distRs(pcMinId);
                        temp_tangent.head(3) = pt_e.transpose() / distRs(pcMinId);
                        if (r_robot < epsilon_) {
                            if (temp_tangent.head(3).dot(fwd_a) + temp_tangent(3) > epsilon_) {
                                const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_a;
                                temp_tangent.head(3) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta;
                                distRs(pcMinId) = temp_tangent.head(3).norm();
                                temp_tangent(3) = -distRs(pcMinId);
                                temp_tangent.head(3) /= distRs(pcMinId);
                            }
                            if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > epsilon_) {
                                const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_b;
                                temp_tangent.head(3) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                                distRs(pcMinId) = temp_tangent.head(3).norm();
                                temp_tangent(3) = -distRs(pcMinId);
                                temp_tangent.head(3) /= distRs(pcMinId);
                            }
                            if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > epsilon_) {
                                const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_b;
                                temp_tangent.head(3) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                                distRs(pcMinId) = temp_tangent.head(3).norm();
                                temp_tangent(3) = -distRs(pcMinId);
                                temp_tangent.head(3) /= distRs(pcMinId);
                            }
                        } else {
                            // Then convert the plane to world frame
                            temp_tange_W = E.toWorldFrame(temp_tangent);
                            // the check in the w frame
                            if (temp_tange_W.head(3).dot(a) + temp_tange_W(3) + r_robot > epsilon_) {
                                findTangentPlaneOfSphere(a, r_robot, pt_w, E.d(), temp_tange_W);
                            }
                            if (temp_tange_W.head(3).dot(b) + temp_tange_W(3) + r_robot > epsilon_) {
                                findTangentPlaneOfSphere(b, r_robot, pt_w, E.d(), temp_tange_W);
                            }
                            temp_tangent = E.toEllipsoidFrame(temp_tange_W);
                        }
//		distRs(pcMinId) = -temp_tangent(3);
                        pcFlags(pcMinId) = 0;
                    }
                    // update pcMinId and bdMinId
                    completed = true;
                    minSqrD = INFINITY;
                    for (int j = 0; j < M; ++j) {
                        if (bdFlags(j)) {
                            completed = false;
                            if (minSqrD > distDs(j)) {
                                bdMinId = j;
                                minSqrD = distDs(j);
                            }
                        }
                    }
                    minSqrR = INFINITY;

                    for (int j = 0; j < N; ++j) {
                        if (pcFlags(j)) {
                            if (temp_tangent.head(3).dot(pc_e.col(j)) + temp_tangent(3) > -epsilon_) {
                                pcFlags(j) = 0;
                            } else {
                                completed = false;
                                if (minSqrR > distRs(j)) {
                                    pcMinId = j;
                                    minSqrR = distRs(j);
                                }
                            }
                        }
                    }


                    planes.push_back(temp_tangent);
                }
                hPoly.resize(planes.size(), 4);
                for (int i = 0; i < planes.size(); ++i) {
                    hPoly.row(i) = E.toWorldFrame(planes[i]);
                }
                if (loop == iterations - 1) {
                    break;
                }

                MVIE::maxVolInsEllipsoid(hPoly, E);
            }

            /// shrink the polytope with robot_r
            for (int i = 0; i < hPoly.rows(); i++) {
                // A B C D / n + r )* n
                double n = hPoly.row(i).head(3).norm();
                hPoly(i, 3) += r_robot * n;
            }
            if (isnan(hPoly.sum())) {
                cout << RED << " -- [CIRI] ERROR! There is nan in generated planes." << RESET << endl;
                cout << a.transpose() << endl;
                cout << b.transpose() << endl;
                return false;
            }
            Vec3f inner;
            if (!geometry_utils::findInterior(hPoly, inner)) {
                ROS_WARN(" -- [CIRI] ERROR! hPoly is empty");
                return false;
            }
            out_poly.SetPlanes(hPoly);
            out_poly.SetSeedLine(std::make_pair(a, b));
            out_poly.SetEllipsoid(E);
            return true;
        }

    };

}
