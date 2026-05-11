#include <traj_opt/exp_traj_optimizer_s4.h>

using namespace traj_opt;

void ExpTrajOpt::constraintsFunctional(const Eigen::VectorXd &T,
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
                                       VectorXd &pena_log) {
//    opt_vars.magnitudeBounds
//            << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk, cfg_.max_omg, cfg_.min_acc_thr, cfg_.max_acc_thr;
//    opt_vars.penaltyWeights << cfg_.penna_pos, cfg_.penna_vel,
//            cfg_.penna_acc, cfg_.penna_jerk,
//            cfg_.penna_attract, cfg_.penna_omg,
//            cfg_.penna_thr;
    const double vmax = magnitudeBounds[0];
    const double amax = magnitudeBounds[1];
    const double jmax = magnitudeBounds[2];
    const double omgmax = magnitudeBounds[3];
    const double accthrmin = magnitudeBounds[4];
    const double accthrmax = magnitudeBounds[5];

    const double vmaxSqr = vmax * vmax;
    const double amaxSqr = amax * amax;
    const double jmaxSqr = jmax * jmax;
    const double omgmaxSqr = omgmax * omgmax;

    const double thrustMean = 0.5 * (accthrmax + accthrmin);
    const double thrustRadi = 0.5 * std::abs(accthrmax - accthrmin);
    const double thrustSqrRadi = thrustRadi * thrustRadi;

    const double weightPos = penaltyWeights[0];
    const double weightVel = penaltyWeights[1];
    const double weightAcc = penaltyWeights[2];
    const double weightJer = penaltyWeights[3];
    const double weightAtt = penaltyWeights[4];
    const double weightOmg = penaltyWeights[5];
    const double weightAccThr = penaltyWeights[6];


    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
    double totalGradPsi, totalGradPsiD;
    double thr, cos_theta;
    Eigen::Vector4d quat;
    Eigen::Vector3d omg;
    double gradThr;
    Eigen::Vector4d gradQuat;
    Eigen::Vector3d gradPos, gradVel, gradAcc, gradJer, gradOmg;

    double step, alpha;
    double s1, s2, s3, s4, s5, s6, s7;
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Vector3d outerNormal;
    int K, L;

    double violaPos, violaVel, violaAcc, violaJer, violaOmg, violaTheta, violaThrust;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaJerPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
    double violaPosPena, violaVelPena, violaAccPena, violaJerPena, violaOmgPena, violaThetaPena, violaThrustPena;
    double violaAtt, violaAttPena, violaAttPenaD;
    double node, pena;
    const int pieceNum = T.size();
    const double integralFrac = 1.0 / integralResolution;
    pena = 0.0;
    double pos_penna_log = 0.0;
    double max_pos_viola_log = 0.0;
    double vel_penna_log = 0.0;
    double max_vel_viola_log = 0.0;
    double acc_penna_log = 0.0;
    double max_acc_viola_log = 0.0;
    double jer_penna_log = 0.0;
    double max_jer_viola_log = 0.0;
    double att_penna_log = 0.0;
    double max_att_viola_log = 0.0;
    double omg_penna_log = 0.0;
    double max_omg_viola_log = 0.0;
    double thr_penna_log = 0.0;
    double max_thr_viola_log = 0.0;

    for (int i = 0; i < pieceNum; i++) {
        const Eigen::Matrix<double, 8, 3> &c = coeffs.block<8, 3>(i * 8, 0);

        step = T(i) * integralFrac;
        for (int j = 0; j <= integralResolution; j++) {
            s1 = j * step;
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            s6 = s4 * s2;
            s7 = s4 * s3;
            beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
            beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
            beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
            beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
//            beta5 << 0.0, 0.0, 0.0, 0., 0.0, 120.0, 720.0 * s1, 2520.0 * s2;

            pos = c.transpose() * beta0;
            vel = c.transpose() * beta1;
            acc = c.transpose() * beta2;
            jer = c.transpose() * beta3;
            sna = c.transpose() * beta4;
            alpha = j * integralFrac;

            L = hIdx(i);
            K = hPolys[L].rows();
            node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;


            violaVel = vel.squaredNorm() - vmaxSqr;
            violaAcc = acc.squaredNorm() - amaxSqr;
            violaJer = jer.squaredNorm() - jmaxSqr;
            violaThrust = 0;
            violaOmg = 0;
            gradThr = 0.0;
//            gradQuat.setZero();
            gradPos << 0, 0, 0;
            gradVel << 0, 0, 0;
            gradAcc << 0, 0, 0;
            gradJer << 0, 0, 0;
            gradOmg << 0, 0, 0;

            pena = 0.0;

            L = hIdx(i);
            K = hPolys[L].rows();

            if (weightPos > 0) {
                for (int k = 0; k < K; k++) {
                    outerNormal = hPolys[L].block<1, 3>(k, 0);
                    violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                    if (violaPos > max_pos_viola_log) max_pos_viola_log = violaPos;
                    if (gcopter::smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
                        gradPos += weightPos * violaPosPenaD * outerNormal;
                        pena += weightPos * violaPosPena;
                        pos_penna_log += weightPos * violaPosPena;
                    }
                }
            }

            if (weightAtt > 0.0) {
                if (j == 0 && i != 0) {
                    // 为waypoint施加吸引力。从左边pena，第1段轨迹对应第0个点
                    Vec3f dir = pos - waypoint_attractor.col(i - 1);
                    violaAtt = dir.norm() - waypoint_attractor_dead_d(i - 1);
                    dir = dir.normalized();
                    if (gcopter::smoothedL1(violaAtt, smoothFactor, violaAttPena, violaAttPenaD)) {
                        gradPos += weightAtt * violaAttPenaD * dir;
                        pena += weightAtt * violaAttPena;
//                        gradC.block<8, 3>(i * 8, 0) +=
//                                0.5 * step * weightAtt * violaAttPenaD * beta0 * dir.transpose();
//                        gradT(i) += 0.5 * (weightAtt * violaAttPenaD * alpha * dir.dot(vel) * step +
//                                           weightAtt * violaAttPena * integralFrac);
//                        double local_penna = 0.5 * step * weightAtt * violaAttPena;
//                        pena += local_penna;
                    }
                }
                if (j == integralResolution && i != pieceNum - 1) {
                    Vec3f dir = pos - waypoint_attractor.col(i);
                    violaAtt = dir.norm() - waypoint_attractor_dead_d(i);
                    dir = dir.normalized();
                    if (gcopter::smoothedL1(violaAtt, smoothFactor, violaAttPena, violaAttPenaD)) {
                        gradPos += weightAtt * violaAttPenaD * dir;
                        pena += weightAtt * violaAttPena;
//                        gradC.block<8, 3>(i * 8, 0) +=
//                                0.5 * step * weightAtt * violaAttPenaD * beta0 * dir.transpose();
//                        gradT(i) += 0.5 * (weightAtt * violaAttPenaD * alpha * dir.dot(vel) * step +
//                                           weightAtt * violaAttPena * integralFrac);
//                        double local_penna = 0.5 * step * weightAtt * violaAttPena;
//                        pena += local_penna;
                    }
                }
            }

            if (weightVel > 0 && gcopter::smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
                gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                pena += weightVel * violaVelPena;
                vel_penna_log += weightVel * violaVelPena;
            }

            if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {
                gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
                pena += weightAcc * violaAccPena;
                acc_penna_log += weightAcc * violaAccPena;
            }

            if (weightJer > 0 && gcopter::smoothedL1(violaJer, smoothFactor, violaJerPena, violaJerPenaD)) {
                gradJer += weightJer * violaJerPenaD * 2.0 * jer;
                pena += weightJer * violaJerPena;
                jer_penna_log += weightJer * violaJerPena;
            }


            if (weightOmg > 0 && weightAccThr > 0) {
                flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);
                violaOmg = omg.squaredNorm() - omgmaxSqr;
                violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                if (weightOmg > 0 && gcopter::smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD)) {
                    gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                    pena += weightOmg * violaOmgPena;
                    omg_penna_log += weightOmg * violaOmgPena;
                }

//            if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
//            {
//                gradQuat += weightTheta * violaThetaPenaD /
//                            sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
//                            Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
//                pena += weightTheta * violaThetaPena;
//            }

                if (weightAccThr > 0 &&
                    gcopter::smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD)) {
                    gradThr += weightAccThr * violaThrustPenaD * 2.0 * (thr - thrustMean);
                    pena += weightAccThr * violaThrustPena;
                    thr_penna_log += weightAccThr * violaThrustPena;
                }

                flatMap.backward(gradPos, gradVel, gradAcc, gradJer, gradThr, Vec4f(0, 0, 0, 0), gradOmg,
                                 totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                 totalGradPsi, totalGradPsiD);
            } else {
                totalGradPos = gradPos;
                totalGradVel = gradVel;
                totalGradAcc = gradAcc;
                totalGradJer = gradJer;
            }

            {
                // log the max violation
                if (violaVel > max_vel_viola_log) max_vel_viola_log = violaVel;
                if (violaAcc > max_acc_viola_log) max_acc_viola_log = violaAcc;
                if (violaJer > max_jer_viola_log) max_jer_viola_log = violaJer;
                if (violaOmg > max_omg_viola_log) max_omg_viola_log = violaOmg;
                if (violaThrust > max_thr_viola_log) max_thr_viola_log = violaThrust;
            }


            node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
            alpha = j * integralFrac;
            gradC.block<8, 3>(i * 8, 0) += (beta0 * totalGradPos.transpose() +
                                            beta1 * totalGradVel.transpose() +
                                            beta2 * totalGradAcc.transpose() +
                                            beta3 * totalGradJer.transpose()) *
                                           node * step;
            gradT(i) += (totalGradPos.dot(vel) +
                         totalGradVel.dot(acc) +
                         totalGradAcc.dot(jer) +
                         totalGradJer.dot(sna)) *
                        alpha * node * step +
                        node * integralFrac * pena;
            cost += node * step * pena;
        }
    }

//    pena_log(1) = pos_penna_log;
//    pena_log(2) = vel_penna_log;
//    pena_log(3) = acc_penna_log;
//    pena_log(4) = jer_penna_log;
//    pena_log(5) = att_penna_log;
//    pena_log(6) = omg_penna_log;
//    pena_log(7) = thr_penna_log;
    pena_log(1) = max_pos_viola_log;
    pena_log(2) = max_vel_viola_log;
    pena_log(3) = max_acc_viola_log;
    pena_log(4) = max_jer_viola_log;
    pena_log(5) = max_att_viola_log;
    pena_log(6) = max_omg_viola_log;
    pena_log(7) = max_thr_viola_log;
}

double ExpTrajOpt::costFunctional(void *ptr,
                                  const Eigen::VectorXd &x,
                                  Eigen::VectorXd &g) {
    OptimizationVariables &obj = *(OptimizationVariables *) ptr;
    const int dimTau = obj.temporalDim;
    const int dimXi = obj.spatialDim;
    const double weightT = obj.rho;
    Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

    obj.iter_num++;
    gcopter::forwardMapTauToT(tau, obj.times);
    switch (obj.pos_constraint_type) {
        case 1: {
            VecDf xi_e = xi;
            obj.points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
            break;
        }
        default: {
            gcopter::forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);
            break;
        }
    }
    double cost{0};
    obj.minco.setParameters(obj.points, obj.times);
    obj.partialGradByCoeffs.setZero();
    obj.partialGradByTimes.setZero();
    if (!obj.block_energy_cost) {
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
    }
    obj.penalty_log(0) = cost;
    constraintsFunctional(obj.times, obj.minco.getCoeffs(),
                          obj.hPolyIdx, obj.hPolytopes,
                          obj.waypoint_attractor, obj.waypoint_attractor_dead_d,
                          obj.smooth_eps, obj.integral_res,
                          obj.magnitudeBounds, obj.penaltyWeights,
                          obj.quadrotor_flatness,
                          cost, obj.partialGradByTimes, obj.partialGradByCoeffs, obj.penalty_log);

    obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                            obj.gradByPoints, obj.gradByTimes);
    cost += weightT * obj.times.sum();
    obj.gradByTimes.array() += weightT;

    gcopter::propagateGradientTToTau(tau, obj.gradByTimes, gradTau);
    switch (obj.pos_constraint_type) {
        case 1: {
            MatDf gp = obj.gradByPoints;
            gradXi = Eigen::Map<Eigen::VectorXd>(gp.data(), gp.size());
            break;
        }
        default: {
            gcopter::backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
            gcopter::normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);
            break;
        }
    }
    return cost;
}

bool ExpTrajOpt::processCorridor() {
    const long sizeCorridor = opt_vars.hPolytopes.size() - 1;

    opt_vars.vPolytopes.clear();
    opt_vars.vPolytopes.reserve(2 * sizeCorridor + 1);

    int nv;
    PolyhedronH curIH;
    PolyhedronV curIV, curIOB; // 走廊的顶点
    opt_vars.waypoint_attractor.resize(3, sizeCorridor);
    opt_vars.waypoint_attractor_dead_d.resize(sizeCorridor);
    opt_vars.hOverlapPolytopes.resize(sizeCorridor);
    for (long i = 0; i < sizeCorridor; i++) {
        if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
            ROS_WARN(" -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs.");
            return false;
        }
        // 走廊顶点的个数
        nv = curIV.cols();
        curIOB.resize(3, nv);
        // 第一个点存储第一个顶点
        curIOB.col(0) = curIV.col(0);
        // 后面的点都归一化到第一个点坐标系下
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
        // 当前走廊和下一个走廊抽出来
        curIH.resize(opt_vars.hPolytopes[i].rows() + opt_vars.hPolytopes[i + 1].rows(), 4);
        curIH.topRows(opt_vars.hPolytopes[i].rows()) = opt_vars.hPolytopes[i];
        curIH.bottomRows(opt_vars.hPolytopes[i + 1].rows()) = opt_vars.hPolytopes[i + 1];
        opt_vars.hOverlapPolytopes[i] = curIH;
        // 生成相交部分的走廊
        Vec3f interior;
        double dis = geometry_utils::findInteriorDist(curIH, interior) / 2;
//        if (dis < 0.0 || std::isinf(dis)) {
//            ROS_WARN(" -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate overlap corridor Vs at %ld.",
//                     i);
//            return false;
//        }
//        waypoint_attractor_dead_d(i) = dis;
//        waypoint_attractor.col(i) = interior;
        geometry_utils::enumerateVs(curIH, interior, curIV);
//        waypoint_attractor.col(i).z() = (curIV.row(2).maxCoeff() + curIV.row(2).minCoeff()) * 0.5;
//
        opt_vars.waypoint_attractor.col(i) = curIV.rowwise().mean();
        opt_vars.waypoint_attractor_dead_d(i) = dis;

//	  double x = (curIV.row(0).maxCoeff() + curIV.row(0).minCoeff()) * 0.5;
//	  double y = (curIV.row(1).maxCoeff() + curIV.row(1).minCoeff()) * 0.5;

//	  waypoint_attractor.col(i) = Vec3f(x, y, z);
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
    }

    if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
        ROS_WARN(" -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs.");
        return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytopes.push_back(curIOB);
    return true;
}

bool ExpTrajOpt::processCorridorWithGuideTraj() {
    // * 1) allocate memory for vertex
    const int sizeCorridor = opt_vars.hPolytopes.size() - 1;

    opt_vars.vPolytopes.clear();
    opt_vars.vPolytopes.reserve(2 * sizeCorridor + 1);

    int nv;
    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    opt_vars.waypoint_attractor.resize(3, sizeCorridor);
    opt_vars.hOverlapPolytopes.resize(sizeCorridor);
    opt_vars.waypoint_attractor_dead_d.resize(sizeCorridor);
    // * 2) Process the corridor
    for (int i = 0; i < sizeCorridor; i++) {
        // * 2.1) Get current vertex
        if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
//            print(fg(color::gold), " -- [processCorridor] Failed to enumerateVs .\n");
            return false;
        }
        // * 2.3) Conver the vertex to the frame of the first point
        nv = curIV.cols();
        curIOB.resize(3, nv);
        // *    Save the position of the first point
        curIOB.col(0) = curIV.col(0);
        // *    Use the relative position of the rest vertex.
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        // *    save the i-th corridor's vertex
        opt_vars.vPolytopes.push_back(curIOB);

        // * 2.4) Find the overlap corridor
        curIH.resize(opt_vars.hPolytopes[i].rows() + opt_vars.hPolytopes[i + 1].rows(), 4);
        curIH.topRows(opt_vars.hPolytopes[i].rows()) = opt_vars.hPolytopes[i];
        curIH.bottomRows(opt_vars.hPolytopes[i + 1].rows()) = opt_vars.hPolytopes[i + 1];
        opt_vars.hOverlapPolytopes[i] = curIH;
        Vec3f interior;
//        // *    find the depth of the overlap coriro
        double dis = geometry_utils::findInteriorDist(curIH, interior) / 2;
        if (dis < 0.0 || std::isinf(dis)) {
//            print(fg(color::gold), " -- [processCorridor] Failed to get Overlap enumerateVs .\n");
            return false;
        }
//        waypoint_attractor_dead_d(i) = dis / 2;
//        waypoint_attractor.col(i) = interior;
        geometry_utils::enumerateVs(curIH, interior, curIV);
        double test_sum = curIV.sum();
        if (isnan(test_sum) || std::isinf(test_sum)) {
            return false;
        }
//
//
        opt_vars.waypoint_attractor.col(i) = curIV.rowwise().mean();
        opt_vars.waypoint_attractor_dead_d(i) = dis;

////	  double x = (curIV.row(0).maxCoeff() + curIV.row(0).minCoeff()) * 0.5;
////	  double y = (curIV.row(1).maxCoeff() + curIV.row(1).minCoeff()) * 0.5;
////	  waypoint_attractor.col(i) = Vec3f(x, y, z);
//        waypoint_attractor.col(i).z() = (curIV.row(2).maxCoeff() + curIV.row(2).minCoeff()) * 0.5;
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
    }

    // * 3) Time and waypoint allocation for hot initialization
    VecDf min_dis(opt_vars.waypoint_attractor.cols());
    VecDi min_id(opt_vars.waypoint_attractor.cols());
    VecDf time_stamps(opt_vars.waypoint_attractor.cols() + 2);
    time_stamps(0) = 0.0;
    time_stamps(opt_vars.waypoint_attractor.cols() + 1) = opt_vars.guide_t.back();
    min_id.setConstant(0);
    min_dis.setConstant(std::numeric_limits<double>::max());
    for (int i = 0; i < opt_vars.guide_path.size(); i++) {
        for (int j = 0; j < opt_vars.waypoint_attractor.cols(); j++) {
            double dis = (opt_vars.guide_path[i] - opt_vars.waypoint_attractor.col(j)).norm();
            if (dis < min_dis[j]) {
                min_dis[j] = dis;
                min_id[j] = i;
                opt_vars.points.col(j) = opt_vars.guide_path[i];
                time_stamps(j + 1) = opt_vars.guide_t[i];
            }
        }
    }

    for (int i = 1; i < time_stamps.size(); i++) {
        opt_vars.times(i - 1) = time_stamps(i) - time_stamps(i - 1);
        opt_vars.times(i - 1) = std::max(0.01, opt_vars.times(i - 1));
    }
//    if (times.minCoeff() <= 1e-3) {
////        print(fg(color::gold), " -- [processCorridor] Guide time becomes negative or zero.\n");
//        cout << time_stamps.transpose() << endl;
//        return false;
//    }
//            if(times.size() == 1 && times(0) == 0.0){
//                times(0) = (headPVAJ.col(0) - tailPVAJ.col(0)).norm()/magnitudeBd(0);
//            }
//            print("hot init ts ");
//            cout << times.transpose() << endl;


    if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
        return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytopes.push_back(curIOB);

    return true;
}

bool ExpTrajOpt::SimplifySFC(const type_utils::Vec3f &head_p, const type_utils::Vec3f &tail_p,
                             geometry_utils::PolytopeVec &sfcs) {
    vec_Vec3f path{head_p, tail_p};
    int start_id{-1}, end_id{-1};
    if (sfcs.size() > 2) {
        for (int i = 0; i < sfcs.size(); i++) {
            if (sfcs[i].PointIsInside(path.front())) {
                start_id = i;
            }
            if (sfcs[sfcs.size() - 1 - i].PointIsInside(path.back())) {
                end_id = sfcs.size() - 1 - i;
            }
        }
        if (start_id < 0 || end_id < 0) {
            cout << RED << " -- [EXPTrajOpt] Ill corridor! Forced return." << RESET << endl;
            return false;
        }
        if (start_id >= end_id) {
            end_id = start_id;
        }
        PolytopeVec sfcs_new(sfcs.begin() + start_id, sfcs.begin() + end_id + 1);
        PolytopeVec sfcs_final;
        if (sfcs_new.size() > 2) {
            Polytope check_cand = sfcs_new[0], last_overlapped = sfcs_new[1];
            sfcs_final.push_back(sfcs_new[0]);
            for (int i = 2; i < sfcs_new.size(); i++) {
                Polytope cross_poly = check_cand.CrossWith(sfcs_new[i]);
                Vec3f interior_pt;
                bool is_overlapped = geometry_utils::findInterior(cross_poly.GetPlanes(), interior_pt);
                if (is_overlapped) {
                    last_overlapped = sfcs_new[i];
                    if (last_overlapped.PointIsInside(path.back())) {
                        sfcs_final.push_back(last_overlapped);
                        break;
                    }
                } else {
                    sfcs_final.push_back(last_overlapped);
                    check_cand = last_overlapped;
                    i--;
                }
            }
            sfcs = sfcs_final;
        } else {
            sfcs = sfcs_new;
        }
    }
    return true;
}

void ExpTrajOpt::defaultInitialization() {
    VecDf dis = (opt_vars.init_path.leftCols(opt_vars.piece_num) -
                 opt_vars.init_path.rightCols(opt_vars.piece_num)).colwise().norm();
    double speed = cfg_.max_vel;

    opt_vars.times = dis / speed;
    opt_vars.points = opt_vars.waypoint_attractor;
}

bool ExpTrajOpt::setupProblemAndCheck() {

    // init internal variables size;
    opt_vars.piece_num = opt_vars.hPolytopes.size();
    opt_vars.times.resize(opt_vars.piece_num);
    opt_vars.points.resize(3, opt_vars.piece_num - 1);

    // Normalize the corridor
    for (size_t i = 0; i < opt_vars.hPolytopes.size(); i++) {
        const Eigen::ArrayXd norms = opt_vars.hPolytopes[i].leftCols<3>().rowwise().norm();
        opt_vars.hPolytopes[i].array().colwise() /= norms;
    }
    // Check corridor and init points
    if (opt_vars.default_init) {
        if (!processCorridor()) {
            return false;
        }
    } else {
        if (!processCorridorWithGuideTraj()) {
            return false;
        }
    }
    opt_vars.init_path.resize(3, opt_vars.piece_num + 1);
    for (long i = 0; i < opt_vars.piece_num - 1; i++) {
        opt_vars.init_path.col(i + 1) = opt_vars.waypoint_attractor.col(i);
    }
    opt_vars.init_path.col(0) = opt_vars.headPVAJ.col(0);
    opt_vars.init_path.rightCols(1) = opt_vars.tailPVAJ.col(0);
    if (opt_vars.default_init) {
        defaultInitialization();
    } else {
        opt_vars.times *= 0.8;
    }

    if (isnan(opt_vars.times.sum())) {
        cout << RED << " -- [ExpOpt] Init times and point failed." << RESET << endl;
        return false;
    }
//    cout << GREEN << " -- [ExpOpt] Init times and point success." << RESET << endl;

    const Eigen::Matrix3Xd deltas = opt_vars.init_path.rightCols(opt_vars.piece_num)
                                    - opt_vars.init_path.leftCols(opt_vars.piece_num);
    opt_vars.pieceIdx = (deltas.colwise().norm() / INFINITY).cast<int>().transpose();
    opt_vars.pieceIdx.array() += 1;


    opt_vars.temporalDim = opt_vars.piece_num;
    opt_vars.spatialDim = 0;
    opt_vars.vPolyIdx.resize(opt_vars.piece_num - 1);
    opt_vars.hPolyIdx.resize(opt_vars.piece_num);

    switch (cfg_.pos_constraint_type) {
        case 1: {
            for (long i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                k = opt_vars.pieceIdx(i);
                for (int l = 0; l < k; l++, j++) {
                    if (l < k - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i;
                    } else if (i < opt_vars.piece_num - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i + 1;
                    }
                    opt_vars.hPolyIdx(j) = i;
                }
            }
            opt_vars.spatialDim = 3 * (opt_vars.piece_num - 1);
            break;
        }
        default: {
            for (long i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                k = opt_vars.pieceIdx(i);
                for (int l = 0; l < k; l++, j++) {
                    if (l < k - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i;
                        opt_vars.spatialDim += opt_vars.vPolytopes[2 * i].cols();
                    } else if (i < opt_vars.piece_num - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i + 1;
                        opt_vars.spatialDim += opt_vars.vPolytopes[2 * i + 1].cols();
                    }
                    opt_vars.hPolyIdx(j) = i;
                }
            }
        }
    }

    // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
    opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.piece_num);

    opt_vars.gradByPoints.resize(3, opt_vars.piece_num - 1);
    opt_vars.gradByTimes.resize(opt_vars.piece_num);
    opt_vars.partialGradByCoeffs.resize(8 * opt_vars.piece_num, 3);
    opt_vars.partialGradByTimes.resize(opt_vars.piece_num);
    return true;
}

bool ExpTrajOpt::setInitPsAndTs(vec_E<Eigen::Vector3d> &init_ps, vector<double> &init_ts) {
    opt_vars.default_init = false;
    if (opt_vars.times.size() != init_ts.size()) {
        return false;
    }
    if (opt_vars.points.cols() != init_ps.size()) {
        return false;
    }

    for (long i = 0; i < opt_vars.points.cols(); i++) {
        opt_vars.times[i] = init_ts[i];
        opt_vars.points.col(i) = init_ps[i];
    }
    opt_vars.times[opt_vars.times.size() - 1] = init_ts.back();
    return true;
}

double ExpTrajOpt::optimize(Trajectory &traj, const double &relCostTol) {
    Eigen::VectorXd x(opt_vars.temporalDim + opt_vars.spatialDim);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), opt_vars.temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + opt_vars.temporalDim, opt_vars.spatialDim);
    opt_vars.penalty_log.resize(8);
    opt_vars.penalty_log.setZero();
    if (opt_vars.times.minCoeff() < 1e-3) {
        cout << RED << " -- [TrajOpt] Error, the init times have zero, force return." << RESET << endl;
        cout << " -- Head PVAJ: " << endl;
        cout << opt_vars.headPVAJ << endl;

        cout << " -- Head PVAJ: " << endl;
        cout << opt_vars.tailPVAJ << endl;
        cout << " -- Times: " << endl;
        cout << opt_vars.times.transpose() << endl;
        return INFINITY;
    }
    gcopter::backwardMapTToTau(opt_vars.times, tau);

    switch (opt_vars.pos_constraint_type) {
        case 1: {
            MatDf p_e = opt_vars.points;
            xi = Map<const VectorXd>(p_e.data(), p_e.size());
            break;
        }
        default: {
            gcopter::backwardP(opt_vars.points, opt_vars.vPolyIdx, opt_vars.vPolytopes, xi);
            break;
        }
    }

    opt_vars.iter_num = 0;
    double minCostFunctional;
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = relCostTol;
    VecDf times_init = opt_vars.times;
    benchmark_utils::TimeConsuming ttt(" -- [ExpTrajOpt]", false);
    int ret = lbfgs::lbfgs_optimize(x,
                                    minCostFunctional,
                                    &ExpTrajOpt::costFunctional,
                                    nullptr,
                                    nullptr,
                                    &this->opt_vars,
                                    lbfgs_params);
    double dt = ttt.stop();
    gcopter::forwardMapTauToT(tau, opt_vars.times);
    if(cfg_.print_optimizer_log){
        cout << " -- [ExpOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
        cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
        cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
        cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
        cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
        cout << "\tOptimized Time: " << opt_vars.times.transpose() << endl;
    }


    if ((cfg_.penna_pos > 0 && opt_vars.penalty_log(1) > 0.15) ||
        // (cfg_.penna_vel > 0 && opt_vars.penalty_log(2) > cfg_.max_vel * cfg_.penna_margin) ||
        (cfg_.penna_acc > 0 && opt_vars.penalty_log(3) > cfg_.max_acc * cfg_.penna_margin) ||
        (cfg_.penna_omg > 0 && opt_vars.penalty_log(6) > cfg_.max_omg * cfg_.penna_margin) ||
        (cfg_.penna_thr > 0 && opt_vars.penalty_log(7) > cfg_.max_acc * cfg_.penna_margin)) {
        ret = -1;
        cout << " -- [ExpOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
        cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
        cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
        cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
        cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
        cout << "\tOptimized Time: " << opt_vars.times.transpose() << endl;
        cout << RED << " -- [ExpOpt] Opt failed, Omg or thr or Pos violation." << RESET << endl;
    }

    if (ret >= 0) {
        gcopter::forwardMapTauToT(tau, opt_vars.times);
        switch (opt_vars.pos_constraint_type) {
            case 1: {
                VecDf xi_e = xi;
                opt_vars.points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
                break;
            }
            default: {
                gcopter::forwardP(xi, opt_vars.vPolyIdx,
                                  opt_vars.vPolytopes, opt_vars.points);
                break;
            }
        }
        opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.temporalDim);
        opt_vars.minco.setParameters(opt_vars.points, opt_vars.times);
        opt_vars.minco.getTrajectory(traj);
    } else {
        traj.clear();
        minCostFunctional = INFINITY;
        cout << RED << " -- [MINCO] TrajOpt failed, " << lbfgs::lbfgs_strerror(ret) << RESET << endl;
        cout << "Init times: " << times_init.transpose() << endl;
    }
    return minCostFunctional;
}

ExpTrajOpt::ExpTrajOpt(const ros::NodeHandle &nh, const TrajOptConfig &cfg) {
    nh_ = nh;
    cfg_ = cfg;
    /// Use time as log file name
//    auto now = std::chrono::system_clock::now();
//    std::time_t t = std::chrono::system_clock::to_time_t(now);
//    std::tm tm = *std::localtime(&t);
//    std::stringstream ss;
//    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
//    std::string filename = ss.str() + "_exp_opt_log.csv";
    std::string filename = "exp_opt_log.csv";
    failed_traj_log.open(DEBUG_FILE_DIR(filename), std::ios::out | std::ios::trunc);
    penalty_log.open(DEBUG_FILE_DIR("exp_opt_penna.csv"), std::ios::out | std::ios::trunc);
    mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_opt/exp/mkr_arr", 1000);
    opt_vars.magnitudeBounds.resize(6);
    opt_vars.penaltyWeights.resize(7);
    opt_vars.magnitudeBounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
            cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
    opt_vars.penaltyWeights << cfg_.penna_pos, cfg_.penna_vel,
            cfg_.penna_acc, cfg_.penna_jerk,
            cfg_.penna_attract, cfg_.penna_omg,
            cfg_.penna_thr;
    opt_vars.rho = cfg_.penna_t;
    opt_vars.pos_constraint_type = cfg_.pos_constraint_type;
    opt_vars.block_energy_cost = cfg_.block_energy_cost;
    opt_vars.smooth_eps = cfg_.smooth_eps;
    opt_vars.integral_res = cfg_.integral_reso;
    opt_vars.quadrotor_flatness = cfg_.quadrotot_flatness;
}

ExpTrajOpt::~ExpTrajOpt() {
    failed_traj_log.close();
    penalty_log.close();
}

bool ExpTrajOpt::checkTrajMagnituteBound(Trajectory &out_traj) {
    if (cfg_.penna_vel > 0 && out_traj.getMaxVelRate() > 1.2 * cfg_.max_vel) {
        cout << RED << " -- [SVC] Minco exp_traj opt failed." << RESET << endl;
        cout << RED << "\t\tBackend Max vel:\t" << out_traj.getMaxVelRate() << " m/s" << RESET << endl;

        VisualUtils::DeleteMkrArr(mkr_pub_);
        out_traj.Visualize(mkr_pub_, "failed_exp", Color::Red(), 0.05, true, false);
//                VisualUtils::VisualizeTrajectory(mkr_pub_, out_traj, Color::Red(), 0.03, "failed_exp", 0.1);
        return false;
    }
    if (cfg_.penna_acc > 0 && out_traj.getMaxAccRate() > 1.2 * cfg_.max_acc) {
        cout << RED << " -- [SVC] Minco exp_traj opt failed." << RESET << endl;
        cout << RED << "\t\tBackend Max Acc:\t" << out_traj.getMaxAccRate() << " m/s." << RESET << endl;
        VisualUtils::DeleteMkrArr(mkr_pub_);
        out_traj.Visualize(mkr_pub_, "failed_exp", Color::Red(), 0.05, true, false);
//                VisualUtils::VisualizeTrajectory(mkr_pub_, out_traj, Color::Red(), 0.03, "failed_exp", 0.1);
        return false;
    }
    return true;
}


bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          PolytopeVec &sfcs,
                          Trajectory &out_traj) {
    /// Check if SFC is valid
    if (sfcs.empty()) {
        cout << RED << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
        return false;
    }

    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
        cout << RED << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
//        VisualUtils::VisualizePoint(mkr_pub_, headPVAJ.col(0),Color::Pink(),"ill_start",0.5,1);
//        VisualUtils::VisualizePoint(mkr_pub_, tailPVAJ.col(0),Color::Pink(),"ill_end",0.5,2);
//        cout << "headPVAJ: " << headPVAJ.col(0).transpose() << endl;
//        cout << "tailPVAJ: " << tailPVAJ.col(0).transpose() << endl;
//        cout << RED << "Killing the node." << RESET << endl;
//        exit(-1);
        return false;
    }

    for (auto poly: sfcs) {
        if (isnan(poly.GetPlanes().sum())) {
            cout << RED << " -- [TrajOpt] Error, the SFC containes NaN." << RESET << endl;
            return false;
        }
    }

    bool success{true};

    /// Setup optimization problems
    opt_vars.default_init = true;
    opt_vars.headPVAJ = headPVAJ;
    opt_vars.tailPVAJ = tailPVAJ;
    opt_vars.guide_path.clear();
    opt_vars.guide_t.clear();
    opt_vars.hPolytopes.resize(sfcs.size());
    for (long i = 0; i < sfcs.size(); i++) {
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
    }

    if (!setupProblemAndCheck()) {
        cout << RED << " -- [SVC] Minco corridor preprocess error." << RESET << endl;
        success = false;
    }

    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
        ROS_WARN(" -- [SUPER] in [ExpTrajOpt::optimize]: Optimization failed.");
        success = false;
    }

    if (success) {
        if (!checkTrajMagnituteBound(out_traj)) {
            success = false;
        } else {
            out_traj.start_WT = ros::Time::now().toSec();
        }
    }

    if (!success || cfg_.save_log_en) {
        failed_traj_log << 990419 << endl;
        failed_traj_log << headPVAJ << endl;
        failed_traj_log << tailPVAJ << endl;
        for (long i = 0; i < sfcs.size(); i++) {
            failed_traj_log << i << endl;
            failed_traj_log << sfcs[i].GetPlanes() << endl;
        }
    }

    return success;
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                          PolytopeVec &sfcs,
                          Trajectory &out_traj) {
    /// Check if hot init is valid
    if (guide_path.size() != guide_t.size()) {
        cout << RED << " -- [TrajOpt] Error, the guide trajectory has wrong path and time stamp." << RESET
             << endl;
        return false;
    }
    /// Check if SFC is valid
    if (sfcs.empty()) {
        cout << RED << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
        return false;
    }

    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
        cout << RED << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
//        VisualUtils::VisualizePoint(mkr_pub_, headPVAJ.col(0),Color::Pink(),"ill_start",0.5,1);
//        VisualUtils::VisualizePoint(mkr_pub_, tailPVAJ.col(0),Color::Pink(),"ill_end",0.5,2);
//        cout << "headPVAJ: " << headPVAJ.col(0).transpose() << endl;
//        cout << "tailPVAJ: " << tailPVAJ.col(0).transpose() << endl;
//        cout << RED << "Killing the node." << RESET << endl;
//        exit(-1);
        return false;
    }

    bool is_narrow = false;
    double corridor_radius = 2.0;
    for (const auto &poly: sfcs) {
        if (isnan(poly.GetPlanes().sum())) {
            cout << RED << " -- [TrajOpt] Error, the SFC containes NaN." << RESET << endl;
            return false;
        }

        // ! new function, when corridor is too narrow, the max vel should be reduced
        Vec3f interior;
        corridor_radius = geometry_utils::findInteriorDist(poly.GetPlanes(), interior);

        if(corridor_radius < 0.7)
        {
            is_narrow = true;
        }else{
            // opt_vars.magnitudeBounds[0] = cfg_.max_vel;
        }
    }

    if(is_narrow)
    {
        opt_vars.magnitudeBounds[0] = cfg_.max_vel;

        if(corridor_radius > 0.4)
        {
            opt_vars.rho = 50 +  (corridor_radius - 0.4) * 10000;
        }else{
            opt_vars.rho = 50;
        }
        
        
        // cout << RtED << " -- [TrajOpt] Narrow corridor, reduce max vel." << RESET << endl;
        // ROS_ERROR(" -- [TrajOpt] Narrow corridor, reduce max vel.");
    }else{
        opt_vars.magnitudeBounds[0] = cfg_.max_vel;
        opt_vars.rho = cfg_.penna_t;
    }


    bool success{true};

    /// Setup optimization problems
    opt_vars.default_init = false;
    opt_vars.headPVAJ = headPVAJ;
    opt_vars.tailPVAJ = tailPVAJ;
    opt_vars.guide_path = guide_path;
    opt_vars.guide_t = guide_t;
    opt_vars.hPolytopes.resize(sfcs.size());

    for (long i = 0; i < sfcs.size(); i++) {
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
    }

    if (!setupProblemAndCheck()) {
        cout << RED << " -- [SVC] Minco corridor preprocess error." << RESET << endl;
        success = false;
    }

    out_traj.clear();

    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
        cout << RED << " -- [SVC] Minco exp_traj opt failed." << RESET << endl;
        success = false;
    }
    penalty_log << opt_vars.penalty_log.transpose() << endl;

    if (success) {
        if (!checkTrajMagnituteBound(out_traj)) {
            success = false;
        } else {
            out_traj.start_WT = ros::Time::now().toSec();
        }
    }

    if (!success || cfg_.save_log_en) {
        failed_traj_log << 990419 << endl;
        failed_traj_log << headPVAJ << endl;
        failed_traj_log << tailPVAJ << endl;
        for (long i = 0; i < guide_t.size(); i++) {
            failed_traj_log << guide_t[i] << " ";
        }
        failed_traj_log << endl;
        for (long i = 0; i < guide_path.size(); i++) {
            failed_traj_log << guide_path[i].transpose() << " ";
        }
        failed_traj_log << endl;
        for (long i = 0; i < sfcs.size(); i++) {
            failed_traj_log << i << endl;
            failed_traj_log << sfcs[i].GetPlanes() << endl;
        }
    }
    return success;
}
