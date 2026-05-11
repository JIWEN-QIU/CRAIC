#include <geometry_utils/trajectory.h>
#include <optimization_utils/waypoint_trajectory_optimizer.h>
#include <ros/ros.h>
#include <visualization_utils/visualization_utils.h>
#include "./sfc_gen.hpp"
using namespace type_utils;
using namespace geometry_utils;
using namespace visualization_utils;
using namespace std;


ros::Publisher mkrarr_pub;

void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    // number of polyhedra
    int M = 13;
    // random size range
    double rmin = 3.0;
    double rmax = 5.0;
    Eigen::Vector3d zeroVec(0.0, 0.0, 0.0);
    // positin in first and last polyhedron, for trajectory generation.
    MatDf head_tail;
    std::vector<Eigen::MatrixXd>  sfcs = genPolySFC(M, zeroVec, rmin, rmax, 0.95, 16, 0.2, 0.7, head_tail);
    simplifySFC(sfcs);

    for (auto &sfc: sfcs) {
        Eigen::MatrixX4d hpoly;
        convertPointNormFormToPlanEquation(sfc,hpoly);
        Polytope p(hpoly);
        p.Visualize(mkrarr_pub);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    mkrarr_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1);
    ros::Subscriber click_sub = nh.subscribe("/goal", 1, &clickCallback);

    ros::spin();
    return 0;
}