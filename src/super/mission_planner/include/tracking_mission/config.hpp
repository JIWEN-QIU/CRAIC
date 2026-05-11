//
// Created by yunfan on 2022/3/26.
//

#ifndef MISSION_PLANNER_CONFIG
#define MISSION_PLANNER_CONFIG

#include "ros/ros.h"
#include "vector"
#include "string"
#include "ros/package.h"
#include <type_utils/common_type_name.h>



namespace mission_planner {
    using namespace std;
    using namespace type_utils;
//    enum TriggerType {
//        RVIZ_CLICK = 0,
//        MAVROS_RC = 1,
//        TARGET_ODOM = 2
//    };
//
//    enum CmdType{
//        PATH = 0,
//        WAYPOINT = 1
//    };

    class TrakcingMissionConfig {
    public:
        // Bool Params
        string path_pub_topic, goal_pub_topic, odom_topic, waypoints_file_name, target_odom_topic;
        double odom_timeout;
        double publish_dt;
        double distance_to_target_z,distance_to_target_xy;


        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
                cout << param_value << endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
                param_value = default_value;
                cout << param_value << endl;
                return false;
            }
        }

        template<class T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
                for (int i = 0; i < param_value.size(); i++) {
                    cout << param_value[i] << " ";
                }
                cout << endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
                param_value = default_value;
                for (int i = 0; i < param_value.size(); i++) {
                    cout << param_value[i] << " ";
                }
                cout << endl;
                return false;
            }
        }

        ros::NodeHandle nh_;

        TrakcingMissionConfig() {};

        TrakcingMissionConfig(const ros::NodeHandle &nh_priv) {
            nh_ = nh_priv;

            LoadParam("tracking_mission/odom_timeout", odom_timeout, 0.1);
            LoadParam("tracking_mission/publish_dt", publish_dt, 1.0);
            LoadParam("tracking_mission/goal_pub_topic", goal_pub_topic, string("/planner/goal"));
            LoadParam("tracking_mission/odom_topic", odom_topic, string("/lidar_slam/odom"));
            LoadParam("tracking_mission/target_odom_topic", target_odom_topic, string("/target/odom"));
            LoadParam("tracking_mission/distance_to_target_xy", distance_to_target_xy, 1.0);
            LoadParam("tracking_mission/distance_to_target_z", distance_to_target_z, -0.5);

        }

    };
}
#endif //PLANNER_CONFIG_HPP
