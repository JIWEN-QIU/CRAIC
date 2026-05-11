#include "rog_map/rog_map.h"

using namespace rog_map;

ROGMap::ROGMap(const ros::NodeHandle &nh, ROGMapConfig &cfg) : ProbMap(cfg), nh_(nh) {

    map_info_log_file_.open(DEBUG_FILE_DIR("rm_info_log.csv"), std::ios::out | std::ios::trunc);
    time_log_file_.open(DEBUG_FILE_DIR("rm_performance_log.csv"), std::ios::out | std::ios::trunc);

    if (cfg_.map_sliding_en) {
        mapSliding(Vec3f(0, 0, 0));
        inf_map_->mapSliding(Vec3f(0, 0, 0));
    } else {
        mapSliding(cfg_.fix_map_origin);
        inf_map_->mapSliding(cfg_.fix_map_origin);
    }

    /// Initialize visualization module
    if (cfg_.visualization_en) {
        vm_.occ_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/occ", 1);
        vm_.unknown_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/unk", 1);
        vm_.occ_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/inf_occ", 1);
        vm_.unknown_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/inf_unk", 1);
        if (cfg_.kd_tree_en) {
            vm_.kd_tree_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/kd_tree", 1);
        }
        if (cfg_.frontier_extraction_en) {
            vm_.frontier_pub = nh_.advertise<sensor_msgs::PointCloud2>("rog_map/frontier", 1);
        }
        if (cfg_.viz_time_rate > 0) {
            vm_.viz_timer = nh_.createTimer(ros::Duration(1.0 / cfg_.viz_time_rate), &ROGMap::vizCallback,
                                            this);
        }
    }
    vm_.mkr_arr_pub = nh_.advertise<visualization_msgs::MarkerArray>("rog_map/map_bound", 1);

    if (cfg_.ros_callback_en) {
        rc_.odom_sub = nh_.subscribe(cfg_.odom_topic, 1, &ROGMap::odomCallback, this);
        rc_.cloud_sub = nh_.subscribe(cfg_.cloud_topic, 1, &ROGMap::cloudCallback, this);
        rc_.update_timer = nh_.createTimer(ros::Duration(0.001), &ROGMap::updateCallback, this);
    }

    if (cfg_.swarm_en) {
        rc_.othertraj_sub = new ros::Subscriber[cfg_.drone_num];
        for (int i = 1; i <= cfg_.drone_num; i++) {
            if (i == cfg_.drone_id) {
                continue;
            }
            string topic_name = cfg_.other_traj_topics;
            topic_name += std::to_string(i);
            // topic_name += std::to_string(cfg_.drone_id);
            // ros::Subscriber* sub = new ros::Subscriber(nh_.subscribe(topic_name, 100, &ROGMap::OtherTrajCallback, this));
            ros::Subscriber* sub = new ros::Subscriber(nh_.subscribe<quadrotor_msgs::PolynomialTrajectory>(topic_name, 5000, boost::bind(&ROGMap::OtherTrajCallback, this, _1, i)));
            rc_.othertraj_sub[i-1] = *sub;

            // Add the subscriber and topic name to the map
            rc_.subscriber_topic_map[sub] = i;
        }
        rc_.other_trajs.resize(cfg_.drone_num);
        rc_.last_other_trajs.resize(cfg_.drone_num);
        rc_.last_other_traj_points.resize(cfg_.drone_num);
        rc_.othertraj_changed_flag.resize(cfg_.drone_num);
        for(int i=0;i<cfg_.drone_num;i++)
        {
            rc_.othertraj_changed_flag[i] = false;
        }
        
        rc_.mutual_avoid_timer = nh_.createTimer(ros::Duration(0.1), &ROGMap::MutualAvoidCallback, this);

        // rc_.drone_state_sub_ = nh_.subscribe(
        //     "/swarm_expl/drone_state_recv", 10, &ROGMap::droneStateMsgCallback, this);

        rc_.other_drone_pos.resize(cfg_.drone_num);

        string out_of_stuck_topic_name = "/out_of_stuck_";
        out_of_stuck_topic_name += std::to_string(cfg_.drone_id);
        rc_.outofstuck_sub = nh_.subscribe(out_of_stuck_topic_name, 1, &ROGMap::outOfStuckCallback, this);
    }

    writeMapInfoToLog(map_info_log_file_);
    map_info_log_file_.close();
    for (int i = 0; i < time_consuming_name_.size(); i++) {
        time_log_file_ << time_consuming_name_[i];
        if (i != time_consuming_name_.size() - 1) {
            time_log_file_ << ", ";
        }
    }
    time_log_file_ << endl;


    if (cfg.load_pcd_en) {
        string pcd_path = PCD_FILE_DIR(cfg.pcd_name);
        PointCloud::Ptr pcd_map(new PointCloud);
        if (pcl::io::loadPCDFile(pcd_path, *pcd_map) == -1) {
            cout << RED << "Load pcd file failed!" << RESET << endl;
            exit(-1);
        }
        Pose cur_pose;
        cur_pose.first = Vec3f(0, 0, 0);
        updateOccPointCloud(*pcd_map);
        cout << BLUE << " -- [ROGMap]Load pcd file success with " << pcd_map->size() << " pts." << RESET << endl;
        map_empty_ = false;
    }
}

bool ROGMap::isLineFree(const Vec3f &start_pt, const Vec3f &end_pt, const double &max_dis,
                        const vec_Vec3i &neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    while (raycaster.step(ray_pt)) {
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
            return false;
        }

        if (neighbor_list.empty()) {
            if (isOccupied(ray_pt)) {
                return false;
            }
        } else {
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto &nei: neighbor_list) {
                Vec3i shift_tmp = ray_pt_id_g + nei;
                if (isOccupied(shift_tmp)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool ROGMap::isLineFree(const Vec3f &start_pt, const Vec3f &end_pt, Vec3f &free_local_goal, const double &max_dis,
                        const vec_Vec3i &neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);
    free_local_goal = start_pt;
    while (raycaster.step(ray_pt)) {
        free_local_goal = ray_pt;
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
            return false;
        }

        if (neighbor_list.empty()) {
            if (isOccupied(ray_pt)) {
                return false;
            }
        } else {
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto &nei: neighbor_list) {
                Vec3i shift_tmp = ray_pt_id_g + nei;
                if (isOccupied(shift_tmp)) {
                    return false;
                }
            }
        }
    }
    free_local_goal = end_pt;
    return true;
}

void ROGMap::updateMap(const PointCloud &cloud, const Pose &pose) {
    TimeConsuming ssss("sss", false);
    if (cfg_.ros_callback_en) {
        std::cout << RED << "ROS callback is enabled, can not insert map from updateMap API." << RESET
                  << std::endl;
        return;
    }

    if (cloud.empty()) {
        static int local_cnt = 0;
        if (local_cnt++ > 100) {
            cout << YELLOW << "No cloud input, please check the input topic." << RESET << endl;
            local_cnt = 0;
        }
        return;
    }

    updateRobotState(pose);
    updateProbMap(cloud, pose);

    if (cfg_.kd_tree_en) {
        if (ikdtree.Root_Node == nullptr) {
            ikdtree.Build(cloud.points);
            return;
        }
        static PointCloudXYZIN kdtree_cloud;
        kdtree_cloud.clear();
        for (const auto &p: cloud.points) {
            static Vec3f p3f;
            p3f.x() = p.x;
            p3f.y() = p.y;
            p3f.z() = p.z;
            if ((p3f - pose.first).norm() > cfg_.raycast_range_max) {
                continue;
            } else {
                kdtree_cloud.points.push_back(p);
            }
        }
        ikdtree.Add_Points(kdtree_cloud.points, true);
    }
    writeTimeConsumingToLog(time_log_file_);
}

RobotState ROGMap::getRobotState() const {
    return robot_state_;
}

void ROGMap::updateRobotState(const Pose &pose) {
    robot_state_.p = pose.first;
    robot_state_.q = pose.second;
    robot_state_.rcv_time = ros::Time::now().toSec();
    robot_state_.rcv = true;
    robot_state_.yaw = geometry_utils::get_yaw_from_quaternion<double>(pose.second);
    updateLocalBox(pose.first);
}


void ROGMap::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
    updateRobotState(std::make_pair(
            Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                  odom_msg->pose.pose.position.z),
            Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)));


    static tf2_ros::TransformBroadcaster br_map_ego;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "drone";
    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w;
    br_map_ego.sendTransform(transformStamped);
}

void ROGMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    if (!robot_state_.rcv) {
        return;
    }
    double cbk_t = ros::Time::now().toSec();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
        return;
    }
    PointCloud temp_pc;
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();
}

void ROGMap::OtherTrajCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &traj_msg, const int& drone_id) {//ros::Subscriber* sub

    // Get the msg drone_id from the map
    // int msg_droneid = rc_.subscriber_topic_map[sub];

    // cout << "Drone " << cfg_.drone_id << " received trajectory from drone " << msg_droneid << endl;
    

    // Transform the traj to geometry_utils::Trajectory
    geometry_utils::Trajectory traj;
    quadrotor_msgs::PolynomialTrajectory msg = *traj_msg;

    if(msg.type == quadrotor_msgs::PolynomialTrajectory::HEART_BEAT){
        // ROS_ERROR("Drone %d received heartbeat from drone %d", cfg_.drone_id, drone_id);
        return;
    }

    // ROS_ERROR("Drone %d received trajectory from drone %d", cfg_.drone_id, drone_id);

    Matrix<double, 3, 8> coeff;
    Eigen::VectorXd eigenVec(8);  // 创建Eigen向量
    traj.clear();
    for (int i = 0; i < msg.piece_num_pos; i++) {
        // 使用Eigen的Map函数将std::vector数据映射到Eigen向量
        Eigen::Map<Eigen::VectorXd>(&eigenVec[0], 8) = Eigen::Map<Eigen::VectorXd>(&msg.coef_pos_x[i * 8], 8);
        coeff.row(0) = eigenVec.transpose();
        Eigen::Map<Eigen::VectorXd>(&eigenVec[0], 8) = Eigen::Map<Eigen::VectorXd>(&msg.coef_pos_y[i * 8], 8);
        coeff.row(1) = eigenVec.transpose();
        Eigen::Map<Eigen::VectorXd>(&eigenVec[0], 8) = Eigen::Map<Eigen::VectorXd>(&msg.coef_pos_z[i * 8], 8);
        coeff.row(2) = eigenVec.transpose();
        double t = msg.time_pos[i];
        traj.emplace_back(t, coeff);
    }
    traj.start_WT = msg.start_WT_pos.toSec();

    // put the traj into the map
    if(drone_id >= cfg_.drone_id){

        if(rc_.last_other_trajs[drone_id - 1].start_WT != traj.start_WT)
        {
            rc_.othertraj_changed_flag[drone_id - 1] = true;
        }
        // else{
        //     rc_.othertraj_changed_flag[drone_id - 1] = false;
        // }

        rc_.last_other_trajs[drone_id - 1] = rc_.other_trajs[drone_id - 1];
        rc_.other_trajs[drone_id - 1] = traj;
    } else {

        if(rc_.last_other_trajs[drone_id].start_WT != traj.start_WT)
        {
            rc_.othertraj_changed_flag[drone_id] = true;
        }
        // else{
        //     rc_.othertraj_changed_flag[drone_id] = false;
        // }

        rc_.last_other_trajs[drone_id] = rc_.other_trajs[drone_id];
        rc_.other_trajs[drone_id] = traj;
    }
}

void ROGMap::MutualAvoidCallback(const ros::TimerEvent &event) {

    // ROS_ERROR("Drone %d start mutual avoidance", cfg_.drone_id);

    // // reset the last traj to free, tried not good
    // vec_E<Vec3f> last_traj;
    // for(int i = 0; i < rc_.last_other_trajs.size(); i++){
    //     if(rc_.last_other_trajs[i].empty()){
    //         continue;
    //     }
    //     geometry_utils::Trajectory traj = rc_.last_other_trajs[i];
    //     double t = ros::Time::now().toSec() - traj.start_WT;
    //     // use 0.1s as step, pick out all the points on the traj
    //     double len = traj.getTotalDuration();
    //     double step = 0.05;
    //     for(t = 0; t < len; t += step){
    //         Vector3d pos = traj.getPos(t);
    //         Vec3f pos_f;
    //         pos_f.x() = pos(0);
    //         pos_f.y() = pos(1);
    //         pos_f.z() = pos(2);
    //         last_traj.push_back(pos_f);
    //     }
    // }
    // // set the last_traj to free
    // for (auto pos: last_traj){
    //     int hash_id = getHashIndexFromPos(pos);
    //     missPointUpdate(pos, hash_id, 100);
    // }

    PointCloud occ_cloud;
    // loop the trajs in rc_.other_trajs, and set the voxels on the trajs to OCCUPIED
    for(int i = 0; i < rc_.other_trajs.size(); i++){

        // cout << "Drone " << i << " traj size: " << rc_.other_trajs.size() << endl;

        if(rc_.other_trajs[i].empty()){
            continue;
        }
        geometry_utils::Trajectory traj = rc_.other_trajs[i];
        double t = ros::Time::now().toSec() - traj.start_WT;
        // if(t < 0){
        //     continue;
        // }else{
        
        // check if the traj is the same as last traj, if is, not set occ repeatly
        if(!rc_.othertraj_changed_flag[i]) 
        {
            continue;
        }else{
            rc_.othertraj_changed_flag[i] = false;
        }

        // use 0.1s as step, pick out all the points on the traj
        double len = traj.getTotalDuration();
        double step = 0.05;
        for(t = 0; t < len; t += step){
            Vector3d pos = traj.getPos(t);

            // print the pos
            // cout << "Drone " << i << " pos: " << pos.transpose() << endl;
            
            Vec3f pos_f;
            pos_f.x() = pos(0);
            pos_f.y() = pos(1);
            pos_f.z() = pos(2);

            // it other_traj's pos is near self pos, skip
            if(((pos_f - robot_state_.p).norm() < 0.4) || ((pos_f - robot_state_.p).norm() > 7.0)){
                continue;
            }
            
            PointType p;
            p.x = pos_f.x();
            p.y = pos_f.y();
            p.z = pos_f.z();
            p.intensity = 100;
            occ_cloud.push_back(p);
            // setOccupied(pos_f);
        }
        // }
    }

    // add other drone's pos into occ
    for(int i = 0; i < rc_.other_drone_pos.size(); i++){
        if((i+1) == cfg_.drone_id){
            continue;
        }

        Vec3f pos_f;
        pos_f.x() = rc_.other_drone_pos[i].x();
        pos_f.y() = rc_.other_drone_pos[i].y();
        pos_f.z() = rc_.other_drone_pos[i].z();
        // it other_uav's pos is far, skip
        if(((pos_f - robot_state_.p).norm() > 7.0)){
            continue;
        }

        PointType p;
        p.x = rc_.other_drone_pos[i].x();
        p.y = rc_.other_drone_pos[i].y();
        p.z = rc_.other_drone_pos[i].z();
        p.intensity = 100;
        occ_cloud.push_back(p);
    }

    // ROS_ERROR("Mutual avoidance add occ cloud size: %d", occ_cloud.size());

    updateOccPointCloud(occ_cloud);

}

void ROGMap::outOfStuckCallback(const std_msgs::EmptyConstPtr& msg)
{

    ROS_ERROR("Triggered out of stuck callback");

    // set the robot nearby voxel to be free
    Vec3f robot_pos = robot_state_.p;
    Vec3i robot_pos_id;
    posToGlobalIndex(robot_pos, robot_pos_id);

    // get the vector of the sphere neighbors
    double radius = 0.2;
    vec_Vec3i sphere_nei;
    // loop the square box to get the sphere neighbors
    int box_size = ceil(radius / cfg_.resolution);
    for(int i = -box_size; i <= box_size; i++){
        for(int j = -box_size; j <= box_size; j++){
            for(int k = -box_size; k <= box_size; k++){
                Vec3i nei(i, j, k);
                if((i*i + j*j + k*k) <= box_size*box_size){
                    nei = robot_pos_id + nei;
                    sphere_nei.push_back(nei);
                }
            }
        }
    }

    cout << "Sphere neighbors size: " << sphere_nei.size() << endl;

    // set the sphere neighbors to be free, it is complex
    // use missPointUpdate function
    for(auto nei: sphere_nei){
        Vec3f nei_f;
        globalIndexToPos(nei, nei_f);
        int hash_id = getHashIndexFromPos(nei_f);
        missPointUpdate(nei_f, hash_id, 999);
    }
}

// void ROGMap::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg)
// {
//     // recieve other drone pos
//     if (msg->drone_id == cfg_.drone_id) return;

//     Vec3f tmp;
//     tmp(0) = msg->pos[0];
//     tmp(1) = msg->pos[1];
//     tmp(2) = msg->pos[2];
//     rc_.other_drone_pos[msg->drone_id - 1] = tmp;
// }

void ROGMap::updateCallback(const ros::TimerEvent &event) {
    if (map_empty_) {
        static double last_print_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
            std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET << std::endl;
            last_print_t = cur_t;
        }
        return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
        return;
    } else if (rc_.unfinished_frame_cnt > 1) {
        std::cout << RED <<
                  " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET
                  << std::endl;
    }
    static PointCloud temp_pc;
    static Pose temp_pose;
    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

//    if () {
//        updateLocalBox(temp_pose.first);
//        cout << YELLOW << " -- [ROG WARN] Empty point cloud, skip update." << RESET << endl;
//        return;
//    }

    updateProbMap(temp_pc, temp_pose);
    if (cfg_.kd_tree_en && !temp_pc.empty()) {
        static PointCloudXYZIN kdtree_cloud;
        kdtree_cloud.clear();
        for (const auto &p: temp_pc.points) {
            Vec3f p3f;
            p3f.x() = p.x;
            p3f.y() = p.y;
            p3f.z() = p.z;
            if (!insideLocalMap(p3f)) { continue; }
            if ((p3f - temp_pose.first).norm() > cfg_.raycast_range_max) {
                continue;
            } else {
                kdtree_cloud.points.push_back(p);
            }
        }
        if (ikdtree.Root_Node == nullptr) {
            ikdtree.Build(kdtree_cloud.points);
            return;
        }
        ikdtree.Add_Points(kdtree_cloud.points, true);
    }
    writeTimeConsumingToLog(time_log_file_);

    mp_t += time_consuming_[0];
    mp_cnt++;
}

void ROGMap::vecEVec3fToPC2(const vec_E<Vec3f> &points, sensor_msgs::PointCloud2 &cloud) {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
        pcl_cloud[i].x = static_cast<float>(points[i][0]);
        pcl_cloud[i].y = static_cast<float >(points[i][1]);
        pcl_cloud[i].z = static_cast<float >(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "world";
}

void ROGMap::NearestNeighborSearch(const Vec3f &pt_in, Vec3f &pt_out, double &dis, const double &max_dis) {
    if (!cfg_.kd_tree_en) {
        pt_out.setZero();
        dis = max_dis;
        cout << RED << "kd_tree is not enable" << RESET << endl;
        return;
    }
    PointType searchPoint;
    searchPoint.x = static_cast<float >(pt_in.x());
    searchPoint.y = static_cast<float >(pt_in.y());
    searchPoint.z = static_cast<float >(pt_in.z());
    // K nearest neighbor search
    static const int K = 1;

    KD_TREE<PointType>::PointVector points_near(K);
    std::vector<float> points_near_square_dis(K);
    ikdtree.Nearest_Search(searchPoint, K, points_near, points_near_square_dis, max_dis);

    if (points_near.size() == 1) {
        pt_out.x() = points_near[0].x;
        pt_out.y() = points_near[0].y;
        pt_out.z() = points_near[0].z;
        dis = static_cast<double>(sqrt(points_near_square_dis[0]));
        if (cfg_.virtual_ceil_height - pt_in.z() < dis) {
            dis = cfg_.virtual_ceil_height - pt_in.z();
            pt_out = pt_in;
            pt_out.z() = cfg_.virtual_ceil_height;
        }
        if (pt_in.z() - cfg_.virtual_ground_height < dis) {
            dis = pt_in.z() - cfg_.virtual_ground_height;
            pt_out = pt_in;
            pt_out.z() = cfg_.virtual_ground_height;
        }
    } else if (points_near.size() == 0) {
        dis = max_dis;
    }
}

void ROGMap::vizCallback(const ros::TimerEvent &event) {
    if (!cfg_.visualization_en) {
        return;
    }
    if (map_empty_) {
        return;
    }
    Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;
    Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
    boundBoxByLocalMap(box_min, box_max);
//    box_max.z() -= cfg_.resolution + cfg_.safe_margin;
//    box_min.z() += cfg_.resolution + cfg_.safe_margin;
    if ((box_max - box_min).minCoeff() <= 0) {
        return;
    }
    if (cfg_.pub_unknown_map_en) {
        vec_E<Vec3f> unknown_map, inf_unknown_map;
        boxSearch(box_min, box_max, UNKNOWN, unknown_map);
        sensor_msgs::PointCloud2 cloud_msg;
        vecEVec3fToPC2(unknown_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.unknown_pub.publish(cloud_msg);
        if (cfg_.unk_inflation_en) {
            boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
            vecEVec3fToPC2(inf_unknown_map, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            vm_.unknown_inf_pub.publish(cloud_msg);
        }
    }
    if (cfg_.frontier_extraction_en) {
        vec_E<Vec3f> frontier_map;
        boxSearch(box_min, box_max, FRONTIER, frontier_map);
        sensor_msgs::PointCloud2 cloud_msg;
        vecEVec3fToPC2(frontier_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.frontier_pub.publish(cloud_msg);
    }
    vec_E<Vec3f> occ_map, inf_occ_map;
    boxSearch(box_min, box_max, OCCUPIED, occ_map);
    sensor_msgs::PointCloud2 cloud_msg;
    vecEVec3fToPC2(occ_map, cloud_msg);
    vm_.occ_pub.publish(cloud_msg);

    boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
    vecEVec3fToPC2(inf_occ_map, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    vm_.occ_inf_pub.publish(cloud_msg);


    VisualUtils::VisualizeBoundingBox(vm_.mkr_arr_pub, box_min, box_max, "visual_range", Color::Purple());
    ros::Duration(0.01).sleep();
    Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
    boundBoxByLocalMap(local_map_min, local_map_max);
    VisualUtils::VisualizeBoundingBox(vm_.mkr_arr_pub, local_map_min, local_map_max, "local_map",
                                      Color::Orange());
    VisualUtils::VisualizeBoundingBox(vm_.mkr_arr_pub, raycast_data_.cache_box_min, raycast_data_.cache_box_max,
                                      "raycast_range",
                                      Color::Green());


    if (cfg_.kd_tree_en) {
        BoxPointType box_point_type;
        box_point_type.vertex_min[0] = box_min.x();
        box_point_type.vertex_min[1] = box_min.y();
        box_point_type.vertex_min[2] = box_min.z();
        box_point_type.vertex_max[0] = box_max.x();
        box_point_type.vertex_max[1] = box_max.y();
        box_point_type.vertex_max[2] = box_max.z();
        KD_TREE<PointType>::PointVector points;
        ikdtree.Box_Search(box_point_type, points);
        PointCloud pc;
        for (auto point: points) {
            pc.push_back(point);
        }
        pc.width = pc.size();
        pc.height = 1;
        pcl::toROSMsg(pc, cloud_msg);
        cloud_msg.width = pc.size();
        cloud_msg.height = 1;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";
        vm_.kd_tree_pub.publish(cloud_msg);
    }


}


