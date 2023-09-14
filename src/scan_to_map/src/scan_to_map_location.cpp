#include "scan_to_map_location.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Scan2MapLocation::Scan2MapLocation() : private_node_("~"), tf_listener_(tfBuffer_)
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> ICP location started.\033[0m");

    //初始化订阅者
    laser_scan_subscriber_ = node_handle_.subscribe("scan", 1, &Scan2MapLocation::ScanCallback, 
                                                      this,  ros::TransportHints().tcpNoDelay());

    map_subscriber_ = node_handle_.subscribe("map", 1, &Scan2MapLocation::MapCallback, this);

    odom_subscriber_ = node_handle_.subscribe("odom", 20, &Scan2MapLocation::OdomCallback,
                                                 this,  ros::TransportHints().tcpNoDelay());

    map_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 10);

    scan_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("scan_pointcloud", 10);

    removal_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("removal_pointcloud", 10);

    map_scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("map_scan", 10);

    // 注意，这里的发布器，发布的数据类型为 PointCloudT
    // ros中自动做了 PointCloudT 到 sensor_msgs/PointCloud2 的数据类型的转换
    icp_pointcloud_publisher_ = node_handle_.advertise<PointCloudT>("icp_pointcloud", 1, this);

    location_publisher_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("location_match", 1, this);
    // initialpose_subscriber_ = node_handle_.subscribe(
    //     "map", 1, &Scan2MapLocation::InitialposeCallback, this);

    // odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom_plicp", 50);

    relocalization_srv_ = node_handle_.advertiseService("relocalization", &Scan2MapLocation::RelocalizeCallback, this);

    // 指针的初始化
    cloud_map_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_scan_ = boost::shared_ptr<PointCloudT>(new PointCloudT());

    // 参数初始化
    InitParams();
}

Scan2MapLocation::~Scan2MapLocation()
{
}

/*
 * 的参数初始化
 */
void Scan2MapLocation::InitParams()
{
    private_node_.param<bool>("if_debug", if_debug_, true);

    private_node_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_node_.param<std::string>("base_frame", base_frame_, "base_link");
    private_node_.param<std::string>("map_frame", map_frame_, "map");
    private_node_.param<std::string>("lidar_frame", lidar_frame_, "lidar_link");
    // **** keyframe params: when to generate the keyframe scan
    // if either is set to 0, reduces to frame-to-frame matching
    private_node_.param<double>("kf_dist_linear", kf_dist_linear_, 0.1);
    private_node_.param<double>("kf_dist_angular", kf_dist_angular_, 5.0 * (M_PI / 180.0));
    kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;
    private_node_.param<int>("kf_scan_count", kf_scan_count_, 10);

    private_node_.param<int>("odom_queue_length", odom_queue_length_, 300);

    //ICP匹配相关参数
    private_node_.param<double>("ANGLE_SPEED_THRESHOLD", ANGLE_SPEED_THRESHOLD_, 1);   //角速度阈值，大于此值不发布结果
    private_node_.param<double>("AGE_THRESHOLD", AGE_THRESHOLD_, 1);   //scan与匹配的最大时间间隔
    private_node_.param<double>("ANGLE_UPPER_THRESHOLD", ANGLE_UPPER_THRESHOLD_, 10);    //最大变换角度
    private_node_.param<double>("ANGLE_THRESHOLD", ANGLE_THRESHOLD_, 0.01);    //最小变换角度
    private_node_.param<double>("DIST_THRESHOLD", DIST_THRESHOLD_, 0.01);    //最小变换距离
    private_node_.param<double>("SCORE_THRESHOLD_MIN", SCORE_THRESHOLD_MIN_, 0.005);    //代价阈值，代价低于此值则发送定位,自适应使用
    private_node_.param<double>("SCORE_THRESHOLD_MAX", SCORE_THRESHOLD_MAX_, 0.1);    //达到最大迭代次数或者到达差分阈值后后，代价仍高于此值，认为无法收敛,自适应使用
    private_node_.param<double>("SCORE_THRESHOLD_DIFF", SCORE_THRESHOLD_DIFF_, 1e-6);    //代价差分阈值，两次代价之差低于此值则发送定位,自适应使用
    private_node_.param<double>("ICP_NUM_ITER", ICP_NUM_ITER_, 100);   //ICP中的最大迭代次数

    //发布位姿的方差
    private_node_.param<double>("Variance_X", Variance_X, 0.01);   //x方向上方差
    private_node_.param<double>("Variance_Y", Variance_Y, 0.01);   //y方向上方差
    private_node_.param<double>("Variance_Yaw", Variance_Yaw, 0.01);   //yaw方向上方差

    private_node_.param<double>("Scan_Range_Max", Scan_Range_Max, 20);   //雷达数据点最大值
    private_node_.param<double>("Scan_Range_Min", Scan_Range_Min, 0.3);   //雷达数据点最小值

    private_node_.param<bool>("Use_TfTree_Always", Use_TfTree_Always, true);   //是否总是使用tf树读取变换

    //离群点滤波相关参数
    private_node_.param<double>("OutlierRemoval_MeanK", OutlierRemoval_MeanK, 30);   //设置在进行统计时考虑查询点邻近点数
    //设置判断是否为离群点 的 阈值  设置为1的 话 表示为：如果一个点的距离超过平均距离一个标准差以上则为离群点
    private_node_.param<double>("OutlierRemoval_StddevMulThresh", OutlierRemoval_StddevMulThresh, 1); 

    //体素滤波的边长
    private_node_.param<double>("VoxelGridRemoval_LeafSize", VoxelGridRemoval_LeafSize, 0.05); 

    //迭代障碍物去除
    //如果雷达点云中点在地图点云最近点大于此值，就认为该点为障碍点，有最大和最小值，会随着icp迭代的SCORE值按比例进行更新
    private_node_.param<double>("ObstacleRemoval_Distance_Max", ObstacleRemoval_Distance_Max, 2);     //最大距离
    private_node_.param<double>("ObstacleRemoval_Distance_Min", ObstacleRemoval_Distance_Min, 0.5);     //最小距离

    // 重定位
    private_node_.param<double>("Relocation_Weight_Score", Relocation_Weight_Score_, 0.5);     //重定位分数系数
    private_node_.param<double>("Relocation_Weight_Distance", Relocation_Weight_Distance_, 0.5);     //重定位距离系数
    private_node_.param<double>("Relocation_Weight_Yaw", Relocation_Weight_Yaw_, 0.5);     //重定位yaw角系数
    private_node_.param<double>("Relocation_ObstacleRemoval_Distance", Relocation_ObstacleRemoval_Distance_, 1.8);     //重定位离群值滤波剔除距离
}

/*
 * Odom回调函数
 */
void Scan2MapLocation::OdomCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg)
{
    std::lock_guard<std::mutex> lock(odom_lock_);
    odom_initialized_ = true;
    odom_queue_.push_back(*odometry_msg);
    // std::cout << "time of odometry_msg :" << odometry_msg->header.stamp.toSec() << std::endl;
    if (odom_queue_.size() > odom_queue_length_)    //弹出超过长度的数据
    {
        odom_queue_.pop_front();
    }
}

/*
 * Map回调函数 进行数据处理
 */
void Scan2MapLocation::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    std::cout << "frame_id of map:" << map_msg->header.frame_id <<std::endl;
    std::cout << "rostime of map:" << map_msg->header.stamp <<std::endl;

    map_initialized_ = true;

    OccupancyGridToPointCloud(map_msg, cloud_map_);     //map数据转为pointcloud数据
    // PointCloudVoxelGridRemoval(cloud_map_, VoxelGridRemoval_LeafSize);
    // 由于ros中自动做了 PointCloudT 到 sensor_msgs/PointCloud2 的数据类型的转换
    map_pointcloud_publisher_.publish(cloud_map_);   //发布pointcloud地图

    std::cout << "rostime of map cloud:" << cloud_map_->header.stamp <<std::endl;

}

/**
 * 重定位服务回调函数
*/
bool Scan2MapLocation::RelocalizeCallback(roborts_msgs::Relocate::Request& req, roborts_msgs::Relocate::Response& res)
{
    // Set the relocalization flag to true
    need_relocalization = true;

    return true;
}

/*
 * Scan回调函数 进行数据处理
 */
void Scan2MapLocation::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    scan_start_time_ = std::chrono::steady_clock::now();    //保存时间，用于最后计时

    //判断地图和里程计数据是否初始化，如果没有则退出
    if (!map_initialized_ || !odom_initialized_)  
    {
        return;
    }
    
    //计算获取变换时间
    start_time_ = std::chrono::steady_clock::now();
    //未初始化时，需要通过tf变换读取map_to_lidar和map_to_base的坐标变换
    //初始化函数与坐标变换
    if (!scan_initialized_)
    {
        if (if_debug_){
        std::cout << "initial scancallback " <<std::endl;}
        scan_time_ = scan_msg->header.stamp.toSec();   //初始化匹配时间
        // match_time_ = odom_queue_.back().header.stamp.toSec();

        //读取base_to_lidar静态变换
        base_to_lidar_ = Eigen::Isometry3d::Identity();
        if (!GetTransform(base_to_lidar_, base_frame_, lidar_frame_, scan_msg->header.stamp))
        {
            ROS_WARN("Did not get base pose at now");
            return;
        }
        // std::cout << "success get base_to_lidar_" <<std::endl;

        //读取map_to_base变换
        map_to_base_ = Eigen::Isometry3d::Identity();
        if (!GetTransform(map_to_base_, map_frame_, base_frame_, scan_msg->header.stamp))
        {
            ROS_WARN("Did not get base pose at now");
            return;
        }
        // std::cout << "success get map_to_base_" <<std::endl;
        scan_initialized_ = true;

        map_to_lidar_ =  map_to_base_ * base_to_lidar_;//获得map到lidar变换，用于雷达数据转地图数据
        // std::cout << "map_to_lidar_ : " << map_to_lidar_.matrix() << std::endl;
    }
    else    //初始化后，通过上一帧数据和odom数据计算得到map_to_lidar和map_to_base的坐标变换
    {
        if (if_debug_){
        std::cout << "scancallback initialized" <<std::endl;}
        // last_scan_time_ = scan_time_;     //保存上一次匹配时间
        last_match_time_ = match_time_;
        last_match_result_ = match_result_;     //保存上一次匹配结果
        scan_time_ = scan_msg->header.stamp.toSec();

        //判断是否超时，如果超时退出
        if (ros::Time::now().toSec() - scan_time_ > AGE_THRESHOLD_ )
        {
            ROS_WARN("Timeout for scan");
            scan_initialized_ = false;  //数据超时，需要重新初始化
            return;
        }
        Eigen::Isometry3d baselast_to_basenow = Eigen::Isometry3d::Identity();
        if (!GetOdomTransform(baselast_to_basenow, last_match_time_, scan_time_))
        {
            ROS_WARN("Did not get base pose on odom at now");
            return;
        }
        // std::cout << "baselast_to_basenow : " << baselast_to_basenow.matrix() << std::endl;

        map_to_base_ =  baselast_to_basenow *last_match_result_; 
        // map_to_base_ = last_match_result_; 
        map_to_lidar_ =  map_to_base_ * base_to_lidar_;  //获得map到lidar变换，用于雷达数据转地图数据
    }

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    if (if_debug_){
    std::cout << "获取map到base和lidar坐标变换用时: " << time_used_.count() << " 秒。" << std::endl;
    }

    if(Use_TfTree_Always)
    {
        scan_initialized_ = false;} //每次都使用tf读取结果

    // step1 进行数据类型转换
    start_time_ = std::chrono::steady_clock::now();
    ScanToPointCloudOnMap(scan_msg, cloud_scan_);
    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    if (if_debug_){
    std::cout << "scan格式转换处理用时: " << time_used_.count() << " 秒。" << std::endl;
    }

    //将雷达和地图点云保存为pcd文件
    if (save_pcd){
        pcl::io::savePCDFileASCII("/home/gh_luck/auto_infantry_ws/src/scan_to_map/bagfiles/cloud_scan_pcd.pcd",*cloud_scan_);
        pcl::io::savePCDFileASCII("/home/gh_luck/auto_infantry_ws/src/scan_to_map/bagfiles/cloud_map_pcd.pcd",*cloud_map_); 
        std::cout << "save pcb success" << std::endl;
        save_pcd = false;
    }

    // 重定位
    if(need_relocalization){
        // 进行离群点滤波，剔除离群点
        PointCloudVoxelGridRemoval(cloud_scan_, VoxelGridRemoval_LeafSize);
        PointCloudObstacleRemoval(cloud_map_, cloud_scan_, Relocation_ObstacleRemoval_Distance_);
        // 进行重定位
        if (ReLocationWithICP(match_result_, cloud_scan_, cloud_map_, map_to_base_)) {
            need_relocalization = false;
            // std::cout << "Robot relocated to (" << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta << ")" << std::endl;
        }
        return; 
    }
    else{   //正常定位
        // 进行离群点滤波，剔除离群点
        // PointCloudOutlierRemoval(cloud_scan_);
        PointCloudVoxelGridRemoval(cloud_scan_, VoxelGridRemoval_LeafSize);
        PointCloudObstacleRemoval(cloud_map_, cloud_scan_, ObstacleRemoval_Distance_Max);
        scan_pointcloud_publisher_.publish(cloud_scan_);
        map_pointcloud_publisher_.publish(cloud_map_);   //发布pointcloud地图

        //使用ICP进行点云匹配
        match_result_ = Eigen::Isometry3d::Identity();  
        if (!ScanMatchWithICP(match_result_, cloud_scan_, cloud_map_))
        {
            scan_initialized_ = false;  //数据错误，需要重新初始化
            return; 
        }

    }

    match_result_ = match_result_ * map_to_base_ ;     //将结果转换到map坐标系

    // 加上计算时位移
    Eigen::Isometry3d basebegin_to_basenow = Eigen::Isometry3d::Identity();
    if (!Get2TimeTransform(basebegin_to_basenow))
    {
        ROS_WARN("Did not get base pose at now");
        return;
    }

    match_result_ = basebegin_to_basenow * match_result_;     //将结果转换到map坐标系

    //将结果转为PoseStamped类型并发布
    //旋转矩阵转四元数
    Eigen::Quaterniond q = Eigen::Quaterniond(match_result_.rotation());
    std_msgs::Header header;
    header.stamp = scan_msg->header.stamp;;
    header.frame_id = "map";
    location_match.header = header;
    location_match.pose.pose.orientation.x = q.x();
    location_match.pose.pose.orientation.y = q.y();
    location_match.pose.pose.orientation.z = q.z();
    location_match.pose.pose.orientation.w = q.w();
    // location_match.pose.orientation = q;
    location_match.pose.pose.position.x = match_result_.translation()(0);
    location_match.pose.pose.position.y = match_result_.translation()(1);
    location_match.pose.pose.position.z = match_result_.translation()(2);
    //x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
    location_match.pose.covariance = {Variance_X, 0, 0, 0, 0, 0,  
                                    0, Variance_Y, 0, 0, 0, 0,  
                                    0, 0, 1e-9, 0, 0, 0,  
                                    0, 0, 0, 1e-9, 0, 0, 
                                    0, 0, 0, 0, 1e-9, 0,  
                                    0, 0, 0, 0, 0, Variance_Yaw}; 
    location_publisher_.publish(location_match);    //发送匹配结果

    //计算函数用时并发布
    scan_end_time_ = std::chrono::steady_clock::now();
    scan_time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(scan_end_time_ - scan_start_time_);
    if (if_debug_){
    std::cout << "ScanCallBack整体函数处理用时: " << scan_time_used_.count() << " 秒。" << std::endl;
    }

    scan_time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(scan_end_time_ - scan_last_end_time_);
    if (if_debug_){
    std::cout << "ScanCallback函数频率为: " << 1.0/scan_time_used_.count() << " HZ。" << std::endl;
    }
    scan_last_end_time_ = scan_end_time_;
}

/**
 * 使用ICP进行帧间位姿的计算
 */
bool Scan2MapLocation::ScanMatchWithICP(Eigen::Isometry3d &trans , PointCloudT::Ptr &cloud_scan_msg, PointCloudT::Ptr &cloud_map_msg)
{
    // icp_.setTransformationEpsilon (0.01);    //为中止条件设置最小转换差异
    // icp_.setEuclideanFitnessEpsilon(0.00001);
    // icp_.convergence_criteria_->setAbsoluteMSE(1e-6);
    // icp_.convergence_criteria_->setMaximumIterationsSimilarTransforms(3);

    icp_.setTransformationEpsilon (1e-6);    //为中止条件设置最小转换差异
    icp_.setMaxCorrespondenceDistance(5);
    icp_.setMaximumIterations (ICP_NUM_ITER_);    //设置匹配迭代最大次数

    // ICP 输入数据,输出数据的设置,还可以进行参数配置,这里使用默认参宿
    icp_.setInputSource(cloud_scan_msg);
    icp_.setInputTarget(cloud_map_msg);

    // 开始迭代计算
    pcl::PointCloud<pcl::PointXYZ> pointcloud_result;
    icp_.align(pointcloud_result);

    // 如果迭代没有收敛,不进行输出
    if (icp_.hasConverged() == false)
    {
        std::cout << "not Converged" << std::endl;
        return false;
    }

    // 收敛了之后, 获取坐标变换
    Eigen::Affine3f transfrom;
    transfrom = icp_.getFinalTransformation();
    // std::cout << "Affine3f transfrom : " << std::endl;
    // std::cout << transfrom.matrix() << std::endl;


    // 将Eigen::Affine3f转换成x, y, theta, 并打印出来
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);
    // std::cout << "ICP transfrom: (" << x << ", " << y << ", " << yaw << ")" << std::endl;

    double tranDist = sqrt(x*x + y*y);
    double angleDist = abs(yaw);

     if (if_debug_){
    std::cout<< "tranDist:" << tranDist << " angleDist: " << angleDist 
    << " score: " << icp_.getFitnessScore() << " angle speed: " << odom_queue_.back().twist.twist.angular.z << std::endl;
    }

    // 如果变换小于一定值，不发布结果，退出
    if(tranDist < DIST_THRESHOLD_ && angleDist < ANGLE_THRESHOLD_)
    {
        if(if_debug_){
        // \033[1;33m，\033[0m 终端显示成黄色
        std::cout << "\033[1;33m" << "Distance out of threshold" << "\033[0m" << std::endl;
        std::cout << "\033[1;33m"  << "tranDist:" << tranDist << " angleDist: " << angleDist 
        << " score: " << icp_.getFitnessScore() << " angle speed: " << odom_queue_.back().twist.twist.angular.z << "\033[0m" << std::endl;
        }
        return false;
    }

    //如果匹配结果不满足条件，退出
    if(angleDist > ANGLE_UPPER_THRESHOLD_ || icp_.getFitnessScore() > SCORE_THRESHOLD_MAX_
        || abs(odom_queue_.back().twist.twist.angular.z) > ANGLE_SPEED_THRESHOLD_)
    {
        // \033[1;33m，\033[0m 终端显示成黄色
        std::cout << "\033[1;33m" << "result out of threshold" << "\033[0m" << std::endl;
        std::cout << "\033[1;33m"  << "tranDist:" << tranDist << " angleDist: " << angleDist 
        << " score: " << icp_.getFitnessScore() << " angle speed: " << odom_queue_.back().twist.twist.angular.z << "\033[0m" << std::endl;
        return false;
    }

    // Eigen::Isometry3d transfrom_iso = Eigen::Isometry3d::Identity();
    // trans.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d(0, 0, 1)));
    // trans.pretranslate(Eigen::Vector3d(x, y, z));

    trans.matrix() = transfrom.matrix().cast<double>();      //Matrix4f类型转换为Isometry3d类型

    icp_pointcloud_publisher_.publish(pointcloud_result);   //发布匹配结果
    return true;
}

/**
 * 通过设置不同初始变换角度使用ICP，并对结果进行评分，实现重定位
*/
bool Scan2MapLocation::ReLocationWithICP(Eigen::Isometry3d &trans , PointCloudT::Ptr &cloud_scan_msg, PointCloudT::Ptr &cloud_map_msg, Eigen::Isometry3d &robot_pose)
{
    // 设置不同的旋转角度来生成不同的初始变换矩阵
    std::vector<Eigen::Matrix4f> initial_transforms;
    for (float angle = 0.0; angle < 360.0; angle += 10.0) {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(angle * M_PI / 180.0, Eigen::Vector3f::UnitZ()));
        initial_transforms.push_back(transform.matrix());
    }

    // 对每个初始变换矩阵进行ICP匹配，并计算匹配分数
    icp_.setInputSource(cloud_scan_msg);
    icp_.setInputTarget(cloud_map_msg);
    icp_.setMaximumIterations(50);
    icp_.setTransformationEpsilon(1e-8);
    icp_.setEuclideanFitnessEpsilon(0.01);
    std::vector<float> fitness_scores;

    std::vector<Eigen::Affine3f> Transformation_sources;

    for (int i = 0; i < initial_transforms.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_result (new pcl::PointCloud<pcl::PointXYZ>);
        icp_.align(*pointcloud_result, initial_transforms[i]);

        // 收敛了之后, 获取坐标变换
        Eigen::Affine3f transfrom;
        transfrom = icp_.getFinalTransformation();

        // 将Eigen::Affine3f转换成x, y, theta
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);

        double tranDist = sqrt(x*x + y*y);
        double angleDist = abs(yaw);

        float fitness_score = Relocation_Weight_Score_*icp_.getFitnessScore() + 
                                Relocation_Weight_Distance_ * tranDist + Relocation_Weight_Yaw_ * angleDist;

        fitness_scores.push_back(fitness_score);
        Transformation_sources.push_back(transfrom);

        std::cout << "tranDist:" << tranDist << " angleDist: " << angleDist << " score: " << icp_.getFitnessScore() << std::endl;
        std::cout << "fitness_score:" << fitness_score << std::endl;

        icp_pointcloud_publisher_.publish(pointcloud_result);   //发布匹配结果
    }

    // 找到分数最高的匹配结果
    int best_index = 0;
    float best_score = fitness_scores[0];
    for (int i = 1; i < fitness_scores.size(); i++) {
        if (fitness_scores[i] < best_score) {
        best_index = i;
        best_score = fitness_scores[i];
        }
    }

    trans.matrix() = Transformation_sources[best_index].matrix().cast<double>();      //Matrix4f类型转换为Isometry3d类型

    // 输出匹配结果
    std::cout << "Best transformation matrix:" << std::endl << Transformation_sources[best_index].matrix() << std::endl;
    std::cout << "Best fitness score:" << std::endl << best_score << std::endl;

    return true;
}

/**
 * 对点云中障碍点进行剔除
 * cloud_map_msg为参考点云，cloud_msg为需要剔除障碍点的点云
 */
void Scan2MapLocation::PointCloudObstacleRemoval(PointCloudT::Ptr &cloud_map_msg, PointCloudT::Ptr &cloud_msg, double Distance_Threshold)
{
    PointCloudT::Ptr cloud_removaled(new PointCloudT);;
    // std::cout<<"size of clound UnObstacleRemoval : " << cloud_msg->points.size() << std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;  //创建kd_tree对象
    kdtree.setInputCloud (cloud_map_msg); //设置搜索空间
    int K =1;   // k近邻收索

    // for (int i = 0; i < cloud_msg->points.size(); i++)
    int i = 0;
    while(i < cloud_msg->points.size())
    {
        PointT searchPoint = cloud_msg->points[i];

        // k近邻收索
        std::vector<int>pointIdxNKNSearch(K); //存储查询点近邻索引
        std::vector<float>pointNKNSquaredDistance(K); //存储近邻点对应平方距离

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0 )
        {
            if ( pointNKNSquaredDistance[0] > Distance_Threshold) //大于阈值认为是障碍点，剔除
            {
                cloud_msg->erase(cloud_msg->begin() + i);

                cloud_removaled->push_back(searchPoint);//从点云最后面插入一点

            }
            else
            {i++;}
        }
    }

    cloud_removaled->width = cloud_removaled->points.size() ;
    cloud_removaled->height = 1;
    cloud_removaled->is_dense = false; // contains nans

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    // header.frame_id = lidar_frame_;
    header.frame_id = "map";
    cloud_removaled->header = pcl_conversions::toPCL(header);
    removal_pointcloud_publisher_.publish(cloud_removaled);
    // std::cout<<"size of clound ObstacleRemovaled : " << cloud_msg->points.size() << std::endl;

}

/**
 * 对点云进行离群点滤波剔除离群点
 */
void Scan2MapLocation::PointCloudOutlierRemoval(PointCloudT::Ptr &cloud_msg)
{
    // std::cout<<"size of clound UnOutlierRemoval : " << cloud_msg->points.size() << std::endl;

    /* 声明 离群点滤波 后 的点云 */
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_OutRemove_filtered (new pcl::PointCloud<pcl::PointXYZ>);
 
    /* 声明 离群点滤波 的 类实例 */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_OutRemove;
    /* 设置输入点云 */
    sor_OutRemove.setInputCloud (cloud_msg);
    /* 设置在进行统计时考虑查询点邻近点数 */
    sor_OutRemove.setMeanK (30);
    /* 设置判断是否为离群点 的 阈值  设置为1的 话 表示为：如果一个点的距离超过平均距离一个标准差以上则为离群点 */
    sor_OutRemove.setStddevMulThresh (1.0);
     /* 执行滤波 返回 滤波后 的 点云 */
    sor_OutRemove.filter (*cloud_msg);
 
 
    /* 打印滤波前后的点数 */
    // std::cout << "size of clound OutlierRemovaled : " << cloud_msg->points.size() << std::endl;
}

/**
 * 对点云进行体素滤波剔除离群点
 */
void Scan2MapLocation::PointCloudVoxelGridRemoval(PointCloudT::Ptr &cloud_msg, double leafSize)
{
    // std::cout<<"size of clound UnVoxelGridRemoval : " << cloud_msg->points.size() << std::endl;

    /* 声明体素滤波的类实例 */
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_msg);
    voxel_grid.setLeafSize(leafSize, leafSize, leafSize);//注：体素不一定是正方体，体素可以是长宽高不同的长方体
    voxel_grid.filter(*cloud_msg);
    /* 打印滤波前后的点数 */
    if (if_debug_){
    std::cout << "size of clound VoxelGridRemovaled : " << cloud_msg->points.size() << std::endl;
    }
}

/**
 * 推测从上次icp的时间到当前时刻间的坐标变换
 * 使用匀速模型，根据当前的速度，乘以时间，得到推测出来的位移
 */
void Scan2MapLocation::GetPrediction(double &prediction_change_x,
                                   double &prediction_change_y,
                                   double &prediction_change_angle,
                                   double dt)
{
    // 速度小于 1e-6 , 则认为是静止的
    prediction_change_x = latest_velocity_.linear.x < 1e-6 ? 0.0 : dt * latest_velocity_.linear.x;
    prediction_change_y = latest_velocity_.linear.y < 1e-6 ? 0.0 : dt * latest_velocity_.linear.y;
    prediction_change_angle = latest_velocity_.linear.z < 1e-6 ? 0.0 : dt * latest_velocity_.linear.z;

    if (prediction_change_angle >= M_PI)
        prediction_change_angle -= 2.0 * M_PI;
    else if (prediction_change_angle < -M_PI)
        prediction_change_angle += 2.0 * M_PI;
}

/**
 * 从x,y,theta创建tf
 */
void Scan2MapLocation::CreateTfFromXYTheta(double x, double y, double theta, tf2::Transform &t)
{
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);
}

/**
 * 发布tf与odom话题
 */
void Scan2MapLocation::PublishTFAndOdometry()
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform = tf2::toMsg(base_in_odom_);

    // 发布 odom 到 base_link 的 tf
    tf_broadcaster_.sendTransform(tf_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    tf2::toMsg(base_in_odom_, odom_msg.pose.pose);
    odom_msg.twist.twist = latest_velocity_;

    // 发布 odomemtry 话题
    odom_publisher_.publish(odom_msg);
}

/**
 * 将占据栅格数据转为pointcloud点云格式数据
 */
void Scan2MapLocation::OccupancyGridToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr &map_msg, PointCloudT::Ptr &cloud_msg)
{
    std::cout << "start OccupancyGrid to PointCloud......" << std::endl;

    float ori_posx = map_msg->info.origin.position.x;   //初始坐标
    float ori_posy = map_msg->info.origin.position.y;
    float resolution = map_msg->info.resolution;    //分辨率
    float width = map_msg->info.width;              //地图长宽
    float height = map_msg->info.height;

    std_msgs::Header header;
    header.stamp = ros::Time(0.0);
    header.frame_id = "map";

    //OccupancyGrid转换为pcl点云
    cloud_msg->height   = 1;
    cloud_msg->is_dense = false;
    cloud_msg->header = pcl_conversions::toPCL(header);

    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
        {
            //@TODO
            if (map_msg->data[x + y * width] == 100)
            {
                // 首先声明一个 cloud_msg第i个点的 引用
                PointT point_tmp;

                // 获取第点在笛卡尔坐标系下的坐标
                point_tmp.x = (.5f + x) * resolution + ori_posx;
                point_tmp.y = (.5f + y) * resolution + ori_posy;
                point_tmp.z = 0;

                cloud_msg->points.push_back(point_tmp);

            }
        }

    cloud_msg->width = cloud_msg->points.size();

}

/**
 * 将scan数据转为pointcloud点云格式数据
 * 并转换到map坐标系下
 * 需要提前获得map_to_lidar_的变换
 */
void Scan2MapLocation::ScanToPointCloudOnMap(const sensor_msgs::LaserScan::ConstPtr &scan_msg, PointCloudT::Ptr &cloud_msg)
{
    // std::cout << "start Scan to pointcloud......" << std::endl;
    
    // 计算雷达数据点长度
    unsigned int point_len = 0;
    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];
        // std::cout << "range = " << range << std::endl; 

        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
        {
            continue;
        }
        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断
        if (range > scan_msg->range_min && range < scan_msg->range_max 
            && range > Scan_Range_Min && range < Scan_Range_Max)
        {
            point_len += 1;

        }
    }

    // 对容器进行初始化
    unsigned int point_num = 0;
    cloud_msg->points.resize(point_len);

    tran_start_time_ = std::chrono::steady_clock::now();
    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];
        // std::cout << "range = " << range << std::endl; 

        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
        {
            continue;
        }

        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断
        if (range > scan_msg->range_min && range < scan_msg->range_max 
            && range > Scan_Range_Min && range < Scan_Range_Max)
        {
            // 首先声明一个 cloud_msg第i个点的 引用
            PointT &point_tmp = cloud_msg->points[point_num];

            // 获取第i个点对应的角度
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // 获取第i个点在笛卡尔坐标系下的坐标
            // point_tmp.x = range * cos(angle);
            // point_tmp.y = range * sin(angle);
            // point_tmp.z = 0.0;

            Eigen::Vector3d point_vector(range * cos(angle), range * sin(angle), 0.0);

            point_vector = map_to_lidar_ * point_vector;     //进行坐标变换
            // std::cout << "point_vector after tranform = " << point_vector.transpose() << std::endl; 
            // std::cout << "point_vector(0) = " << point_vector(0) << std::endl; 

            // 获取map坐标系下在笛卡尔坐标系下的坐标
            point_tmp.x = point_vector(0);
            point_tmp.y = point_vector(1);
            point_tmp.z = 0.0;

            point_num += 1;

        }
    }


    tran_end_time_ = std::chrono::steady_clock::now();
    tran_time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(tran_end_time_ - tran_start_time_);
    // std::cout << "雷达数据转换后半处理用时: " << tran_time_used_.count() << " 秒。" << std::endl;

    // cloud_msg->width = scan_msg->ranges.size();
    cloud_msg->width = point_len;
    cloud_msg->height = 1;
    cloud_msg->is_dense = false; // contains nans

    std_msgs::Header header;
    header.stamp = scan_msg->header.stamp;
    // header.frame_id = lidar_frame_;
    header.frame_id = "map";
    cloud_msg->header = pcl_conversions::toPCL(header);
}


/**
 * 从odom中读取start_time 到end_time时间段的坐标变换
 * 保存到trans中
 * trans 类型： Eigen::Isometry3d
 */
bool Scan2MapLocation::GetOdomTransform(Eigen::Isometry3d &trans, double start_time, double end_time)
{
    ros::Rate loop_rate(100);
    while (odom_queue_.back().header.stamp.toSec() < end_time && ros::ok()
            && odom_queue_.front().header.stamp.toSec() < start_time )
    {
        ROS_WARN("wait for Odometry data ...");
        loop_rate.sleep();
        // std::cout << "time of front :" << odom_queue_.front().header.stamp.toSec() << std::endl;
        std::cout << "time of back :" << odom_queue_.back().header.stamp.toSec() << std::endl;
        // std::cout << "time of start_time :" << start_time << std::endl;
        std::cout << "time of end_time :" << end_time << std::endl;

    }

    // odom数据队列的头尾的时间戳要在雷达数据的时间段外
    if (odom_queue_.empty() ||
        odom_queue_.front().header.stamp.toSec() > start_time)
    {
        ROS_WARN("start_time out of Odometry data ...");
        return false;
    }

    std::cout << "time of front :" << odom_queue_.front().header.stamp.toSec() << std::endl;
    std::cout << "time of back :" << odom_queue_.back().header.stamp.toSec() << std::endl;
    std::cout << "time of start_time :" << start_time << std::endl;
    std::cout << "time of end_time :" << end_time << std::endl;

    // get start odometry at the beinning of the scan
    // 寻找与要求的时间段最相近的odom数据段
    double current_odom_time;
    nav_msgs::Odometry start_odom_msg_, end_odom_msg_;

    for (int i = 0; i < (int)odom_queue_.size(); i++)
    {
        current_odom_time = odom_queue_[i].header.stamp.toSec();

        if (current_odom_time < start_time)
        {
            start_odom_msg_ = odom_queue_[i];
            continue;
        }

        if (current_odom_time <= end_time)
        {
            end_odom_msg_ = odom_queue_[i];
        }
        else
            break;
    }

    double start_odom_time_, end_odom_time_;
    start_odom_time_ = start_odom_msg_.header.stamp.toSec();
    end_odom_time_ = end_odom_msg_.header.stamp.toSec();


    // 获取起始odom消息的位移与旋转
    Eigen::Isometry3d transBegin = Eigen::Isometry3d::Identity();
    tf2::Quaternion quat_start(
                start_odom_msg_.pose.pose.orientation.x,
                start_odom_msg_.pose.pose.orientation.y,
                start_odom_msg_.pose.pose.orientation.z,
                start_odom_msg_.pose.pose.orientation.w);

    double roll_start, pitch_start, yaw_start;   //定义存储r\p\y的容器
    double x_start, y_start, z_start;   //定义存储r\p\y的容器
    x_start = start_odom_msg_.pose.pose.position.x;
    y_start = start_odom_msg_.pose.pose.position.y;
    z_start = start_odom_msg_.pose.pose.position.z;

    tf2::Matrix3x3(quat_start).getRPY(roll_start, pitch_start, yaw_start);//进行转换

    Eigen::Matrix3d rotation_start;
    rotation_start = Eigen::AngleAxisd(yaw_start, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch_start, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll_start, Eigen::Vector3d::UnitX());

    transBegin.rotate(rotation_start);
    transBegin.pretranslate(Eigen::Vector3d(x_start, y_start, z_start));


    // 获取结束odom消息的位移与旋转
    Eigen::Isometry3d transEnd = Eigen::Isometry3d::Identity();
    tf2::Quaternion quat_end(
                end_odom_msg_.pose.pose.orientation.x,
                end_odom_msg_.pose.pose.orientation.y,
                end_odom_msg_.pose.pose.orientation.z,
                end_odom_msg_.pose.pose.orientation.w);

    double roll_end, pitch_end, yaw_end;   //定义存储r\p\y的容器
    double x_end, y_end, z_end;   //定义存储r\p\y的容器
    x_end = end_odom_msg_.pose.pose.position.x;
    y_end = end_odom_msg_.pose.pose.position.y;
    z_end = end_odom_msg_.pose.pose.position.z;
    tf2::Matrix3x3(quat_end).getRPY(roll_end, pitch_end, yaw_end);//进行转换

    //通过线性插值得到end_time时的变换结果, front和back表示队列时间，start和end表示待求时间
    double time_front_back = start_odom_time_ - end_odom_time_;
    double time_start_end = start_time - end_time;
    double interpolation_coefficient = time_front_back / time_start_end;    //插值系数

    Eigen::Matrix3d rotation_end;
    rotation_end = Eigen::AngleAxisd(yaw_end, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch_end, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll_end, Eigen::Vector3d::UnitX());

    transEnd.rotate(rotation_end);
    transEnd.pretranslate(Eigen::Vector3d(x_end, y_end, z_end));

    // 计算得到这段时间的坐标变换
    // trans =  transBegin.inverse() * transEnd;
    trans = transEnd *  transBegin.inverse();
    std::cout << "trans matrix =  \n" << trans.matrix() << std::endl;
    return true;

}

/**
 * 从tf树读取map到base_link两个时刻之间的坐标变换
 * 保存到trans中
 * trans 类型： Eigen::Isometry3d
 */
bool Scan2MapLocation::Get2TimeTransform(Eigen::Isometry3d &trans)
{
    // std::cout << "ready get transBegin" << std::endl;
    Eigen::Isometry3d transBegin = Eigen::Isometry3d::Identity();
    transBegin = map_to_base_;

    Eigen::Isometry3d transEnd = Eigen::Isometry3d::Identity();
    // transEnd = Eigen::Isometry3d::Identity();     //map到base的欧式变换矩阵4x4
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tfBuffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("DIDNT GET TRANSFORM ON Get2TimeTransform");
        ros::Duration(1.0).sleep();
        return false;
    }
    tf2::Quaternion quat(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

    double roll, pitch, yaw;   //定义存储r\p\y的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // std::cout << "trans roll, pitch, yaw =  \n" << roll << "  " << pitch << "  " << yaw << std::endl;

    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    transEnd.rotate(point_rotation);
    transEnd.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                transformStamped.transform.translation.y,
                                transformStamped.transform.translation.z));

    // 计算得到这段时间的坐标变换
    trans = transEnd *  transBegin.inverse();
    // std::cout << "trans matrix =  \n" << trans.matrix() << std::endl;
    return true;

}

/**
 * 从tf树读取parent_frame到child_frame的坐标变换
 * 保存到trans中
 * trans 类型： Eigen::Isometry3d
 */
bool Scan2MapLocation::GetTransform(Eigen::Isometry3d &trans , const std::string parent_frame, const std::string child_frame, const ros::Time stamp)
{
    bool gotTransform = false;
    trans = Eigen::Isometry3d::Identity();     //map到base的欧式变换矩阵4x4
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        gotTransform = true;
        transformStamped = tfBuffer_.lookupTransform(parent_frame, child_frame, stamp, ros::Duration(1.0));
        // std::cout << "input rostime of Transform:" << stamp <<std::endl;
        // std::cout << "output rostime of Transform:" << transformStamped.header.stamp <<std::endl;
    }
    catch (tf2::TransformException &ex)
    {
        gotTransform = false;
        ROS_WARN("DIDNT GET TRANSFORM %s %s IN B", parent_frame.c_str(), child_frame.c_str());
        ros::Duration(1.0).sleep();
        return false;
    }

    tf2::Quaternion quat(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

    double roll, pitch, yaw;   //定义存储r\p\y的容器
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    // std::cout << "trans roll, pitch, yaw =  \n" << roll << "  " << pitch << "  " << yaw << std::endl;

    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    // std::cout << "point_rotation = " << point_rotation <<std::endl;

    trans.rotate(point_rotation);
    trans.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                transformStamped.transform.translation.y,
                                transformStamped.transform.translation.z));


    return gotTransform;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_map_location_node"); // 节点的名字
    Scan2MapLocation scan_to_map;

    // ros::spin();
    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    return 0;
}