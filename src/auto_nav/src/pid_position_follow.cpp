#include "pid_position_follow.h"

double limtDiff(const double set_value , const double prev_value , double diff_lim , const double time){
  double diff = std::max(std::min(set_value - prev_value, diff_lim*time), -diff_lim*time);

  return prev_value + diff;
}

RobotCtrl::RobotCtrl()
{

    ros::NodeHandle nh("~");;

    //LAUNCH
    nh.param<bool>("simulink", simulink_, true);
    nh.param<std::string>("local_path_topic", local_path_topic_, "/local_path");
    nh.param<std::string>("global_path_topic", global_path_topic_, "/move_base/GlobalPlanner/plan");
    nh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/base_vel" );
    nh.param<std::string>("game_state_topic", game_state_topic_, "/game_state");

    nh.param<std::string>("gimbal_yaw_position_topic", gimbal_yaw_position_topic_, "/4ws_auto_infantry/yaw_steering_position_controller/command");
    nh.param<std::string>("gimbal_pitch_position_topic", gimbal_pitch_position_topic_,"/4ws_auto_infantry/pitch_steering_position_controller/command" );
    nh.param<std::string>("jointstate_topic", jointstate_topic_, "/joint_states");

    //YAML 
    global_frame_        = getParam<std::string>("track_follow/global_frame","/map");
    plan_freq_           = getParam<int>("track_follow/plan_frequency",1);
    goal_dist_tolerance_ = getParam<double>("track_follow/goal_dist_tolerance",0.25);
    slow_down_tolerance_ = getParam<double>("track_follow/slow_down_tolerance_",0.25);
    prune_ahead_distance_= getParam<double>("track_follow/prune_ahead_distance",0.6);
    race_sw_             = getParam<bool>("track_follow/race_sw",true);
    track_mode_          = getParam<int>("track_follow/track_mode",0);
    max_x_speed_         = getParam<double>("track_follow/max_x_speed",0.5);
    max_y_speed_         = getParam<double>("track_follow/max_y_speed",0.5);
    max_yaw_speed_       = getParam<double>("track_follow/set_yaw_speed",1.0);
    p_value_             = getParam<double>("track_follow/p_value",1.5);
    i_value_             = getParam<double>("track_follow/i_value",1.0);
    d_value_             = getParam<double>("track_follow/d_value",1.0);
    max_yaw_speed_       = getParam<double>("track_follow/max_yaw_speed",0.5);
    max_angle_diff_      = getParam<double>("track_follow/max_angle_diff",60)* M_PI / 180;
    vx_accel_limit       = getParam<double>("track_follow/vx_accel_limit", 0.5);
    vy_accel_limit       = getParam<double>("track_follow/vy_accel_limit", 0.5);

    //Dynamic_reconfigure init
    dynamic_reconfigure::Server<auto_nav::tracks_followConfig> *dsrv_;
    dsrv_ = new dynamic_reconfigure::Server<auto_nav::tracks_followConfig>(ros::NodeHandle("~/"));
    dynamic_reconfigure::Server<auto_nav::tracks_followConfig>::CallbackType cb = boost::bind(
                &RobotCtrl::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    local_path_pub_= nh.advertise<nav_msgs::Path>(local_path_topic_, 5);
    global_path_sub_ = nh.subscribe(global_path_topic_, 5, &RobotCtrl::GlobalPathCallback,this);
    game_state_sub_ = nh.subscribe(game_state_topic_,5,&RobotCtrl::Game_StateCallback,this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_,10);
    diff_yaw_pub_ = nh.advertise<std_msgs::Float64>("diff_yaw",10);
    filter_vx_pub_ = nh.advertise<std_msgs::Float64>("filter_vx",10);
    filter_vy_pub_ = nh.advertise<std_msgs::Float64>("filter_vy",10);
    marker_raw_pub = nh.advertise<visualization_msgs::Marker>("/ahead_maker", 1);
    tf_listener_ = std::make_shared<tf::TransformListener>();
    move_base_cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10);

    if (simulink_)
    {
        gimbal_yaw_position_cmd_ = nh.advertise<std_msgs::Float64>(gimbal_yaw_position_topic_,10);
        gimbal_pitch_position_cmd_ = nh.advertise<std_msgs::Float64>(gimbal_pitch_position_topic_,10);
        jointstate_sub_ = nh.subscribe("/joint_states",10, &RobotCtrl::JointstateCallback,this);

    }

    plan_timer_ = nh.createTimer(ros::Duration(1.0/plan_freq_),&RobotCtrl::Plan,this);

}

void RobotCtrl::reconfigureCB(auto_nav::tracks_followConfig& config,uint32_t level) {

    global_frame_         = config.global_frame;
    plan_freq_            = config.plan_frequency;
    goal_dist_tolerance_  = config.goal_dist_tolerance;
    slow_down_tolerance_  = config.slow_down_tolerance_;
    prune_ahead_distance_ = config.prune_ahead_distance;
    race_sw_              = config.race_sw;
    track_mode_           = config.track_mode;
    max_x_speed_          = config.max_x_speed;
    max_y_speed_          = config.max_y_speed;
    set_yaw_speed_        = config.set_yaw_speed;
    p_value_              = config.p_value;
    i_value_              = config.i_value;
    d_value_              = config.d_value;
    max_yaw_speed_        = config.max_yaw_speed;
    max_angle_diff_       = config.max_angle_diff * M_PI / 180;
    vx_accel_limit        = config.vx_accel_limit;
    vy_accel_limit        = config.vy_accel_limit;

     ROS_INFO("Reconfig: %d ",level);

    std::cout << vx_accel_limit << "fhuesifgeusigsui" << vy_accel_limit << std::endl;
            
}



void RobotCtrl::Plan(const ros::TimerEvent& event){
            ROS_INFO("start plan");
            

            if (game_state_ == 4 || race_sw_ == false)
            {   
            
            // 获取机器人yaw方位角 用于小陀螺
            geometry_msgs::PoseStamped robot_pose_1;
            GetGlobalRobotPose(tf_listener_, global_frame_ , robot_pose_1); 
            yaw_ = tf::getYaw(robot_pose_1.pose.orientation);
            //std::cout<<"yaw_"<<yaw_<<std::endl;         
            if (plan_ ){
                ROS_INFO("start if plan");

                auto begin = std::chrono::steady_clock::now();
                auto start = ros::Time::now();
                // 1. 更新全局到局部路径的坐标变换
                UpdateTransform(tf_listener_, global_frame_,
                                global_path_.header.frame_id, global_path_.header.stamp,
                                global2path_transform_);//source_time needs decided
                
                std::cout<<ros::Time::now()- start<<std::endl;

                // 2. 获取机器人当前在全局坐标系下的变换
                geometry_msgs::PoseStamped robot_pose;
                GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

                // 3. 根据当前机器人坐标从给定的全局路径获取跟踪最近点索引
                FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_distance_);  // TODO: double direct prune index is needed later!

                // 4. 生成裁减后的路径并转化到局部坐标系下
                nav_msgs::Path prune_path, local_path;

                local_path.header.frame_id = global_frame_;
                prune_path.header.frame_id = global_frame_;

                geometry_msgs::PoseStamped tmp_pose;
                tmp_pose.header.frame_id = global_frame_;

                TransformPose(global2path_transform_, robot_pose, tmp_pose);
                prune_path.poses.push_back(tmp_pose);
                
                int i = prune_index_;

                while (i < global_path_.poses.size() && i - prune_index_< 20 ){
                    
                    TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
                    prune_path.poses.push_back(tmp_pose);
                    i++;

                }
                // for (int i = prune_index_; i < global_path_.poses.size(); i++){
                //     TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
                //     prune_path.poses.push_back(tmp_pose);

                // }

                // 5. 通过裁减的路径生成样条曲线局部轨迹
                GenTraj(prune_path, local_path);
                local_path_pub_.publish(local_path);

                // 6. 跟踪局部轨迹并计算机器人速度
                geometry_msgs::Twist cmd_vel;
                FollowTraj(robot_pose, local_path, cmd_vel);

                // 7. 检查机器人是否进入减速段
                double goal_dist =  GetEuclideanDistance(robot_pose,global_path_.poses.back());
                ROS_INFO("goal_dist : %f",goal_dist);
                if (goal_dist < goal_dist_tolerance_ + slow_down_tolerance_){
                    double slow_p = goal_dist - goal_dist_tolerance_ / slow_down_tolerance_;
                    if (slow_p < 0.2){slow_p = 0.2;}
                    cmd_vel.linear.x *= slow_p;
                    cmd_vel.linear.y *= slow_p;
                    cmd_vel.angular.z = set_yaw_speed_;
                    ROS_INFO("Planning close , start slow down, slow_p = %f, vx = %f, vy = %f",
                                                slow_p, cmd_vel.linear.x,cmd_vel.linear.y);
                }
                // 8. 检查机器人是否已经在给定容忍距离内到达目标点
                if (goal_dist <= goal_dist_tolerance_
                    || prune_index_ == global_path_.poses.size() - 1){
                    plan_ = false;
                    actionlib_msgs::GoalID cancel_msgs;
                    move_base_cancel_pub_.publish(cancel_msgs);

                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = set_yaw_speed_;
                    cmd_vel.linear.z = 1;   // bool success or not
                    cmd_vel_pub_.publish(cmd_vel);
                    ROS_INFO("Planning Success!");
                    return;
                }


                cmd_vel_pub_.publish(cmd_vel);
                if (simulink_){
                GimbalCtrl();
                }
                auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
                ROS_INFO("Planning takes %f ms and passed %d/%d.",
                         plan_time.count()/1000.,
                         prune_index_,
                         static_cast<int>(global_path_.poses.size()));
            }
            else
            {
                geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = set_yaw_speed_;
                    cmd_vel.linear.z = 0;   // bool success or not
                    cmd_vel_pub_.publish(cmd_vel);
                    if (simulink_){
                    GimbalCtrl();
                    }
            }

        }
    
    else{
                geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = 0;
                    cmd_vel.linear.z = 0;   // bool success or not
                    cmd_vel_pub_.publish(cmd_vel);
                    if (simulink_){
                    GimbalCtrl();
                    }

    }
    ROS_INFO("end plan");
}




void RobotCtrl::FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist){

            double dist_threshold = 10;// threshold is 10 meters (basically never over 10m i suppose)
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist;
            if(prune_index!=0){
                sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index-1]);
            }else{
                sq_dist = 1e10;
            }

            double new_sq_dist = 0;
            while (prune_index < (int)path.poses.size()) {
                new_sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index]);
                if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {

                    //Judge if it is in the same direction and sq_dist is further than 0.3 meters
                    if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                        (path.poses[prune_index-1].pose.position.x - robot_pose.pose.position.x) +
                        (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                        (path.poses[prune_index-1].pose.position.y - robot_pose.pose.position.y) > 0
                        && sq_dist > prune_ahead_dist) {
                        prune_index--;
                    }else{
                        sq_dist = new_sq_dist;
                    }

                    break;
                }
                sq_dist = new_sq_dist;
                ++prune_index;
            }

            prune_index = std::min(prune_index, (int)(path.poses.size()-1));

        }

void RobotCtrl::FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel){
            
            

            if (track_mode_== 0){

            double diff_yaw = GetYawFromOrientation(traj.poses[0].pose.orientation)
                              - GetYawFromOrientation(robot_pose.pose.orientation);
            printf("diff_yaw: %f\n",diff_yaw);
            if(diff_yaw > M_PI){
                diff_yaw -= 2*M_PI;
            } else if(diff_yaw < -M_PI){
                diff_yaw += 2*M_PI;
            }

            if( diff_yaw > 0 ){
                cmd_vel.angular.z = std::min(10*diff_yaw, max_yaw_speed_);
            }else{
                cmd_vel.angular.z = std::max(10*diff_yaw, -max_yaw_speed_);
            }

            cmd_vel.linear.x = max_x_speed_*(1.0-std::abs(diff_yaw)/(max_angle_diff_));
            cmd_vel.linear.y = 0;
            if(std::abs(diff_yaw) > max_angle_diff_){
                cmd_vel.linear.x = 0;
            }
           
            }
            else{

            double diff_yaw = atan2((traj.poses.back().pose.position.y-robot_pose.pose.position.y),(traj.poses.back().pose.position.x-robot_pose.pose.position.x));
            if(diff_yaw != diff_yaw) {
                // ROS_ERROR("Has NAN!!!!! traj.poses[1].pose.position.y %f robot_pose.pose.position.y %f"
                //           "traj.poses[1].pose.position.x %f robot_pose.pose.position.x %f", 
                //           traj.poses[0].pose.position.y, robot_pose.pose.position.y, 
                //           traj.poses[0].pose.position.x, robot_pose.pose.position.x);
                // diff_yaw = 0;
            }


            double diff_distance = GetEuclideanDistance(robot_pose,traj.poses[1]);

            // set it from -PI t
            if(diff_yaw > M_PI){
                diff_yaw -= 2*M_PI;
            } else if(diff_yaw < -M_PI){
                diff_yaw += 2*M_PI;
            }



            double vx_global = max_x_speed_*cos(diff_yaw)*p_value_;//*diff_distance*p_value_;
            double vy_global = max_y_speed_*sin(diff_yaw)*p_value_;//*diff_distance*p_value_;

            if(diff_yaw != diff_yaw) {

                vx_global = 0;
                vy_global = 0;
            }

            // filter_vx = limtDiff(vx_global, filter_vx, vx_accel_limit, 1.0 / plan_freq_);
            // filter_vy = limtDiff(vy_global, filter_vy, vy_accel_limit, 1.0 / plan_freq_);

            // vx = filter_vx * cos(yaw_) + vy_global * sin(yaw_);
            // vy = -filter_vy * sin(yaw_) + vy_global * cos(yaw_);
            // vw = set_yaw_speed_;
            vx = vx_global * cos(yaw_) + vy_global * sin(yaw_);
            vy = -vx_global * sin(yaw_) + vy_global * cos(yaw_);
            vw = set_yaw_speed_;

            filter_vx = limtDiff(vx, filter_vx, vx_accel_limit, 1.0 / plan_freq_);
            filter_vy = limtDiff(vy, filter_vy, vy_accel_limit, 1.0 / plan_freq_);

            cmd_vel.linear.x = vx;
            cmd_vel.linear.y = vy;
            cmd_vel.angular.z = vw;

            // cmd_vel.linear.x = filter_vx;
            // cmd_vel.linear.y = filter_vy;
            // cmd_vel.angular.z = vw;

            filter_vx_data_.data = filter_vx;
            filter_vy_data_.data = filter_vy;

            filter_vx_pub_.publish(filter_vx_data_);
            filter_vy_pub_.publish(filter_vy_data_);

            publish_Marker(traj.poses.back().pose.position.x, traj.poses.back().pose.position.y, 0);

            std::cout << "=================================================="
                      << std::endl
                      << "diff_yaw: "
                      << diff_yaw
                      << std::endl
                      << "vx_global: "
                      << vx_global
                      << std::endl
                      << "vy_global: "
                      << vy_global
                      << std::endl
                      << "vx: "
                      << vx
                      << std::endl
                      << "vy: "
                      << vy
                      << std::endl
                      << "filter_vx: "
                      << filter_vx
                      << std::endl
                      << "filter_vy: "
                      << filter_vy
                      << std::endl;
            }

            //printf("diff_yaw: %f\n",diff_yaw);
            //printf("diff_distance: %f\n",diff_distance);

            // ::std_msgs::Float64 diff_yaw_data_;
            // diff_yaw_data_.data = diff_yaw;
            // diff_yaw_pub_.publish(diff_yaw_data_);





        }


void RobotCtrl::GlobalPathCallback(const nav_msgs::PathConstPtr & msg){
    printf("start global callback");
  if (!msg->poses.empty()){
      global_path_ = *msg;
      prune_index_ = 0;
      plan_ = true;
  }
  else{
    plan_ = false;
  }
  printf("end global callback");
}

void RobotCtrl::Game_StateCallback(const roborts_msgs::GameStatusPtr &msg ){
    printf("start Game_StateCallback");
    game_state_ = msg->game_state;
    printf("end Game_StateCallback");
}


void RobotCtrl::JointstateCallback(const sensor_msgs::JointStateConstPtr &msg){
    printf("start Game_StateCallback");
  cur_gimbal_yaw_position = msg->position.at(8);
  cur_gimbal_pitch_position = msg->position.at(10);
  printf("end Game_StateCallback");
}



void RobotCtrl::GimbalCtrl()
{  
  a_gimbal_pitch_position = 0;
  a_gimbal_yaw_position = -1 * yaw_;

  std_msgs::Float64 cmd;
  cmd.data = a_gimbal_pitch_position;
  gimbal_pitch_position_cmd_.publish(cmd);
  cmd.data = a_gimbal_yaw_position;
  gimbal_yaw_position_cmd_.publish(cmd);
}

void RobotCtrl::publish_Marker(float m2c_x,float m2c_y,float m2c_z)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = m2c_x;
    marker.pose.position.y = m2c_y;
    marker.pose.position.z = m2c_z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_raw_pub.publish(marker);
}


int main(int argc,char **argv)
{
  ros::init(argc, argv, "pid_position_follow");
  RobotCtrl robotctrl;
  ros::spin();
    return 0;
}