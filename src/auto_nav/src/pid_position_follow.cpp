#include "pid_position_follow.h"

RobotCtrl::RobotCtrl()
{
    ros::NodeHandle nh("~");;
    nh.param<double>("max_x_speed", max_x_speed_, 1.0);
    nh.param<double>("max_y_speed", max_y_speed_, 1.0);

    nh.param<double>("set_yaw_speed", set_yaw_speed_, 0.0);

    nh.param<double>("p_value", p_value_, 0.5);
    nh.param<double>("i_value", i_value_, 1);
    nh.param<double>("d_value", d_value_, 1);

    nh.param<int>("plan_frequency", plan_freq_, 30);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.2);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.5);
    nh.param<std::string>("global_frame", global_frame_, "map");


    gimbal_yaw_position_cmd_ = nh.advertise<std_msgs::Float64>("/auto_car/yaw_steering_position_controller/command",10);
    gimbal_pitch_position_cmd_ = nh.advertise<std_msgs::Float64>("/auto_car/pitch_steering_position_controller/command",10);
    local_path_pub_= nh.advertise<nav_msgs::Path>("path", 5);
    global_path_sub_ = nh.subscribe("/move_base/GlobalPlanner/plan", 5, &RobotCtrl::GlobalPathCallback,this);
    game_state_sub_ = nh.subscribe("/game_state",5,&RobotCtrl::Game_StateCallback,this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/base_vel",10);
    //imu_sub_ = nh.subscribe("/imu", 1, &RobotCtrl::ImuCallback, this);
    jointstate_sub_ = nh.subscribe("/joint_states",10, &RobotCtrl::JointstateCallback,this);

    planner_server_ = nh.advertiseService("/pid_planner_status",&RobotCtrl::Follower_StateReq, this);

    tf_listener_ = std::make_shared<tf::TransformListener>();

    plan_timer_ = nh.createTimer(ros::Duration(1.0/plan_freq_),&RobotCtrl::Plan,this);

}

void RobotCtrl::Plan(const ros::TimerEvent& event){

    if (game_state_ == 4 && planner_state_ == 2)
        {            
            if (plan_ ){

                auto begin = std::chrono::steady_clock::now();
                auto start = ros::Time::now();
                // 1. Update the transform from global path frame to local planner frame
                UpdateTransform(tf_listener_, global_frame_,
                                global_path_.header.frame_id, global_path_.header.stamp,
                                global2path_transform_);//source_time needs decided
                std::cout<<ros::Time::now()- start<<std::endl;

                // 2. Get current robot pose in global path frame
                geometry_msgs::PoseStamped robot_pose;
                GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

                // 3. Check if robot has already arrived with given distance tolerance
                if (GetEuclideanDistance(robot_pose,global_path_.poses.back())<= goal_dist_tolerance_
                    || prune_index_ == global_path_.poses.size() - 1){
                    plan_ = false;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = set_yaw_speed_;
                    cmd_vel.linear.z = 1;   // bool success or not
                    cmd_vel_pub_.publish(cmd_vel);
                    ROS_INFO("Planning Success!");
                    return;
                }

                // 4. Get prune index from given global path
                FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);// TODO: double direct prune index is needed later!

                // 5. Generate the prune path and transform it into local planner frame
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

                // 6. Generate the cubic spline trajectory from above prune path
                GenTraj(prune_path, local_path);
                local_path_pub_.publish(local_path);

                // 7. Follow the trajectory and calculate the velocity
                geometry_msgs::Twist cmd_vel;
                FollowTraj(robot_pose, local_path, cmd_vel);
                cmd_vel_pub_.publish(cmd_vel);
                GimbalCtrl();

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
                    GimbalCtrl();
            }

        }
    
    else if(planner_state_ == 1)
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = set_yaw_speed_;
        cmd_vel.linear.z = 0;   // bool success or not
        cmd_vel_pub_.publish(cmd_vel);
        GimbalCtrl();
    }

    else{
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.z = 0;   // bool success or not
        cmd_vel_pub_.publish(cmd_vel);
        GimbalCtrl();

    }
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

            geometry_msgs::PoseStamped robot_pose_1;
            GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose_1); 

            yaw_ = tf::getYaw(robot_pose_1.pose.orientation);

            //double diff_yaw = GetYawFromOrientation(traj.poses[0].pose.orientation)- GetYawFromOrientation(robot_pose.pose.orientation);
            double diff_yaw = atan2((traj.poses[1].pose.position.y-robot_pose.pose.position.y ),( traj.poses[1].pose.position.x-robot_pose.pose.position.x));
            
            double diff_distance = GetEuclideanDistance(robot_pose,traj.poses[1]);

            // set it from -PI t
            if(diff_yaw > M_PI){
                diff_yaw -= 2*M_PI;
            } else if(diff_yaw < -M_PI){
                diff_yaw += 2*M_PI;
            }

            printf("diff_yaw: %f\n",diff_yaw);
            printf("diff_distance: %f\n",diff_distance);

            double vx_global = max_x_speed_*cos(diff_yaw)*p_value_;//*diff_distance*p_value_;
            double vy_global = max_y_speed_*sin(diff_yaw)*p_value_;//*diff_distance*p_value_;
            std::cout<<"yaw_"<<yaw_<<std::endl;
            std::cout<<"vx_gl  "<<vx_global<<"   vy_gl   "<<vy_global<<std::endl;
            
            cmd_vel.linear.x = vx_global * cos(yaw_) + vy_global * sin(yaw_);
            cmd_vel.linear.y = - vx_global * sin(yaw_) + vy_global * cos(yaw_);
            cmd_vel.angular.z = set_yaw_speed_;

        }


void RobotCtrl::GlobalPathCallback(const nav_msgs::PathConstPtr & msg){
  if (!msg->poses.empty()){
      global_path_ = *msg;
      prune_index_ = 0;
      plan_ = true;
  }
}

void RobotCtrl::Game_StateCallback(const roborts_msgs::GameStatusPtr &msg ){
    game_state_ = msg->game_state;
}


void RobotCtrl::JointstateCallback(const sensor_msgs::JointStateConstPtr &msg){
  cur_gimbal_yaw_position = msg->position.at(0);
  cur_gimbal_pitch_position = msg->position.at(1);
}


void RobotCtrl::ImuCallback(const sensor_msgs::Imu &msg)
{
  int a;
  //yaw_ = tf2::getYaw(msg.orientation);
  //ROS_INFO("imu_yaw:%f",yaw_);
}

// 服务处理函数
bool RobotCtrl::Follower_StateReq(roborts_msgs::PidPlannerStatus::Request& req,
          roborts_msgs::PidPlannerStatus::Response& resp){

    ROS_INFO("Request data : planner_state = %d, max_x_speed = %f, max_y_speed = %f, yaw_speed = %f"
            ,req.planner_state, req.max_x_speed, req.max_y_speed, req.yaw_speed);

    if (req.planner_state < 0 || req.planner_state > 2)
    {
        ROS_ERROR("Submitted data exception: Data cannot be negative");
        resp.result = 0;    //失败时返回0
        return false;
    }

    planner_state_ = req.planner_state;

    if(req.max_x_speed >0 && req.max_y_speed > 0)
    {
        max_x_speed_ = req.max_x_speed;
        max_y_speed_ = req.max_y_speed; 
    }
    if(req.yaw_speed > 0){
        set_yaw_speed_ = req.yaw_speed;
    }

    resp.result = 1;   //成功时返回1

    return true;
}

void RobotCtrl::GimbalCtrl()
{
  a_gimbal_pitch_position = 0;
  a_gimbal_yaw_position = yaw_;

  std_msgs::Float64 cmd;
  cmd.data = a_gimbal_pitch_position;
  //gimbal_pitch_position_cmd_.publish(cmd);
  cmd.data = a_gimbal_yaw_position;
  //gimbal_yaw_position_cmd_.publish(cmd);

}


int main(int argc,char **argv)
{
  ros::init(argc, argv, "pid_position_follow");
  RobotCtrl robotctrl;
  ros::spin();
    return 0;
}
