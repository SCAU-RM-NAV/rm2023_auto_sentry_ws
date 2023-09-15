#ifndef PID_POSITION_FOLLOW_H
#define PID_POSITION_FOLLOW_H

#include <cmath>
#include <float.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/JointState.h"

#include "utility.h"
#include <tf2/utils.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib_msgs/GoalID.h>

#include "cubic_spline/cubic_spline_ros.h"
#include "utility.h"
#include <Eigen/Eigen>
#include <chrono>

#include <roborts_msgs/GameStatus.h>

#include <yaml-cpp/yaml.h>

#include "dynamic_reconfigure/server.h"
#include "auto_nav/tracks_followConfig.h"

#define DEBUG false

using namespace std;

class RobotCtrl {
    public:
    RobotCtrl();
    ~RobotCtrl() = default;
    void JointstateCallback(const sensor_msgs::JointStateConstPtr &msg);
    void GimbalCtrl();
    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);
    void Game_StateCallback(const roborts_msgs::GameStatusPtr &msg );
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel);
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist);
    void Plan(const ros::TimerEvent& event);
    void reconfigureCB(auto_nav::tracks_followConfig& config,uint32_t level);

private:
    ros::Publisher gimbal_yaw_position_cmd_;
    ros::Publisher gimbal_pitch_position_cmd_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher move_base_cancel_pub_;

    ros::Subscriber imu_sub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber jointstate_sub_;
    ros::Subscriber game_state_sub_;
    ros::Timer plan_timer_;

    std::shared_ptr<tf::TransformListener> tf_listener_;
    tf::StampedTransform global2path_transform_;

    nav_msgs::Path global_path_;

    bool simulink_ = true; 

    std::string local_path_topic_;
    std::string global_path_topic_;
    std::string cmd_vel_topic_;
    std::string game_state_topic_;

    std::string gimbal_yaw_position_topic_;
    std::string gimbal_pitch_position_topic_;
    std::string jointstate_topic_;

    bool plan_ = false;
    int prune_index_ = 0;

    std::string global_frame_;
    int plan_freq_;

    double max_x_speed_;
    double max_y_speed_;
    double max_yaw_speed_;
    double max_angle_diff_;
    double set_yaw_speed_;
    
    double p_value_;
    double i_value_;
    double d_value_;

    bool race_sw_ = true;

    int track_mode_ = 0 ;
    
    double goal_dist_tolerance_;
    double slow_down_tolerance_;
    double prune_ahead_distance_;

    double a_gimbal_yaw_position;  //
    double a_gimbal_pitch_position;  //

    double cur_gimbal_yaw_position;
    double cur_gimbal_pitch_position;

    double yaw_;  //机器人航向角
    
    uint8_t game_state_;

    double vx;
    double vy;
    double vw;
    double filter_vx;
    double filter_vy;
    double filter_vw;

    double vx_accel_limit;
    double vy_accel_limit;

private:
  ::ros::Publisher diff_yaw_pub_;
  ::ros::Publisher filter_vx_pub_;
  ::ros::Publisher filter_vy_pub_;
  ::std_msgs::Float64 filter_vx_data_;
  ::std_msgs::Float64 filter_vy_data_;

  ros::Publisher marker_raw_pub;

  void publish_Marker(float m2c_x,float m2c_y,float m2c_z);
};



//弧度制归一化
double normalizeRadian(const double angle)
{
   double n_angle = std::fmod(angle, 2 * M_PI);
   n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
   return n_angle;
}

double ABS_limit(double value,double limit)
{
  if(value<limit && value>-limit)
  {
    return 0;
  }
  else
  {
    return value;
  }

}

template<typename T>
T getParam(const std::string& name,const T& defaultValue)//This name must be namespace+parameter_name
{
    T v;
    if(ros::param::get(name,v))//get parameter by name depend on ROS.
    {
        ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
        return v;
    }
    else 
        ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
    return defaultValue;//if the parameter haven't been set,it's value will return defaultValue.
}


#endif 
