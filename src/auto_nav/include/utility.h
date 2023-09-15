#ifndef ROBOMASTER_UTILITY_H
#define ROBOMASTER_UTILITY_H
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

double GetYawFromOrientation(const geometry_msgs::Quaternion& orientation){

  tf::Quaternion q;
  tf::quaternionMsgToTF(orientation,q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double GetEuclideanDistance(const geometry_msgs::PoseStamped & pose_1,
                            const geometry_msgs::PoseStamped & pose_2){
  return hypot(pose_1.pose.position.x-pose_2.pose.position.x,
               pose_1.pose.position.y-pose_2.pose.position.y);
}

bool GetGlobalRobotPose(const std::shared_ptr<tf::TransformListener>& tf_listener,
                        const std::string& target_frame,
                        geometry_msgs::PoseStamped& robot_global_pose){
  tf::Stamped<tf::Pose> robot_pose_tf;
  robot_pose_tf.setIdentity();
  robot_pose_tf.frame_id_ = "base_link";
  robot_pose_tf.stamp_ = ros::Time();

  tf::Stamped<tf::Pose> robot_global_pose_tf;
  try{
    tf_listener->transformPose( target_frame, robot_pose_tf, robot_global_pose_tf);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Failed to transform robot pose: %s", ex.what());
    return false;
  }
  tf::poseStampedTFToMsg(robot_global_pose_tf, robot_global_pose);
  return true;
}

bool TransformPose(const std::shared_ptr<tf::TransformListener>& tf_listener,
                   const std::string& target_frame,
                   const geometry_msgs::PoseStamped& input_pose,
                   geometry_msgs::PoseStamped& output_pose,
                   const ros::Time& target_time = ros::Time::now(),
                   const ros::Duration& timeout = ros::Duration(0.5)){

                  
  if (target_frame == input_pose.header.frame_id){
    output_pose = input_pose;
    return true;
  }
     
  tf::Stamped<tf::Pose> input_pose_tf, output_pose_tf;
  tf::poseStampedMsgToTF(input_pose,input_pose_tf);

  try{
    tf_listener->waitForTransform( target_frame, target_time, 
                                   input_pose.header.frame_id, input_pose.header.stamp, 
                                   input_pose.header.frame_id, timeout );
    tf_listener->transformPose( target_frame, input_pose_tf, output_pose_tf);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Failed to transform pose: %s", ex.what());
    return false;
  }

  tf::poseStampedTFToMsg(output_pose_tf, output_pose);
  return true;
}

bool TransformPose(const std::shared_ptr<tf::TransformListener>& tf_listener,
                   const std::string& target_frame,
                   const geometry_msgs::PoseStamped& input_pose,
                   tf::Stamped<tf::Pose>& output_pose_tf,
                   const ros::Time& target_time = ros::Time::now(),
                   const ros::Duration& timeout = ros::Duration(0.5)){


  if (target_frame == input_pose.header.frame_id){
    tf::poseStampedMsgToTF(input_pose, output_pose_tf);
    return true;
  }

  tf::Stamped<tf::Pose> input_pose_tf;
  tf::poseStampedMsgToTF(input_pose,input_pose_tf);

  try{
    tf_listener->waitForTransform( target_frame, target_time,
                                   input_pose.header.frame_id, input_pose.header.stamp,
                                   input_pose.header.frame_id, timeout );
    tf_listener->transformPose( target_frame, input_pose_tf, output_pose_tf);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Failed to transform pose: %s", ex.what());
    return false;
  }

  return true;
}


void TransformPose(const tf::StampedTransform transform,
                   const geometry_msgs::PoseStamped& input_pose,
                   geometry_msgs::PoseStamped& output_pose){

  tf::Stamped<tf::Pose> input_pose_tf;
  tf::poseStampedMsgToTF(input_pose, input_pose_tf);
  input_pose_tf.setData(transform * input_pose_tf);
  input_pose_tf.stamp_ = transform.stamp_;
  input_pose_tf.frame_id_ = transform.frame_id_;
  tf::poseStampedTFToMsg(input_pose_tf, output_pose);
}

bool UpdateTransform(const std::shared_ptr<tf::TransformListener>& tf_listener,
                     const std::string& target_frame,
                     const std::string& source_frame,
                     const ros::Time& source_time,
                     tf::StampedTransform& target_to_source_transform,
                     const ros::Time& target_time = ros::Time::now(),
                     const ros::Duration& timeout = ros::Duration(0.5)){
  try{
    tf_listener->waitForTransform(target_frame, target_time,
                                  source_frame, source_time,
                                  source_frame, timeout);
    tf_listener->lookupTransform(target_frame, target_time,
                                 source_frame, source_time,
                                 source_frame, target_to_source_transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Failed to update transform: %s", ex.what());
    return false;
  }

  return true;


}
#endif //PROJECT_UTILITY_H
