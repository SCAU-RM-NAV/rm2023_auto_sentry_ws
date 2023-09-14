/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBOMASTER_CUBIC_SPLINE_ROS_H
#define ROBOMASTER_CUBIC_SPLINE_ROS_H

#include "cubic_spline.h"
#include "cpprobotics_types.h"

struct TrajInfo{
  std::vector<float> smoothed_path_x,
      smoothed_path_y,
      smoothed_path_yaw,
      smoothed_path_curvature,
      smoothed_path_s;
};

TrajInfo GenTraj(const nav_msgs::Path& path, nav_msgs::Path& smoothed_path, const float interval = 0.1){
  using namespace cpprobotics;
  TrajInfo traj_info;
  std::vector<float> path_x, path_y;

  for (int j = 0 ; j < path.poses.size(); ++j) {
    path_x.push_back(path.poses[j].pose.position.x);
    path_y.push_back(path.poses[j].pose.position.y);
  }

  // create a cubic spline interpolator
  Spline2D cublic_spline(path_x, path_y);
  // calculatae the new smoothed trajectory
  geometry_msgs::PoseStamped tmp_pose;
  tmp_pose.header.frame_id = path.header.frame_id;
  tmp_pose.header.stamp = ros::Time::now();
  for(float i=0; i<cublic_spline.s.back(); i += interval){

    std::array<float, 2> point_ = cublic_spline.calc_postion(i);
    traj_info.smoothed_path_x.push_back(point_[0]);
    traj_info.smoothed_path_y.push_back(point_[1]);
    float yaw = cublic_spline.calc_yaw(i);
    traj_info.smoothed_path_yaw.push_back(yaw);
    traj_info.smoothed_path_curvature.push_back(cublic_spline.calc_curvature(i));
    traj_info.smoothed_path_s.push_back(i);

    tmp_pose.pose.position.x = point_[0];
    tmp_pose.pose.position.y = point_[1];
    tmp_pose.pose.position.z = 0;
    tmp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    smoothed_path.poses.push_back(tmp_pose);

  }
  return traj_info;
}
#endif //ROBOMASTER_CUBIC_SPLINE_ROS_H
