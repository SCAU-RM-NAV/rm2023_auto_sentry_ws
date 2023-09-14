# rm2023_auto_sentry_ws
 RM2023华南农业大学Taurus机器人战队哨兵定位导航算法与仿真开源

RM2022自动步兵开源链接https://github.com/SCAU-RM-NAV/rm2022_auto_infantry_ws

## 简介
本项目提供一种低成本二维激光雷达的哨兵导航定位算法实现，并在RMUC仿真赛场上实现，将详细讲述技术点、使用方法和原理，希望能对其他队伍有所帮助。受限于时间原因，部分内容将以后进行补充。

我们哨兵机器人使用了二维激光雷达的方案，整体算法架构与navigation框架类似，在此基础上改进了定位和轨迹跟踪算法，使之能够实现快速的定位修正和小陀螺过程中的导航，同时实现了定位丢失后的重定位。

项目环境
* Ubuntu20.04
* ros Noetic


## 使用方法
克隆存储库并catkin_make：
```
cd ~/catkin_ws/src
git clone https://github.com/SCAU-RM-NAV/rm2023_auto_sentry_ws.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
启动仿真
```
roslaunch simple_meca_car race.launch 
```
启动定位与导航
```
roslaunch auto_nav navi_simple_meca_car_pid.launch 
```
