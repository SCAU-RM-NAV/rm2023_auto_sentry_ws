#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy #导入rospy库
import actionlib #导入actionlib 库
import os,inspect #导入os库
import copy
import yaml
import time
import threading
import shutil
from math import pi #导入圆周率pi
import math
import tf
from tf_conversions import transformations    #库函数

from actionlib_msgs.msg import * #导入actionlib的所有模块
from geometry_msgs.msg import Pose, Point, Quaternion, Twist #导入四个消息数据类型，姿态，目标点，四元数，运动消息Twist
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #导入movebase的两个消息数据类型
from tf.transformations import quaternion_from_euler #导入tf变换库的欧拉角转四元数库
from std_msgs.msg import Int8 
from std_msgs.msg import String #导入标准消息的字符串消息数据格式
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from roborts_msgs.msg import EnemyInfo,GimbalFdb ,GimbalCtrl,GameStatus

count_marker = 0
MARKERS_MAX = 200
markerArray = MarkerArray()

nav_goals=[]
goal_dict=dict()
place_name=""

current_x = 0
current_y = 0
current_pos = [0,0]
xy_tolerate = 0.3   #离目标点容忍度 /m
enemy_exist = 0
enemy_exist_count = 0

nav_start_time = 0
square_size = 1.0

gimbal_yaw_current = 0
gimbal_pitch_current = 0

game_area = 0
game_shoot_number = 0
game_health_point = 0 

#map_border_point = [[0,0],[0,5],[5,5],[5,0]]
map_border_point = []
map_central_point = [0,0]

gimbal_yaw_speed = 0.005
gimbal_pitch_speed = 0.01

gimbal_min_pitch = -0.0
gimbal_max_pitch = 0.4

gimbal_min_yaw = -1.57
gimbal_max_yaw = 1.57

add_map_point_flag = 0


#初始化节点
rospy.init_node('auto_nav_mult',anonymous=False)
cmd_vel_pub = rospy.Publisher('base_vel', Twist, queue_size=5) #实例化一个消息发布函数
Marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=5)
gimbal_ctrl_pub = rospy.Publisher('/gimbal_cmd',GimbalCtrl,queue_size=1)
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) #action服务器连接

#test
simulink_gimbal_yaw_position_cmd_ = rospy.Publisher("/auto_car/yaw_steering_position_controller/command",Float64,queue_size=10)
simulink_gimbal_pitch_position_cmd_ = rospy.Publisher("/auto_car/yaw_steering_position_controller/command",Float64,queue_size=10)

tf_listener = tf.TransformListener()

rate_wait = rospy.Rate(10)

#设置参数 
rospy.loginfo('等待move_base action服务器连接...')
move_base.wait_for_server(rospy.Duration(30))
rospy.loginfo('已连接导航服务')
#yaml_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/goal_mul.yaml"
goal_msg = [0.0,0.0,0.0]
clicked_point_list = []


def callback_clicked_point(data):
    global goal_msg 
    goal_msg[0] = data.point.x
    goal_msg[1] = data.point.y
    print ("[INFO] Goal:[x:%f  , y:%f]" %(goal_msg[0] , goal_msg[1]))
    if add_map_point_flag==0 :
        mark_point(goal_msg[0],goal_msg[1])
        clicked_point_list.append(copy.deepcopy(goal_msg))
    elif add_map_point_flag ==1:
        map_border_point.append(copy.deepcopy(goal_msg))
        mark_point_green(goal_msg[0],goal_msg[1])
    elif add_map_point_flag ==2:
        map_central_point[0] = goal_msg[0]
        map_central_point[1] = goal_msg[1]
        mark_point_blue(goal_msg[0],goal_msg[1])


def callback_enemy_info(msg):
    global enemy_exist
    if(msg.enemy_flag == True):
        enemy_exist =1
    else:
        enemy_exist= 0 

def callback_gimabl_fdb(msg):
    global gimbal_yaw_current
    global gimbal_pitch_current
    gimbal_yaw_current = msg.yaw_fdb
    gimbal_pitch_current = msg.pitch_fdb

def callback_game_state(msg):
    global game_area
    global game_health_point
    global game_shoot_number

    game_area = msg.area
    game_health_point = msg.health_point
    game_shoot_number = msg.shoot_number

def thread_robot_pos():
    rate_tf = rospy.Rate(100)
    while not rospy.is_shutdown():
        get_pos()
        rate_tf.sleep()

def thread_sub():
    rospy.spin()

def thread_gimbal():
    global current_pos
    rate_gimbal = rospy.Rate(100)
    dire_yaw_flag = 0
    dire_pitch_flag = 0
    
    gimbal_pitch_target = gimbal_pitch_current #绝对
    gimbal_yaw_speed_target = 0 #相对
    yaw_temp =  gimbal_yaw_current

    gimbal_ctrl_msg = GimbalCtrl()

    #test
    simulink_gimbal_yaw = Float64()
    simulink_gimbal_pitch = Float64()
    while not rospy.is_shutdown():
        while enemy_exist ==0 and not rospy.is_shutdown():
            
            if gimbal_pitch_target < 0.4 and dire_pitch_flag==0 :
                gimbal_pitch_target += 0.015
                if gimbal_pitch_target >= 0.4:
                    dire_pitch_flag =1

            elif gimbal_pitch_target > 0.0 and dire_pitch_flag==1 :
                gimbal_pitch_target -= 0.015
                if gimbal_pitch_target <= 0.0:
                    dire_pitch_flag=0
                
            #print(gimbal_pitch_target)

            if len(map_border_point)>0:
                ang1,ang2,ang_c =  angle_ans(current_pos,2,1,map_border_point,map_central_point)
                # print (ang1)
                # print (ang2)
                # print (ang_c)
                # print("----------")
                if ang1 ==0 and ang2 ==0 and ang_c ==0:
                    dire_yaw_flag = 0
                    gimbal_yaw_speed_target = gimbal_yaw_speed
                    gimbal_yaw_speed_target = -1.0
                else:
                    yaw_temp =  gimbal_yaw_current
                    if dire_yaw_flag ==0 :
                        if yaw_temp < ang1:
                            yaw_temp +=gimbal_yaw_speed
                            gimbal_yaw_speed_target = gimbal_yaw_speed
                            #yaw_temp +=gimbal_yaw_speed
                            gimbal_yaw_speed_target = 0.4
                            if yaw_temp >= ang1:
                                dire_yaw_flag =1
                        else:
                            dire_yaw_flag =1
                    else:
                        if yaw_temp > ang2:
                            yaw_temp -=gimbal_yaw_speed
                            gimbal_yaw_speed_target = -1*gimbal_yaw_speed
                            #yaw_temp -=gimbal_yaw_speed
                            gimbal_yaw_speed_target = -0.4
                            if yaw_temp <=ang2:
                                dire_yaw_flag=0
                        else:
                            dire_yaw_flag =0

            else:
                dire_yaw_flag = 0
                gimbal_yaw_speed_target = gimbal_yaw_speed
                gimbal_yaw_speed_target = -1.0

            #print(yaw_temp)
            # simulink_gimbal_yaw.data = -yaw_temp
            # simulink_gimbal_pitch.data = gimbal_pitch_target 

            # simulink_gimbal_pitch_position_cmd_.publish(simulink_gimbal_pitch)
            # simulink_gimbal_yaw_position_cmd_.publish(simulink_gimbal_yaw)

            gimbal_ctrl_msg.yaw_cmd = gimbal_yaw_speed_target
            gimbal_ctrl_msg.pitch_cmd = gimbal_pitch_target
            gimbal_ctrl_msg.shoot_cmd = 0
            gimbal_ctrl_msg.friction_cmd = 1
            gimbal_ctrl_pub.publish(gimbal_ctrl_msg)
            rate_gimbal.sleep()
        

def angle_ans(robot_pos,r_360,r_90,map_border_point,map_central_point):
    
    robto2point_angle_list = []
    robot2central_ve = [0,0]
    robot2point_ve = [0,0]
    i_list = []
    dist_list = []

    robot2central_ve[0] = map_central_point[0] - robot_pos[0]
    robot2central_ve[1] = map_central_point[1] - robot_pos[1]

    robot2central_dist = two_point_el_dist(robot_pos,map_central_point)
    if robot2central_dist <r_360:
        return 0,0,0
    else:
        for i in range(len(map_border_point)):
            dist = two_point_el_dist(robot_pos,map_border_point[i])
            dist_list.append(copy.deepcopy(dist))
            if dist > r_90:
                robot2point_ve[0] = map_border_point[i][0] - robot_pos[0]
                robot2point_ve[1] = map_border_point[i][1] - robot_pos[1]
                angle = two_vector_angle(robot2central_ve,robot2point_ve)
                robto2point_angle_list.append(copy.deepcopy(angle))
                i_list.append(copy.deepcopy(i))

        robto2point_angle_list_as = sorted(robto2point_angle_list,reverse=True)
        #print(robto2point_angle_list_as)
        #print(dist_list)
        #max_point_index = robto2point_angle_list.index(robto2point_angle_list_as[0])
        #min_point_index = robto2point_angle_list.index(robto2point_angle_list_as[-1])
        central_angle = math.atan2(robot2central_ve[1],robot2central_ve[0])
        
        max_angle = robto2point_angle_list_as[0] +central_angle
        min_angle = robto2point_angle_list_as[-1]+central_angle
   
        #max_angle = angle_re(max_angle)
        #min_angle = angle_re(min_angle)
        #max_angle = math.atan2((map_border_point[max_point_index][1]-robot_pos[1]),(map_border_point[max_point_index][0]-robot_pos[0]))
        #min_angle = math.atan2((map_border_point[min_point_index][1]-robot_pos[1]),(map_border_point[min_point_index][0]-robot_pos[0]))
        if min_angle > max_angle:
            ang = max_angle
            max_angle = min_angle
            min_angle = ang


        return max_angle,min_angle,central_angle

def two_point_el_dist(point1,point2):
    dist =0
    dist = math.sqrt(pow((point2[0]-point1[0]),2)+pow((point2[1]-point1[1]),2))
    return dist

def two_vector_angle(vector1,vector2):
    ang_v1 = math.atan2(vector1[1],vector1[0])
    ang_v2 = math.atan2(vector2[1],vector2[0])
    angle = ang_v1 - ang_v2
    angle = angle_re(angle)
    return angle

def angle_re (angle):
    if angle > 3.14:
        angle -=6.28
    elif angle <-3.14:
        angle +=6.28
    return angle

def mark_point_blue(x,y):
    global count_marker
    global MARKERS_MAX
    global markerArray
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    if(count_marker > MARKERS_MAX):
       markerArray.markers.pop(0)
    markerArray.markers.append(marker)
     # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1
   # Publish the MarkerArray
    Marker_publisher.publish(markerArray)
    count_marker += 1

def mark_point_green(x,y):
    global count_marker
    global MARKERS_MAX
    global markerArray
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    if(count_marker > MARKERS_MAX):
       markerArray.markers.pop(0)
    markerArray.markers.append(marker)
     # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1
   # Publish the MarkerArray
    Marker_publisher.publish(markerArray)
    count_marker += 1

def mark_point(x,y):
    global count_marker
    global MARKERS_MAX
    global markerArray
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    if(count_marker > MARKERS_MAX):
       markerArray.markers.pop(0)
    markerArray.markers.append(marker)
     # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1
   # Publish the MarkerArray
    Marker_publisher.publish(markerArray)
    count_marker += 1

#添加导航坐标点,输入x（前）坐标，y（左）坐标，th（平面朝向0～360度）
def nav_to(x,y,th):
    mark_point(x,y)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id='map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose_e(x,y,th)
    move_base.send_goal(goal)

def get_state(timeout):
    start_time = rospy.Time.now().to_sec()
    while  not rospy.is_shutdown():
        state = move_base.get_state()
        #print(str(state))
        now_time = rospy.Time.now().to_sec()
        if state == 3:
           return True
        if (now_time-start_time)>timeout:
           return False

#写一个函数 用于任务完成提示。
def move(goal):
    move_base.send_goal(goal)
    if get_state(45):
        rospy.loginfo(place_name+'导航成功！')
    else:
        while not rospy.is_shutdown():
            rospy.loginfo('时间超时，进入恢复状态，重新导航。')
            #move_for(0,-1,5)
            move_base.send_goal(goal)
            if get_state(10):
                break
   
def shutdown():
    rospy.loginfo('机器人任务停止')
    move_base.cancel_goal()
    rospy.sleep(2)
    cmd_vel_pub.publish(Twist)
    rospy.sleep(1)
    
def pose_e(x,y,th):#输入x（前）坐标，y（左）坐标，th（平面朝向0～360度）
    new_pose=Pose()
    new_pose.position.x=float(x)
    new_pose.position.y=float(y)
    #机器朝向，平面朝向弧度转化成四元数空间位姿
    q=quaternion_from_euler(0.0,0.0,float(th)/180.0*pi)
    new_pose.orientation.x=q[0]
    new_pose.orientation.y=q[1]
    new_pose.orientation.z=q[2]
    new_pose.orientation.w=q[3]
    return  new_pose

def yaml_read():
    global goal_dict
    global nav_goals
    f = open(yaml_path) 
    cfg =  f.read()
    goal_dict = yaml.load(cfg)
    nav_goals=goal_dict.keys()

def nav_place(place):
    global place_name
    place_name=place
    print("尝试导航去:"+place_name)
    goal=place
    if goal in nav_goals:
        goal_data=goal_dict[goal]
        nav_to(goal_data[0],goal_data[1],goal_data[2])

def nav_to_place(place):
    global place_name
    place_name=place
    print("尝试导航去:"+place_name)
    goal = place
    if goal in nav_goals:
        goal_data=goal_dict[goal]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id='map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose_e(goal_data[0],goal_data[1],goal_data[2])
        move(goal)

def move_for(xspeed,tspeed,time_second):
    twist_data=Twist()
    twist_data.linear.x=xspeed
    twist_data.angular.z=tspeed
    time_start=time.time()
    while time.time()-time_start<time_second:
        cmd_vel_pub.publish(twist_data)
    cmd_vel_pub.publish(Twist())

#定义更新坐标函数
def get_pos():    
        try:
            (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        #rospy_Time(0)指最近时刻存储的数据
        #得到从 '/map' 到 '/base_link' 的变换，在实际使用时，转换得出的坐标是在 '/base_link' 坐标系下的。
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        global current_x, current_y 
        global current_pos
        current_x = trans[0]
        current_y = trans[1]
        current_pos[0] = trans[0]
        current_pos[1] = trans[1]
        #th = euler[2] / pi * 180

#定义判断是否到达离目标点指定距离函数
def reach_goal(place):
    #get_pos()
    goal=place
    if goal in nav_goals:
        goal_data=goal_dict[goal]
    if((goal_data[0]-current_x)**2+(goal_data[1]-current_y)**2  <= xy_tolerate**2):
        print('reach the %s goal'%place)
        return True
    else:
        return False

def reach_point(x,y,tolerate):
    #get_pos()
    if((x-current_x)**2+(y-current_y)**2  <= tolerate**2):
        return True
    else:
        return False
          
        
def Goto_point(goal_x,goal_y,tolerate,nav_timeout):
    global nav_start_time
    global enemy_exist
    global rate_wait
    global enemy_exist_count
    nav_start_time = rospy.Time.now().to_sec()
    nav_to(goal_x,goal_y,0)
    
    while not reach_point(goal_x,goal_y,tolerate):
        nav_current_time = rospy.Time.now().to_sec()
        if (nav_current_time-nav_start_time)>nav_timeout:
            rospy.loginfo('时间超时，重新导航,下一目标点')
            move_base.cancel_goal()
            #move_for(0,-1,5)
            nav_start_time = rospy.Time.now().to_sec()
            nav_to(goal_x,goal_y,0)
            break
        
        if enemy_exist ==1:
            enemy_exist_count = 30
            move_base.cancel_goal()
            rospy.loginfo('发现敌方，暂停导航')
            while enemy_exist_count >0:
                if enemy_exist ==1:
                    enemy_exist_count =30
                    enemy_exist =0
                else:
                    enemy_exist_count-=1
                rospy.loginfo(enemy_exist_count)
                rate_wait.sleep()
            nav_start_time = rospy.Time.now().to_sec()
            rospy.loginfo('未发现敌方，继续当前导航')
            nav_to(goal_x,goal_y,0)

if __name__ == "__main__":
    
    Mode = 0 
    quit_flag = 1
    mode_flag = 0
    rate_nav = rospy.Rate(100)

    rospy.Subscriber("/clicked_point",PointStamped,callback_clicked_point)

    rospy.Subscriber('/enemy_info',EnemyInfo,callback_enemy_info)
    rospy.Subscriber('/gimbal_fdb',GimbalFdb,callback_gimabl_fdb)
    rospy.Subscriber('/game_state',GameStatus,callback_game_state)

    thread_sub_ = threading.Thread(target = thread_sub)
    thread_sub_.start()

    threa_gimbal_ = threading.Thread(target = thread_gimbal)
    threa_gimbal_.start()

    threa_robot_pos_ = threading.Thread(target = thread_robot_pos)
    threa_robot_pos_.start()
    while not rospy.is_shutdown():
        print ("[1] SET MAP Points \n")
        print ("[2] Multi Points in RVIZ \n")
        print ("[3] Multi Points in YAML \n")
        Mode = int(input("[Enter] Mode Number:\n"))
        while mode_flag ==0:
            if Mode ==1:
                mode_flag =1
            elif Mode ==2:
                mode_flag =1
            elif Mode ==3:
                mode_flag =1
            else:
                print ("[ERROR] Wrong Number\n")
                Mode = int(input("[Enter] Mode Number:\n"))
                mode_flag =0

        while Mode == 1 and not rospy.is_shutdown():
            add_map_point_flag = 1
            print("[Map-Point] Point the Map Border Points")
            fin1 = input("[Enter] 1 To finish \n")
            print("[Map-Point]Total: %d  Points \n"%(len(map_border_point)))
            add_map_point_flag = 2           
            print("[Map-Point] Point the Map Central Point")
            fin1 = input("[Enter] 1 To finish \n")
            print("[Map-Point] Go back to th menu")
            mode_flag = 0
            add_map_point_flag = 0   
            break
            
        while Mode ==2 and not rospy.is_shutdown():
            print("[RVIZ] Waiting Points And Enter [o] To Over\n")
            points_overflag=0
            on_circle = 0 
            on_circle = input("[Enter] [1]Circle [0]No Circle\n")
            dis_tolen = 0.1
            dis_tolen = float(input("[Enter] Tolerate Distance\n"))
            Interval_time = 0
            Interval_time = float(input("[Enter] Interval Time\n"))
            over_flag = 0
            while points_overflag == 0:
                if len(clicked_point_list)>=3:
                        print("[Point]Total: %d  Points \n"%(len(clicked_point_list)))
                        points_overflag = 1
                else:
                        print("[Point]Points less than 2")
            while over_flag == 0:
                for i in range(len(clicked_point_list)):
                    Goto_point(clicked_point_list[i][0],clicked_point_list[i][1],dis_tolen,15)
                    rospy.sleep(Interval_time)
                    print ("[INFO]Reach point:")
                    print (str(i)+'\n')

                if on_circle==0:
                    over_flag=1
                    clicked_point_list =[]
                    break

        while Mode ==3 and not rospy.is_shutdown():
            nav_to_place("B1")
            nav_to_place("B2")
            nav_to_place("B3")
            break
    
    print("OVER")




            
        
        


    
