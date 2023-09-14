#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from re import T
from sys import flags
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
import random

from actionlib_msgs.msg import * #导入actionlib的所有模块
from geometry_msgs.msg import Pose, Point, Quaternion, Twist #导入四个消息数据类型，姿态，目标点，四元数，运动消息Twist
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #导入movebase的两个消息数据类型
from tf.transformations import quaternion_from_euler #导入tf变换库的欧拉角转四元数库
from std_msgs.msg import Int8 
from std_msgs.msg import String #导入标准消息的字符串消息数据格式
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from roborts_msgs.msg import EnemyInfo,GimbalFdb ,GimbalCtrl,GameStatus,EnemyLocate
from nav_msgs.msg import Path 
import optimal_point as op


class AUTO_FSM:
    def __init__(self):
        #变量
        self.gimbal_ctrl_min_pitch = float(rospy.get_param('/pitch_min',0.0))
        self.gimbal_ctrl_max_pitch = float(rospy.get_param('/pitch_max',0.4))
        self.gimbal_pitch_speed = float(rospy.get_param('/pitch_speed',0.015))
        self.gimbal_yaw_speed = float(rospy.get_param('/yaw_speed',0.3))
        self.gimbal_ctrl_yaw_360_is = bool(rospy.get_param('/yaw_is_360',True))
        self.gimbal_ctrl_max_yaw = float(rospy.get_param('/yaw_max',-1.57))
        self.gimbal_ctrl_min_yaw = float(rospy.get_param('/yaw_min',1.57))

        self.set_health_point_max = int(rospy.get_param('/set_health_max',200))
        self.set_shoot_number_max = int(rospy.get_param('/set_shoot_number_max',300))
        self.set_unhealth_point_percent = float(rospy.get_param('/set_unhealth_percent',0.3))

        self.yaml_name = str(rospy.get_param('/goal_yaml_filename','/goal_simulink.yaml')) 
        self.yaml_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + self.yaml_name
        self.goal_dict = dict()
        self.goal_path = []
        self.nav_goal_name = []
        self.random_list = []
        
        self.robot_position_x = 0
        self.robot_position_y = 0 
        
        self.game_area = 0
        self.game_shoot_number = 0
        self.game_health_point = self.set_health_point_max
        self.game_health_percent = self.game_health_point/self.set_health_point_max
        self.game_shoot_rest_number = self.set_shoot_number_max
        self.game_heat_rest = 280
        self.game_heat_low = 80 
        self.game_heat_low_flag = 0

        self.game_state = 0 # 0
        
        self.enemy_exist = 0
        self.enemy_locate_exist = 0
        self.enemy_position_x = 0
        self.enemy_position_y = 0

        self.gimbal_yaw_current = 0
        self.gimbal_pitch_current = 0

        self.count_marker = 0
        self.MARKERS_MAX = 200
        self.markerArray = MarkerArray()

        self.optimal_point = op.Optimal()
        self.robot2attack_dis = 0
        self.robot2defend_dis = 0
        self.robot2attack_index = 0 

        self.attack_point = []
        self.defend_point = []
        self.near_point = []

        self.tole_dis = 0.3
        self.nav_timeout = 5
        self.interval_time = 1.0

        self.rate_nav = rospy.Rate(100)

        self.nomarl_rate = rospy.Rate(20)
        self.global_plan_state = 0  

        self.tf_time = 0.5

        self.tower_enemy_locate_x = 0
        self.tower_enemy_locate_y = 0
        self.tower_self_locate_x = 0
        self.tower_self_locate_y = 0

        self.tower_enemy_locate_flag = 0

        self.imu_yaw = 0

        #FSM 
        '''
        0 休眠状态 1 巡逻状态 2追击状态 3攻击状态 4逃跑状态 【5】重定位状态
        '''
        self.FSM_state = 0

        #FSM

        self.cmd_topic = rospy.get_param('/cmd_topic','base_vel') #控制话题名字

        self.base_vel_pub = rospy.Publisher(self.cmd_topic,Twist,queue_size=1)
        self.gimbal_ctrl_pub = rospy.Publisher('/gimbal_cmd',GimbalCtrl,queue_size=1)
        self.move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=5)
        
        self.enemy_info_sub = rospy.Subscriber('/enemy_info',EnemyInfo,self.callback_enemy_info)
        #self.gimbal_fdb_sub = rospy.Subscriber('/gimbal_fdb',GimbalFdb,self.callback_gimabl_fdb)
        self.game_state_sub = rospy.Subscriber('/game_state',GameStatus,self.callback_game_state)
        self.global_plan_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan',Path,self.callback_global_plan)
        #self.enemy_locate_sub = rospy.Subscriber('/tower_locate',EnemyLocate,self.callback_enemy_locate)

        self.clicked_point_sub = rospy.Subscriber("/clicked_point",PointStamped,self.callback_clicked_point)

        self.robot_pos_tf_listener = tf.TransformListener()
        self.enemy_pos_tf_listerer = tf.TransformListener()

        self.yaml_read()

        rospy.loginfo('等待move_base action服务器连接...')
        self.move_base.wait_for_server(rospy.Duration(10))
        rospy.loginfo('已连接导航服务')

        self.thread_tf_ = threading.Thread(target = self.thread_tf)
        self.thread_tf_.start()

        self.thread_gimbal_ = threading.Thread(target = self.thread_gimbal)
        self.thread_gimbal_.start()

        self.thread_FSM_ = threading.Thread(target= self.thread_FSM)
        self.thread_FSM_.start()

        self.thread_FSM_state_ = threading.Thread(target= self.thread_FSM_state)
        self.thread_FSM_state_.start()


    def callback_enemy_locate(self,msg):
        if self.game_area ==0: #attack blue
            if (rospy.Time.now().to_sec() - msg.blue_time.data.to_sec())<3: 
                self.tower_enemy_locate_x = -msg.blue_x
                self.tower_enemy_locate_y = msg.blue_y
                self.tower_enemy_locate_flag = 1
            else :
                self.tower_enemy_locate_flag = 0
            if (rospy.Time.now().to_sec() - msg.red_time.data.to_sec())<3: 
                self.tower_self_locate_x = -msg.red_x
                self.tower_self_locate_y = msg.red_y
        else:                    #attack red
            if (rospy.Time.now().to_sec() - msg.blue_time.data.to_sec())<3: 
                self.tower_self_locate_x = -msg.blue_x
                self.tower_self_locate_y = msg.blue_y
                self.tower_enemy_locate_flag = 1
            else :
                self.tower_enemy_locate_flag = 0

            if (rospy.Time.now().to_sec() - msg.red_time.data.to_sec())<3: 
                self.tower_enemy_locate_x = -msg.red_x
                self.tower_enemy_locate_y = msg.red_y


    def callback_enemy_info(self,msg):
        if(msg.enemy_flag == True):
            self.enemy_exist =1
        else:
            self.enemy_exist= 0 

    # def callback_gimabl_fdb(self,msg):
    #     #self.gimbal_yaw_current = msg.yaw_fdb 
    #     #self.gimbal_pitch_current = msg.pitch_fdb

    def callback_game_state(self,msg):
        self.game_area = msg.area
        self.game_health_point = msg.health_point
        self.game_shoot_number = msg.shoot_number
        self.game_health_percent = msg.health_point / self.set_health_point_max
        self.game_shoot_rest_number =  self.set_shoot_number_max - msg.shoot_number
        self.game_state = msg.game_state
        self.game_heat_rest = msg.heat_rest
        if self.game_heat_rest < self.game_heat_low:
            self.game_heat_low_flag = 1

    def callback_clicked_point(self,msg):
        self.mark_point(msg.point.x,msg.point.y,1,0,0)

    def callback_global_plan(self,msg):
        if len(msg.poses)!=0:
            #print("get a plan")
            self.global_plan_state = 1
        else :
            #print("fall to get a plan")
            self.global_plan_state = 0    

    def thread_tf(self):
        rate_tf = rospy.Rate(100)
        self.enemy_locate_exist_count = 0
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.robot_pos_tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    #rospy_Time(0)指最近时刻存储的数据
                            #得到从 '/map' 到 '/base_link' 的变换，在实际使用时，转换得出的坐标是在 '/base_link' 坐标系下的。
                self.robot_position_x = trans[0]
                self.robot_position_y = trans[1]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("baselink tf Error")
            #self.enemy_pos_tf_listerer.waitForTransform('/map','/enemy_link',rospy.Time(),rospy.Duration(0.01))
            if self.enemy_exist==1:
                try:
                    #now = rospy.Time.now()
                    #self.enemy_pos_tf_listerer.waitForTransform('/map','/enemy_link',now,rospy.Duration(0.5))
                    (trans2, rot2) =self.enemy_pos_tf_listerer.lookupTransform('/map','/enemy_link',rospy.Time.now()-rospy.Duration(self.tf_time)) 

                    #self.enemy_locate_exist_count=0
                    if trans2[0]<5  and trans2[0] > -1 and trans2[1]< 0 and trans2[1]>-5 :
                        self.enemy_locate_exist = 1
                        self.enemy_position_x = trans2[0]
                        self.enemy_position_y = trans2[1]
                        self.enemy2robot_dis_update()
                    #print('enemy tf get')
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #rospy.loginfo("enemylink tf Error")
                    #self.enemy_locate_exist_count+=1
                    self.enemy_locate_exist = 0
                    
                    # if self.enemy_locate_exist_count >100:
                    #     self.enemy_locate_exist_count=0
                    #     self.enemy_locate_exist = 0
            else :
                try:
                   
                    (trans3, rot3) =self.enemy_pos_tf_listerer.lookupTransform('/map','/enemy2_link',rospy.Time.now()-rospy.Duration(self.tf_time))
                    if trans3[0]<5  and trans3[0] > -1 and trans3[1]< 0 and trans3[1]>-5 :
                        self.enemy_locate_exist = 1
                        self.tower_enemy_locate_flag =1 
                        self.enemy_position_x = trans3[0]
                        self.enemy_position_y = trans3[1]
                        self.enemy2robot_dis_update()
                    #print('enemy tf get')
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #rospy.loginfo("enemylink2 tf Error")
                    #self.enemy_locate_exist_count+=1
                    self.enemy_locate_exist = 0
                    self.tower_enemy_locate_flag =0 
                    
                    # if self.enemy_locate_exist_count >100:
                    #     self.enemy_locate_exist_count=0
                    #     self.enemy_locate_exist = 0
            try:
                (trans4, rot4) =self.enemy_pos_tf_listerer.lookupTransform('/map','/yaw_steering_link',rospy.Time(0))
                tf_rot = [0,0,0]
                tf_rot = tf.transformations.euler_from_quaternion(rot4)
                self.gimbal_yaw_current = tf_rot[2] #云台方向，用于判断枪口与目标方向是否一致，是否开枪
                    #print('enemy tf get')
                    
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #rospy.loginfo("enemylink tf Error")
                    #self.enemy_locate_exist_count+=1
                pass
                    
                    # if self.enemy_locate_exist_count >100:
                    #     self.enemy_locate_exist_count=0
                    #     self.enemy_locate_exist = 0
            


            rate_tf.sleep()
        

    def thread_gimbal(self):
        rate_gimbal = rospy.Rate(100)
        dire_yaw_flag = 0
        dire_pitch_flag = 0

        gimbal_pitch_target = self.gimbal_pitch_current #绝对
        gimbal_yaw_speed_target = 0 #相对
        #yaw_temp =  self.gimbal_yaw_current

        gimbal_ctrl_msg = GimbalCtrl()

        while not rospy.is_shutdown():

            if self.FSM_state==0:
                gimbal_ctrl_msg.yaw_cmd = 0
                gimbal_ctrl_msg.pitch_cmd = 0
                gimbal_ctrl_msg.shoot_cmd = 0
                gimbal_ctrl_msg.friction_cmd = 0
                self.gimbal_ctrl_pub.publish(gimbal_ctrl_msg)
                rate_gimbal.sleep()
            #print(self.FSM_state)
            while self.FSM_state !=0 and self.enemy_exist ==0 and self.enemy_locate_exist==0 and not rospy.is_shutdown():  #摆头模式搜索目标
                
                if gimbal_pitch_target < self.gimbal_ctrl_max_pitch and dire_pitch_flag==0 :
                    gimbal_pitch_target += self.gimbal_pitch_speed
                    if gimbal_pitch_target >= self.gimbal_ctrl_max_pitch:
                        dire_pitch_flag =1

                elif gimbal_pitch_target > self.gimbal_ctrl_min_pitch and dire_pitch_flag==1 :
                    gimbal_pitch_target -= self.gimbal_pitch_speed
                    if gimbal_pitch_target <= self.gimbal_ctrl_min_pitch:
                        dire_pitch_flag=0
                    
                if self.gimbal_ctrl_yaw_360_is == True:
                    dire_yaw_flag = 0
                    gimbal_yaw_speed_target = -self.gimbal_yaw_speed

                gimbal_ctrl_msg.yaw_cmd = gimbal_yaw_speed_target
                gimbal_ctrl_msg.pitch_cmd = gimbal_pitch_target
                gimbal_ctrl_msg.shoot_cmd = 0
                gimbal_ctrl_msg.friction_cmd = 1
                self.gimbal_ctrl_pub.publish(gimbal_ctrl_msg)
                #print("摆头模式")
                rate_gimbal.sleep()
            
            #print(self.tower_enemy_locate_flag)

            while self.FSM_state !=0 and self.enemy_exist ==0 and self.tower_enemy_locate_flag ==1 and not rospy.is_shutdown():  #已获取坐标，但未看到目标时
                #print("锁定目标")
                #print(self.tower_enemy_locate_flag)
                target_yaw = math.atan2(self.enemy_position_y-self.robot_position_y,self.enemy_position_x-self.robot_position_x)
                #print("target:"+str(target_yaw))
                #print("gimbal_yaw:"+str(self.gimbal_yaw_current))
                if math.fabs(target_yaw-self.gimbal_yaw_current) >0.005 and math.fabs(target_yaw-self.gimbal_yaw_current) <0.1:
                    gimbal_yaw_speed_target =  0.25* (target_yaw - self.gimbal_yaw_current) 
                elif math.fabs(target_yaw-self.gimbal_yaw_current) >0.1:
                    gimbal_yaw_speed_target =  0.5 * (target_yaw - self.gimbal_yaw_current) 
                    
                else:
                    gimbal_yaw_speed_target = 0
                
                gimbal_pitch_target = 0
                #print("------------------")
                #print(gimbal_yaw_speed_target)
                gimbal_ctrl_msg.yaw_cmd = gimbal_yaw_speed_target
                gimbal_ctrl_msg.pitch_cmd = gimbal_pitch_target
                gimbal_ctrl_msg.shoot_cmd = 0
                gimbal_ctrl_msg.friction_cmd = 1
                self.gimbal_ctrl_pub.publish(gimbal_ctrl_msg)
                rate_gimbal.sleep()
            
        
    
    def thread_FSM_state(self):
        while not rospy.is_shutdown():
            self.state_update()
            self.rate_nav.sleep()


    def thread_FSM(self):
        rospy.loginfo('准备完成，等待比赛开始')
        while not rospy.is_shutdown():
            if self.FSM_state==1:   #巡逻模式
                #self.mission_patrol()
                self.mission_random()
                #pass
            elif self.FSM_state==2:     #追击模式
                self.mission_pursue()
                #pass
            elif self.FSM_state==3:     #进攻模式
                self.mission_attack()
                #pass
            elif self.FSM_state==4:     #逃跑磨损
                self.mission_escape()
            elif self.FSM_state==5:     #重定位模式
                self.mission_relocate()
                

    def mission_relocate(self):
        pass


    def mission_random(self):
        '''随机点巡逻 随机权重由random_list决定'''
        rospy.loginfo("开始随机跑点")
        while self.FSM_state==1 and not rospy.is_shutdown():
            random_num = random.randint(0,len(self.random_list)-1)  #生成随机数
            reach_result = self.goto_point(self.goal_dict[self.random_list[random_num]],self.tole_dis,self.nav_timeout)
            if reach_result == 1 :
                rospy.loginfo("到达目标点"+str(self.random_list[random_num]))
                reach_result =0
            rospy.sleep(self.interval_time)
        rospy.loginfo("退出随机跑点")

    def mission_patrol(self):
        '''按顺序巡逻 从当前点开始'''
        robot_pos = [0,0]
        robot_pos[0] = self.robot_position_x
        robot_pos[1] = self.robot_position_y
        nearest_point_name = self.find_nearest_point(robot_pos,self.goal_dict,self.goal_path)
        point_index = self.goal_path.index(nearest_point_name)
        point_num = len(self.goal_path) 
        rospy.loginfo("开始巡逻")
        while self.FSM_state==1 and not rospy.is_shutdown():
            reach_result = self.goto_point(self.goal_dict[self.goal_path[point_index]],self.tole_dis,self.nav_timeout)
            if reach_result == 1 :
                rospy.loginfo("到达目标点"+str(self.goal_path[point_index]))
                reach_result =0
            if point_index < point_num:
                point_index +=1
            if point_index >= point_num:
                point_index = 0 
            rospy.sleep(self.interval_time)
        rospy.loginfo("退出巡逻")

    def mission_attack(self):
        rospy.loginfo("开始攻击")
        while self.FSM_state==3 and not rospy.is_shutdown():
            # self.tf_time = 8
            # random_time = random.random()*self.tf_time
            # rospy.sleep(random_time)
            # self.robot2attack_index = random.randint(0,2)


            self.rate_nav.sleep()
                
        rospy.loginfo("退出攻击")
    
    def mission_pursue(self):
        rospy.loginfo("开始追击")
        while self.FSM_state==2 and not rospy.is_shutdown():
            if self.robot2attack_dis >self.tole_dis and self.enemy_locate_exist ==1:
                self.goto_point(self.attack_point[self.robot2attack_index],self.tole_dis,self.nav_timeout)
                self.mark_point(self.attack_point[self.robot2attack_index ][0],self.attack_point[self.robot2attack_index ][1],0.0,1.0,0.0)
            else:
                rospy.loginfo("到达攻击点")
            self.rate_nav.sleep()

        if self.enemy_locate_exist ==0 :
                rospy.loginfo("丢失目标坐标,前往最后出现点")
                self.goto_point(self.near_point[0],self.tole_dis,self.nav_timeout)
                self.mark_point(self.near_point[0][0],self.near_point[0][1],1.0,0.0,0.0)
                wait_time_start = rospy.Time.now().to_sec()
                wait_current_time = wait_time_start
                while (self.enemy_locate_exist ==0 or self.enemy_exist==0) and (wait_current_time - wait_time_start)<5:
                    wait_current_time = rospy.Time.now().to_sec()
                    #print(wait_current_time)
                    self.rate_nav.sleep()

        rospy.loginfo("退出追击")

    def mission_escape(self):
        rospy.loginfo("开始逃跑")
        while self.FSM_state==4 and not rospy.is_shutdown():

            #if self.game_heat_rest > 250:
            #    self.game_heat_low_flag =0
            #    self.FSM_state=2


            if self.robot2defend_dis > self.tole_dis:
                self.goto_point(self.defend_point[0],self.tole_dis,self.nav_timeout)
            else:
                rospy.loginfo("到达防御点")
            self.rate_nav.sleep()
        rospy.loginfo("退出逃跑")
        
    def pose_e(self,x,y,th):#输入x（前）坐标，y（左）坐标，th（平面朝向0～360度）
        '''x,y,th转换为Pose()'''
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

    def nav_to(self,x,y,th):
        plan_count = 0
        plan_flag = 0
        goal_x = x 
        goal_y = y 
        while plan_count <20 and plan_flag ==0:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id='map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.pose_e(goal_x,goal_y,th)
            self.move_base.send_goal(goal)
            rospy.sleep(0.05)
            if self.global_plan_state == 1:
                plan_flag = 1
                # if plan_count >0:
                #     print('找到新目标点')
                # else:
                #     print("get path")
            else:   #无法到达，正在搜索最近点
                goal_x, goal_y = self.random_point(x,y,self.tole_dis)
                #print('x:'+str(goal_x)+'y'+str(goal_y))
                #print("无法到达，正在搜索最近点")
                self.move_base.cancel_goal()
                plan_flag = 0
                plan_count+=1
        if plan_count < 20:
            return 1
        else:
            return 0 

    def random_point(self,x,y,raid):
        '''在x,y附近寻找随机点,范围是raid'''
          random_raid  =   random.random()*raid
          random_angle = random.random()*6.28
          random_x = x + math.cos(random_angle)*random_raid
          random_y = y + math.sin(random_angle)*random_raid
          return random_x,random_y
    

    def reach_point(self,x,y,tole):
        '''
        功能：  判断机器人是否到达    
        参数:   tole : 为距离容忍度  
                (x,y) : 为目标点坐标
        '''
        if((x-self.robot_position_x)**2+(y-self.robot_position_y)**2  <= tole**2):
            return True
        else:
            return False

    def goto_point(self,goal_point,tole_dis,nav_timeout):
        '''
        功能：  导航到目标点
        参数:   goal_point :  [x,y] 一维列表，为目标点的坐标
                tole_dis : 距离容忍度
                nav_timeout : 时间容忍度
        '''
        self.nav_start_time = rospy.Time.now().to_sec()
        plan_result = self.nav_to(goal_point[0],goal_point[1],0)
        self.last_FSM_state = self.FSM_state
        while not self.reach_point(goal_point[0],goal_point[1],tole_dis) and not rospy.is_shutdown():
            nav_current_time = rospy.Time.now().to_sec()
            if (nav_current_time-self.nav_start_time)>nav_timeout:
                rospy.loginfo('导航超时')
                self.move_base.cancel_goal()
                return 0 
            if plan_result ==0:
                rospy.loginfo('导航无法到达')
                self.move_base.cancel_goal()
                return 0 
            if self.last_FSM_state != self.FSM_state:
                self.move_base.cancel_goal()
                rospy.loginfo('状态更换，导航取消')
                return 0 
            self.rate_nav.sleep()
        if self.reach_point(goal_point[0],goal_point[1],tole_dis):
            return 1
        else:
            return 0
        
    def find_nearest_point(self,robot_pos,point_dict,point_list_name):
        '''寻找与robot_pos最近的点'''
        dist_min = self.two_point_el_dist(robot_pos,point_dict[point_list_name[0]])
        min_point_index = point_list_name[0]
        for i in range (len(point_dict)-1):
            dist = self.two_point_el_dist(robot_pos,point_dict[point_list_name[i]])
            if dist < dist_min:
                min_point_index = point_list_name[i]
        return min_point_index

    def yaml_read(self):
        '''读取yaml文件(参数文件)'''
        f = open(self.yaml_path) 
        cfg =  f.read()
        self.goal_dict = yaml.load(cfg)
        self.nav_goal_name= list(self.goal_dict.keys())
        self.goal_path = self.goal_dict['path']
        self.random_list = self.goal_dict['random_list']
        for i in range (len(self.nav_goal_name)-2):
            goal_data = self.goal_dict[self.nav_goal_name[i]]
            print(goal_data)
            self.mark_point(goal_data[0],goal_data[1],0.0,0.0,1.0)
            self.nomarl_rate.sleep()

    def mark_point(self,x,y,r,g,b):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        if(self.count_marker > self.MARKERS_MAX):
            self.markerArray.markers.pop(0)
        
        self.markerArray.markers.append(marker)
        # Renumber the marker IDs
        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1
        # Publish the MarkerArray
        self.marker_publisher.publish(self.markerArray)
        self.count_marker += 1
        #print('marker')

    def get_state(self,timeout):
        start_time = rospy.Time.now().to_sec()
        while  not rospy.is_shutdown():
            state = self.move_base.get_state()
            print('movebase:'+str(state))
            now_time = rospy.Time.now().to_sec()
            if state == 3:
                return True
            if (now_time-start_time)>timeout:
                return False

    def two_point_el_dist(self,point1,point2):
        dist =0
        dist = math.sqrt(pow((point2[0]-point1[0]),2)+pow((point2[1]-point1[1]),2))
        return dist


    def enemy2robot_dis_update(self):
        self_point = [0,0]
        self_point[0] = self.robot_position_x
        self_point[1] = self.robot_position_y
        enemy_point = [0,0]
        enemy_point[0] = self.enemy_position_x
        enemy_point[1] = self.enemy_position_y
        self.attack_point , self.defend_point, self.near_point = self.optimal_point.calculation_point(self_point,enemy_point)
        self.robot2attack_dis = self.two_point_el_dist(self_point,self.attack_point[self.robot2attack_index])
        self.robot2defend_dis = self.two_point_el_dist(self_point,self.defend_point[0])
        self.robot2fllow_dis = self.two_point_el_dist(self_point,self.near_point[0])
        #print(self.robot2attack_dis)


    def state_update(self):
        #print('enemy_locate'+str(self.enemy_locate_exist))
        #print('enemy_exist'+str(self.enemy_exist))
        #print(self.FSM_state)
        # self.get_state(10)

        # 低血量 or 剩余子弹 or 超热量  ==> 防御模式
        if self.game_health_percent < self.set_unhealth_point_percent or self.game_shoot_rest_number <= 5 or self.game_heat_low_flag == 1:
            self.FSM_state = 4

        # 非防御 and 哨岗发现敌人 and 距离远 and 比赛开始 ==> 追击模式
        elif self.FSM_state!=4 and self.enemy_locate_exist ==1 and self.robot2attack_dis > self.tole_dis and self.game_state==4 :
            self.FSM_state = 2

        # 非防御 and 哨岗丢失敌人 and 未识别到敌人 and 比赛开始 ==> 巡逻模式
        elif self.FSM_state!=4 and self.enemy_locate_exist ==0 and self.enemy_exist ==0 and self.game_state==4 :
            self.FSM_state = 1

        # 非防御 and 识别到敌人 and 距离近 and 比赛开始 ==> 进攻模式
        elif self.FSM_state!=4 and self.enemy_exist == 1 and self.robot2attack_dis <= self.tole_dis and self.game_state==4 and self.game_heat_low_flag ==0 :
            #print(self.robot2attack_dis)
            self.FSM_state = 3
        
        # 比赛未开始/已结束 ==> 休眠
        elif self.game_state !=4:
            self.FSM_state =0
        
        



if __name__ == '__main__':

    rospy.init_node('auto_fsm_node',anonymous=True)
    auto_fsm = AUTO_FSM()
    rospy.spin()
    
