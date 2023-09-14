#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
from geometry_msgs.msg import Pose, Point, Quaternion, Twist #导入四个消息数据类型，姿态，目标点，四元数，运动消息Twist
import threading
from roborts_msgs.msg import EnemyInfo,GameStatus,EnemyLocate

game_area = 0
tower_enemy_locate_x = 0
tower_enemy_locate_y = 0
tower_self_locate_x = 0
tower_self_locate_y = 0

enemy_exist = 0

tower_self_locate_flag = 0
tower_enemy_locate_flag = 0

tf_flag = 0
time_count = 0 


def callback_enemy_locate(msg):
    global tower_enemy_locate_x
    global tower_enemy_locate_y
    global tower_self_locate_x
    global tower_self_locate_y
    global tower_self_locate_flag
    global tower_enemy_locate_flag
    global time_count 
   
    
    if game_area ==0: #attack blue
            
        tower_enemy_locate_x = msg.blue_x
        tower_enemy_locate_y = msg.blue_y
        tower_enemy_locate_flag = 1
        time_count +=1
    
        tower_self_locate_x = msg.red_x
        tower_self_locate_y = msg.red_y
        tower_self_locate_flag = 1 
                
     
    else:                    #attack red
             
        tower_self_locate_x = msg.blue_x
        tower_self_locate_y = msg.blue_y
        tower_self_locate_flag = 1 
        

        tower_enemy_locate_x = msg.red_x
        tower_enemy_locate_y = msg.red_y
        tower_enemy_locate_flag = 1
        time_count +=1
            
    

def callback_enemy_info(msg):
    global enemy_exist
    if(msg.enemy_flag == True):
            enemy_exist =1
    else:
            enemy_exist= 0 

def callback_game_state(msg):
    global game_area
    game_area = msg.area
    


def spin_th():
    while not rospy.is_shutdown():
        rospy.spin()

def time_th():
    global time_count
    global tf_flag
    rate_time = rospy.Rate(1)
    count = 0
    while not rospy.is_shutdown():
        print(time_count)
        if time_count >0:
            tf_flag =1
        else :
            tf_flag =0
        time_count = 0
        rate_time.sleep()

def tf_th():
    rate_tf = rospy.Rate(20)
    while not rospy.is_shutdown():
        if tf_flag:
            br = tf.TransformBroadcaster()
            br.sendTransform((-tower_enemy_locate_x, tower_enemy_locate_y, 0.0),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"map","enemy2_link")
        rate_tf.sleep()


if __name__ == '__main__':
    rospy.init_node('tower_tf')
    rospy.Subscriber('/enemy_info',EnemyInfo,callback_enemy_info)
    rospy.Subscriber('/tower_locate',EnemyLocate,callback_enemy_locate)
    rospy.Subscriber('/game_state',GameStatus,callback_game_state)
    
    #thread_spin = threading.Thread(target = spin_th)
    #thread_spin.start()
    rate_tf = rospy.Rate(20)

    thread_time = threading.Thread(target = time_th)
    thread_time.start()

    thread_tf = threading.Thread(target = tf_th)
    thread_tf.start()
    
    rospy.spin()
    

    
