import copy
import math
import random
import time
import matplotlib.pyplot as plt
import numpy as np
import os,inspect
import yaml

"""
在n个点中计算最优攻击和防御点
输入参数：己方机器人坐标，敌方机器人坐标
输出结果：最优攻击点，最优防御点

注：要注意修改坐标点路径
"""
yaml_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/optimal_point.yaml"

class Optimal:
    def __init__(self):
        self.read_yaml()
        pass

    #计算最优点
    def calculation_point(self,self_point,enemy_point):
        for i in range(len(self.goal_list)):
            #计算点到自身的移动代价distance_self和障碍代价obstacle_self
            distance_self = self.distance_of_two_point(self_point,self.goal_list[i])
            obstacle_self = self.obstacle_of_two_point(self_point,self.goal_list[i])
            #print("distance_self = {} , obstacle_self = {}".format(distance_self,obstacle_self))
            #计算点到敌方的移动代价distance_self和障碍代价obstacle_self
            distance_enemy = self.distance_of_two_point(enemy_point,self.goal_list[i])
            obstacle_enemy = self.obstacle_of_two_point(enemy_point,self.goal_list[i])
            #print("distance_enemy = {} , obstacle_enemy = {}".format(distance_enemy,obstacle_enemy))

            self.attack_costlist[i] = self.k_attach_self*distance_self + self.k_attach_enemy*distance_enemy
            self.defend_costlist[i] = self.k_defend_self*distance_self - self.k_defend_enemy*distance_enemy
            self.obstacle_num_list[i] = obstacle_enemy
            self.pursuit_costlist[i] = self.k_defend_enemy*distance_enemy

        #寻找最优攻击点
        min_obs = np.min(self.obstacle_num_list)
        attach_point = []
        attach_cost = []

        for i in range(len(self.goal_list)):
            if self.obstacle_num_list[i] == min_obs:
                if len(attach_point) == 0:
                    attach_point.append(self.goal_list[i])
                    attach_cost.append(self.attack_costlist[i])
                else:
                    for j in range(len(attach_point)):
                        if self.attack_costlist[i]<=attach_cost[j]:
                            attach_point.insert(j,self.goal_list[i])
                            attach_cost.insert(j,self.attack_costlist[i])
                            break

                
        #寻找最优防御点
        max_obs = np.max(self.obstacle_num_list)
        #max_index = [i for i, x in enumerate(self.obstacle_num_list) if x == max_obs]
        defend_point = []
        defend_cost = []
        #min_defend_cost = 1000
        for i in range(len(self.goal_list)):
            if self.obstacle_num_list[i] == max_obs:
                if len(defend_point) == 0:
                    defend_point.append(self.goal_list[i])
                    defend_cost.append(self.defend_costlist[i])
                else:
                    for j in range(len(defend_point)):
                        if self.defend_costlist[i]>=defend_cost[j]:
                            defend_point.insert(j,self.goal_list[i])
                            defend_cost.insert(j,self.defend_costlist[i])
                            break
            # print("point:",self.goal_list[i])
            # print("defend_cost:",defend_cost)
            # print("defenf_point",defend_point)

        pursuit_point = []
        pursuit_cost = []
        for i in range(len(self.goal_list)):
            if self.obstacle_num_list[i] == min_obs:
                if len(pursuit_point) == 0:
                    pursuit_point.append(self.goal_list[i])
                    pursuit_cost.append(self.pursuit_costlist[i])
                else:
                    for j in range(len(pursuit_point)):
                        if self.pursuit_costlist[i]<=pursuit_cost[j]:
                            pursuit_point.insert(j,self.goal_list[i])
                            pursuit_cost.insert(j,self.pursuit_costlist[i])
                            break
            # print("point:",self.goal_list[i])
            # print("pursuit_cost:",pursuit_cost)
            # print("pursuit_point",pursuit_point)

        #寻找最近点


        return attach_point,defend_point,pursuit_point
    
        

    #从yaml文件读取需要遍历的n个点
    def read_yaml(self):
        f = open(yaml_path, 'r', encoding='utf-8') 
        cfg =  f.read()
        goal_dict = yaml.safe_load(cfg)
        nav_goals=goal_dict.keys()
        print("目标点：",goal_dict["obstruct"])
        print("障碍点：",goal_dict["target"])
        #定义障碍点和目标点
        self.obstacle_List=goal_dict["obstruct"]
        self.goal_list=goal_dict["target"]
        #定义四个权重
        self.k_attach_self = goal_dict["k_attach_self"]
        self.k_attach_enemy = goal_dict["k_attach_enemy"]
        self.k_defend_self =  goal_dict["k_defend_self"]
        self.k_defend_enemy = goal_dict["k_defend_enemy"]
        print("k_attach_self: ",self.k_attach_self)
        print("k_attach_enemy: ",self.k_attach_enemy)
        print("k_defend_self: ",self.k_defend_self)
        print("k_defend_enemy: ",self.k_defend_enemy)
        #初始化代价值列表
        self.attack_costlist = [0 for i in range(len(self.goal_list))]
        self.defend_costlist =  [0 for i in range(len(self.goal_list))]
        self.obstacle_num_list = [0 for i in range(len(self.goal_list))]
        self.pursuit_costlist = [0 for i in range(len(self.goal_list))]

    #计算两点间距离
    def distance_of_two_point(self,start,goal):
        return math.sqrt((start[0]-goal[0])**2+(start[1]-goal[1])**2)

    #计算两点间障碍物个数
    def obstacle_of_two_point(self,start,goal):
        obstacle_num=0
        for (ox, oy, size) in self.obstacle_List:
            dd = self.distance_squared_point_to_segment(
                np.array([start[0], start[1]]),
                np.array([goal[0], goal[1]]),
                np.array([ox, oy]))
            if dd <= size ** 2:
                obstacle_num+=1
                return obstacle_num
        return obstacle_num

    #计算两点连线到障碍物中点距离
    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        #若两点重合，结果为其中一点到障碍物距离
        if np.array_equal(v, w):
            return (p - v).dot(p - v)  # v == w case
        l2 = (w - v).dot(w - v)  # i.e. |w-v|^2 -  avoid a sqrt
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)  # Projection falls on the segment
        return (p - projection).dot(p - projection)

    #画图
    def draw_graph(self, self_point=None, enemy_point=None,attach_point=None,defend_point=None,pursuit_point=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        #画自身和敌人位置
        if self_point is not None:
            plt.plot(self_point[0],self_point[1],"8r",ms=10)
        if enemy_point is not None:
            plt.plot(enemy_point[0],enemy_point[1],"8b",ms=10)      

        #画障碍
        for (ox, oy, size) in self.obstacle_List:
            # self.plot_circle(ox, oy, size)
            plt.plot(ox, oy, "ok", ms=10 * size)


        #画最优点
        if attach_point is not None:
            plt.plot(attach_point[0][0],attach_point[0][1],"Dm",ms=10)
        if defend_point is not None:
            plt.plot(defend_point[0][0],defend_point[0][1],"Dy",ms=10)
        if pursuit_point is not None:
            plt.plot(pursuit_point[0][0],pursuit_point[0][1],"Dr",ms=10)
        plt.axis([0, 50, 0, 50])
        plt.grid(True)
        plt.pause(0.02)
        #plt.show()


