#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
import rospy
import random

from geometry_msgs.msg import Twist

import tf

import numpy as np
import actionlib
import requests
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
from nav_msgs.msg import Odometry

from obstacle_detector.msg import Obstacles, SegmentObstacle, CircleObstacle

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class NaviBot():

    waypoint_list = []

    waypoint_list.append([0.0,0.0,0])#0
    waypoint_list.append([0.0,0.0,0])#1
    waypoint_list.append([0.0,0.0,0])#2
    waypoint_list.append([0.0,0.0,0])#3
    waypoint_list.append([0.0,0.0,0])#4
    waypoint_list.append([0.0,0.0,0])#5
    waypoint_list.append([0.85,0.5,-3.1415/2])#6
    waypoint_list.append([0.25,0.5,3.1415/2])#7
    waypoint_list.append([0.85,-0.5,-3.1415/2])#8
    waypoint_list.append([0.25,-0.5,3.1415/2])#9
    waypoint_list.append([-0.25,0.5,-3.1415/2])#10
    waypoint_list.append([-0.85,0.5,3.1415/2])#11
    waypoint_list.append([-0.25,-0.5,-3.1415/2])#12
    waypoint_list.append([-0.85,-0.5,3.1415/2])#13
    waypoint_list.append([0.5,0.0,-3.1415/2])#14
    waypoint_list.append([0.0,-0.5,0])#15
    waypoint_list.append([0.0,0.5,3.1415])#16
    waypoint_list.append([-0.5,0.0,3.1415/2])#17



    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.get_rosparams()
        #tfのlistenerとbroadcasterの生成
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.all_field_score = np.ones([18])  # field score state
        rospy.Subscriber("enemy_position", Odometry, self.enemy_position_callback)
        self.enemy_position = Odometry()

        self.enemy_distance_prev = 0.0
        self.enemy_direction_diff_prev = 0.0
        self.enemy_detect_last_time = rospy.get_time()

        #直接車輪に制御命令を送るpublisherの生成
        self.direct_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # war_stateの定期更新する
        rospy.Timer(rospy.Duration(1), self.get_war_state)



#        self.enemy_info = [0.0, 0.0]
#        self.detect_counter = 0


    def get_rosparams(self):
        self.my_side = rospy.get_param('side')
        self.robot_namespace = rospy.get_param('robot_namespace')
        self.judge_server_url = rospy.get_param('/send_id_to_judge/judge_url')

    def enemy_position_callback(self, msg):
        self.enemy_position = msg

    def detect_enemy(self):
        #base_frame_name = "base_link"
        base_frame_name = "map"
        enemy_frame_name = self.robot_namespace + "/enemy_closest"
        trans, rot, res = self.get_position_from_tf(enemy_frame_name, base_frame_name)

        if res == False:
            return False, 0.0, 0.0
        
        enemy_dist = math.sqrt(trans[0]*trans[0]+trans[1]*trans[1])
        enemy_dire = math.atan2(trans[1], trans[0])
        
        self.enemy_distance_prev = enemy_dist
        self.enemy_direction_diff_prev = enemy_dire

        self.enemy_detect_last_time = rospy.get_time()

        return True, enemy_dist, enemy_dire

    def get_position_from_tf(self, target_link, base_link):
        trans = []
        rot = []
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                base_link, target_link, rospy.Time(0))
            return trans, rot, True
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('tf error')
            return trans, rot, False


    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        ###
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        


    def get_war_state(self, dummy):
        req = requests.get(self.judge_server_url+"/warState")
        dic = req.json()

        #scoreの取得
        self.my_score = int(dic["scores"][self.my_side])
        if self.my_side == "r":
            self.enemy_score = int(dic["scores"]["b"])
        else:
            self.enemy_score = int(dic["scores"]["r"])

        #time_stampの取得
        self.game_timestamp = int(dic["time"])
        if dic["state"] == "running":
            self.last_game_timestamp = self.game_timestamp

        #フィールドのスコアを取得する
        for idx in range(18): # フィールドターゲットの数は18個
            target_state = dic["targets"][idx]["player"] #ターゲットを取得しているプレイヤーを取得

            if target_state == "n": #自分も敵もターゲットを取得していない
                self.all_field_score[idx] = 1
            elif target_state == self.my_side: #自分がターゲットを取得している
                self.all_field_score[idx] = 0
            else: #敵がターゲットを取得している
                self.all_field_score[idx] = 2

    def select_target_pos(self):

        unaquired_target_idx_list = []
        all_field_score = self.all_field_score #最新のフィールドスコア状況を取得
        
        for idx in range(6, 18): #全てのターゲットに対して、誰が取っているかを確認
            # idx 0~5はロボットについているマーカーなので無視

            if all_field_score[idx] == 0:
                pass #自分が取得しているのでパス

            else:
                unaquired_target_idx_list.append(idx)

        # 未取得のターゲット（ロボットについているものは除く）が無い場合
        if len(unaquired_target_idx_list) == 0:
            return -1 
        
        target_way_cost_list = []

        for target_idx in unaquired_target_idx_list:
            #一定距離内かどうか
            dist_idx=self.dist_target_from_mypos(target_idx)
            if dist_idx<0.8:
                target_way_cost_list.append(1)
            elif dist_idx<1.5:
                target_way_cost_list.append(2)
            elif dist_idx<2.5:
                target_way_cost_list.append(3)
            else:
                target_way_cost_list.append(4)

            #敵が最近までいたかどうか
		##未実装##
        return unaquired_target_idx_list[target_way_cost_list.index(min(target_way_cost_list))]



    def dist_target_from_mypos(self,target_idx):
        trans, rot, res = self.get_position_from_tf("map", "base_footprint")
        if res == False:
            return 100
        point_x=self.waypoint_list[target_idx][0]
        point_y=self.waypoint_list[target_idx][1]

        return math.sqrt(pow((point_x - trans[0]), 2) + pow((point_y - trans[1]), 2))


    def strategy(self):
    #    r = rospy.Rate(30) # change speed 5fps

        exist, dist, dire = self.detect_enemy()

        #敵発見

         #遠ざかっているか？
        selected_id=self.select_target_pos()
        gpos_x=self.waypoint_list[selected_id][0]
        gpos_y=self.waypoint_list[selected_id][1]
        gpos_aw=self.waypoint_list[selected_id][2]

        self.setGoal(gpos_x,gpos_y,gpos_aw)


if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    loop_rate = rospy.Rate(30) #30Hz
    while not rospy.is_shutdown():
        bot.strategy()    
        loop_rate.sleep()
