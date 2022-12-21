#!/usr/bin/env python3

import rospy 
from human_following.msg import track
import numpy as np
from leg_tracker.msg import Person, PersonArray, Leg, LegArray
import time
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from utils.track_helper.config_info import config_maker,config_reader,config_comparer
from utils.track_helper.help_track import track_function
from utils.track_helper.help_matching import matching_data,matching_target
from utils.track_helper.help_transform import data_transform
from geometry_msgs.msg import Twist
import math

file_path=os.path.dirname(__file__) + '/config/following_config.ini'

if os.path.isfile(file_path):
    print('The config file exist, path: \n {}'.format(file_path))
    config_comparer(file_path)
    config=config_reader(file_path)
else:
    print("The config file doesn't exist, therefor will make a file, parh:\n {} ".format(file_path))
    config_maker(file_path)
    config=config_reader(file_path)

class Track():
    def __init__(self,configs):
        #Config
        self.subs1 = []
        self.pubs = {}
        self.srvs = []

        self.cnt=int(configs['lower count setting']['cnt'])
        self.cnt2=int(configs['lower count setting']['cnt2'])
        self.searching_cnt_lim=int(configs['upper count setting']['searching_cnt_lim'])
        self.waiting_cnt_lim=int(configs['upper count setting']['waiting_cnt_lim'])
        self.finding_cnt_lim=int(configs['upper count setting']['finding_cnt_lim'])
        self.distance_lim=float(configs['lower distance setting']['distance_lim'])
        self.pubs['cmd_vel'] = rospy.Publisher("cmd_vel", Twist,queue_size=1)

        #Subscribing rostopics
        self.subs1.append(rospy.Subscriber('tracker', track, self.cam_track_callback))
        # self.subs2=[rospy.Subscriber('people_tracked', PersonArray, self.person_callback)]
        # self.sub3=[rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.robot_current_pos)]
        
        self.robot_x=0
        self.robot_y=0
        self.robot_th=0
        self.linear=0.25
        self.angular=0
        self.delta=0.95
        #Publicating a rostopic : goal position
        # self.pub=rospy.Publisher('/move_base/current_goal_follow',PoseStamped,queue_size=1)
        # self.goal_msg=PoseStamped()
        self.r=rospy.Rate(30)
        
        #For saveing the last target's position
        # self.save_last_goal=list()

        #Checking a target id 
        self.camera_id_none,self.lidar_id_none =True,True

        #save_data
        self.camera_data,self.lidar_data=list(),list()
        
    

    def cam_track_callback(self, data):
        """
        Description: callback for detected human pose
        Args:
            data (human_following/camera_persons): detected human list
        """
        
        merge=data_transform(data,'cam','all')
        if self.camera_id_none:
            self.camera_track(merge)
            if len(merge)!=0:
                self.camera_cmd(merge)
        else:
            self.camera_cmd(merge)

    def camera_track(self, list_): ### making a target human who is near by the robot
        print("Make a target id")
        if len(list_)!=0:
            self.camera_id_none,self.target_c,self.target_axis=track_function(list_)
            
            # return target_axis
        
        else: 
            self.target_c= None
            self.camera_id_none=True
            # return None,None,None
        
    def wait_time_cnt(self,list_):  ### the time is overing its limitation, then the robot will be waiting(stopping)
        if  self.cnt2<self.waiting_cnt_lim and matching_data(list_,self.target_c,'len')==0 : 
            print('Robot is waiting the target until count {}/{}'.format(self.cnt2,self.waiting_cnt_lim))
            if self.cnt2==self.waiting_cnt_lim-1:
                self.cnt2,self.cnt=0,0
                self.camera_track(list_)
                
    def camera_cmd(self, list_):   
        if matching_data(list_,self.target_c,'len')!=0:
            if self.cnt>=self.finding_cnt_lim:  ### considering misses the target a couple of second
                print('\nFind the target {} \n'.format(self.target_c))
            print("Track id {}".format(self.target_c))
            # _,_,target_axis=track_function(list_)
            print('\nRobot has a target now, id: {} \n'.format(self.target_c))
            x,y,z=self.target_axis[1],self.target_axis[0],self.target_axis[2]
            self.velocity(x,y,z)
            self.cnt=0

        else:
            if self.cnt>self.searching_cnt_lim: 
                if self.cnt2<self.waiting_cnt_lim:
                    self.wait_time_cnt(list_)
                self.cnt2+=1
            else: 
                print("Robot is deriving for searching the target until count: {}/{}".format(self.cnt,self.searching_cnt_lim))
                self.cnt+=1

    def velocity(self,x,y,z):
        theta=math.atan(y/x)
        self.angular=theta*self.delta
        if x>0.5: ## meter 
            print(x,y,z)
            if y>0:
                print("turnleft")
                if y<0.25:
                    if x<1:
                        if x>0.8:
                            self.cmd(0,0)
                        else:
                            self.cmd(-self.linear,0)
                        self.cmd(0,0)
                    else:            
                        self.cmd(self.linear, 0)
                else:
                    if z<1:
                        self.cmd(0,0)
                    else:
                        self.cmd(0, self.angular)
            else:
                print("turnright")
                if y>-0.25:
                    if x<1:
                        if x>0.8:
                            self.cmd(0,0)
                        else:
                            self.cmd(-self.linear,0)
                    else:
                        self.cmd(self.linear,0)
                else:
                    if z<1:
                        self.cmd(0,0)
                    else:
                        self.cmd(0, self.angular)
        else: ### when robot is stopping to avoid collusion problem 
            self.cmd(0,0) 

    def cmd(self, linear, angular):
        cmd_msg=Twist()
        cmd_msg.linear.x=linear
        cmd_msg.linear.y=0
        cmd_msg.linear.z=0
        cmd_msg.angular.x=0
        cmd_msg.angular.y=0
        cmd_msg.angular.z=angular
        self.pubs['cmd_vel'].publish(cmd_msg)

##########################
    # def person_callback(self, data):
    #     merge=data_transform(data,'lidar')
    #     if self.lidar_id_none:
    #         self.lidar_track(merge)
    #         if len(merge)!=0:
    #             self.lidar_track(merge)
      
    
    # def lidar_track(self, list_): ### making a target human who is near by the robot
    #     print("Make a target id")

    #     if len(list_)!=0:
    #         self.lidar_id_none,self.target_l,target_axis=track_function(list_)
    #         return self.target_l ,target_axis
        
    #     else: 
    #         self.target_l= None
    #         self.lidar_id_none=True
    #         return None,None,None

    # def make_target(self):
    #     match=matching_datas
    #     self.camera_data
    #     self.lidar_data
###################################### 

if __name__ == '__main__':
    rospy.init_node('sub_yolo', anonymous=True)
    cls_=Track(config)
    rospy.spin()


    #### while -> camera or lidar 