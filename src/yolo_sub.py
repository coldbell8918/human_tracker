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
from utils.track_helper.help_functions import matching_target,goal_pub,track_function,matching_datas
from utils.track_helper.help_transform import data_transform,tf_transform

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
        self.cnt=int(configs['lower count setting']['cnt'])
        self.cnt2=int(configs['lower count setting']['cnt2'])
        self.searching_cnt_lim=int(configs['upper count setting']['searching_cnt_lim'])
        self.waiting_cnt_lim=int(configs['upper count setting']['waiting_cnt_lim'])
        self.finding_cnt_lim=int(configs['upper count setting']['finding_cnt_lim'])
        self.distance_lim=float(configs['lower distance setting']['distance_lim'])

        #Subscribing rostopics
        self.subs1=[rospy.Subscriber('tracker', track, self.cam_track_callback)]
        self.subs2=[rospy.Subscriber('people_tracked', PersonArray, self.person_callback)]
        self.sub3=[rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.robot_current_pos)]
        
        self.robot_x=0
        self.robot_y=0
        self.robot_th=0

        #Publicating a rostopic : goal position
        self.pub=rospy.Publisher('/move_base/current_goal_follow',PoseStamped,queue_size=1)
        self.goal_msg=PoseStamped()
        self.r=rospy.Rate(30)
        
        #For saveing the last target's position
        self.save_last_goal=list()

        #Checking a target id 
        self.camera_id_none,self.lidar_id_none =True,True

        #save_data
        self.camera_data,self.lidar_data=list(),list()
        
    
    def robot_current_pos(self,data): ### Robot's current position
        positions=data.pose.pose
        self.robot_x=positions.position.x
        self.robot_y=positions.position.y
        self.robot_th=positions.orientation.z

    def cam_track_callback(self, data):
        merge=data_transform(data,'cam')
        if self.camera_id_none:
            self.camera_track(merge)
            if len(merge)!=0:
                self.camera_cmd(merge)
        else:
            self.camera_cmd(merge)

    def camera_track(self, list_): ### making a target human who is near by the robot
        print("Make a target id")
        if len(list_)!=0:
            self.camera_id_none,self.target_c,target_axis=track_function(list_)
            
            return target_axis
        
        else: 
            self.target_c= None
            self.camera_id_none=True
            return None,None,None
        
    def wait_time_cnt(self,list_):  ### the time is overing its limitation, then the robot will be waiting(stopping)
        if  self.cnt2<self.waiting_cnt_lim and matching_target(list_,self.target_c,'len')==0 : 
            print('Robot is waiting the target until count {}/{}'.format(self.cnt2,self.waiting_cnt_lim))
            if self.cnt2==self.waiting_cnt_lim-1:
                self.cnt2,self.cnt=0,0
                self.camera_track(list_)
                
    def camera_cmd(self, list_):   
        if matching_target(list_,self.target_c,'len')!=0:
            if self.cnt>=self.finding_cnt_lim:  ### considering misses the target a couple of second
                print('\nFind the target {} \n'.format(self.target_c))
            self.camera_move(list_,save=True)
            print("Track id {}".format(self.target_c))
            # if list_[self.target_c][]
            self.cnt=0
        else:
            if self.cnt>self.searching_cnt_lim: 
                if self.cnt2<self.waiting_cnt_lim:
                    self.wait_time_cnt(list_)
                self.cnt2+=1
            else: 
                self.camera_move(list_,save=False)
                print("Robot is deriving for searching the target until count: {}/{}".format(self.cnt,self.searching_cnt_lim))
                self.cnt+=1
    
    def camera_move(self, list_,save): ### Pointing a robot's goal position. When the robot missed the target ,then save target's last position 
        if save:             
            num=matching_target(list_,self.target_c,'index')
            x=list_[num][0][2][0]-self.distance_lim
            y=list_[num][0][2][1]-self.distance_lim

            x,y= tf_transform(self.robot_x,self.robot_y,self.robot_th,x,y)

            if len(self.save_last_goal)!=0:
                self.save_last_goal=list()
            self.save_last_goal.append(x)
            self.save_last_goal.append(y)
        # print(self.goal_msg)
        self.goal_msg=goal_pub(self.goal_msg,self.save_last_goal)
        self.pub.publish(self.goal_msg)
        # print(self.goal_mssg)
        self.r.sleep()

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
    tracker=Track(config)
    rospy.spin()


    #### while -> camera or lidar 