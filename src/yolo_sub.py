#!/usr/bin/env python3
from human_following.msg import camera_person, camera_persons
from leg_tracker.msg import Person, PersonArray, Leg, LegArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

import rospy 
import numpy as np
import time
import os
import math

from std_srvs.srv import SetBool, SetBoolResponse 

from utils.track_helper.config_info import config_maker,config_reader,config_comparer
from utils.track_helper.help_track import make_target_function
from utils.track_helper.help_matching import matching_data,cam_aof_check
from utils.track_helper.help_transform import data_transform
from utils.track_helper.help_cmd import cmd_decision

# _______________Config_______________ #

file_path=os.path.dirname(__file__) + '/config/following_config.ini'

if os.path.isfile(file_path):
    print('The config file exist, path: \n {}'.format(file_path))
    config_comparer(file_path)
    config=config_reader(file_path)
else:
    print("The config file doesn't exist, therefor will make a file, parh:\n {} ".format(file_path))
    config_maker(file_path)
    config=config_reader(file_path)
# _______________Config_______________ #

class Track():
    def __init__(self,configs):
        
        
        # ____________constant___________#
        self.cnt=int(configs['lower count setting']['cnt'])
        self.cnt2=int(configs['lower count setting']['cnt2'])
        self.searching_cnt_lim=int(configs['upper count setting']['searching_cnt_lim'])
        self.id_or_axis=str(configs['track type setting']['type'])
        self.const=float(configs['matching setting']['matching const'])
        self.max_linear=float(configs['cmd setting']['max_linear const'])
        self.max_angular=float(configs['cmd setting']['max_angular const'])
        self.angular_const=float(configs['cmd setting']['angular const'])
        self.linear_const=float(configs['cmd setting']['linear const'])
        self.cam_aov=int(configs['camera aov']['angle of view'])
        self.is_service=True
        
        # ____________constant___________#


        # _________missing check_________#
        self.make_target_cam,self.make_target_lidar=True,True
        self.to_lidar,self.to_cam=False,False
        self.cam_merge,self.lidar_merge,self.target_cam,self.target_lidar=list(),list(),None,None         
        # _________missing check_________#


        # ____________publish____________#
        self.pubs = {}
        self.pubs['cmd_vel'] = rospy.Publisher("cmd_vel", Twist,queue_size=1)
        self.pubs['marker'] = rospy.Publisher("tracker/marker", Marker, queue_size=1)
        # ____________publish____________#
        
        # ____________visual_____________#
        self.visual_mesh=os.path.dirname(__file__) +'/utils/visualize_target/Body_mesh.dae'
        self.marker=Marker()
        # ____________visual_____________#


        
        # ___________subscribe___________#
        self.subs1=[rospy.Subscriber('tracker/data', camera_persons, self.cam_callback)]
        self.subs2=[rospy.Subscriber('people_tracked', PersonArray, self.lidar_callback)]
        # self.sub3=[rospy.Subscriber('tracker/data/crops',None, self.crops_callback)]
        # ___________subscribe___________#

        # ____________service____________#
        self.srv=rospy.Service('follow_on',SetBool, self.launch_callback)
        # ____________service____________#


        # ____________fustion____________#
        self.timer=rospy.Timer(rospy.Duration(0.1),self.merge_callback)
        # ____________fustion____________#



# _______________CAM and LIDAR_______________ #
# _________________callback__________________ #
    def cam_callback(self, data):

        """
        Description: callback for detected human pose
        Args:
            data (human_following/track): detected human list
        """
        self.cam_merge,self.crop_images=data_transform(data,'cam','all')

    def lidar_callback(self, data):
        """
        Description: callback for detected human pose
        Args:
            data (leg_tracker/PersonArray): detected human list
        """
        self.lidar_merge=data_transform(data,'lidar','all')
# _______________CAM and LIDAR_______________ #
# _________________callback__________________ #



# ____________________CAM____________________ #

    def cam_target_maker(self, list_): ### making a target human who is near by the robot
        """
        Description: making a target human
        Args:
            list_ (list array): detected human list e.g. [[0,id,(x,y,depth)],[0,id,(x,y,depth)],[...]]
        """
        print("Make a target id from CAMERA")
        if len(list_)!=0:
            self.target_cam_id,self.target_cam,index =make_target_function(list_,self.id_or_axis,'cam')
            self.make_target_cam=False
            self.to_lidar=False
            self.target_img=self.crop_images[index]
        else:  
            self.target_cam=list()

    def cam_missing_check(self, list_,target):   

        """
        Description: checking which the target missing or not 
        Args:
            list_ (list array): detected human list e.g. [[0,id,(x,y,depth)],[0,id,(x,y,depth)],[...]]
        
        """
        self.target_cam=target
        if self.target_cam!=None:

            if len(list_)!=0:
                datas=list_,self.target_cam,self.target_img,self.crop_images
                if matching_data(*datas,'len',self.id_or_axis,self.const,'cam')==0:
                    if self.to_lidar==False:
                        print('\ncam missing {}\n'.format(self.cnt))
                        if self.cnt==self.searching_cnt_lim: 
                            self.to_lidar=True
                            print("\n Missing the target human form CAMEAR\n")
                            self.cnt=0
                        self.cnt+=1 
                else:
                    self.to_lidar=False
                    self.target_cam_id,self.target_cam,index=matching_data(*datas,'id_axis',self.id_or_axis,self.const,'cam')
                    self.target_img=self.crop_images[index]
                    print("Track id from CAMERA {}".format(self.target_cam_id))
                    self.cnt=0
            else:
                if self.to_lidar==False:
                    if self.cnt==self.searching_cnt_lim: 

                        self.to_lidar=True
                        print("\n Missing the target human form CAMEAR\n")
                        self.cnt=0
                    print('\ncam missing {}\n'.format(self.cnt))
                    self.cnt+=1 
# ____________________CAM____________________ #



# ____________________LIDAR____________________ #

    def lidar_target_maker(self, list_): ### making a target human who is near by the robot
        """
        Description: making a target human
        Args:
            list_ (list array): detected human list e.g. [[0,id,(x,y,depth)],[0,id,(x,y,depth)],[...]]
        """
        
        print("Make a target id from LIDAR")
        if len(list_)!=0:
            self.target_lidar_id,self.target_lidar=make_target_function(list_,self.id_or_axis,'lidar')
            self.target_lidar_id,self.target_lidar=self.cam_aof_check(self.target_lidar_id,self.target_lidar)
            if self.target_lidar!=None:
                self.make_target_lidar=False
                self.to_cam=False
            else:
                self.target_lidar=list()
        else:
            self.target_lidar=list()

    def lidar_missing_check(self, list_,target):   
        """
        Description: checking which the target missing or not 
        Args:
            list_ (list array): detected human list e.g. [[0,id,(x,y,depth)],[0,id,(x,y,depth)],[...]]
        """
        self.target_lidar=target
        if self.target_lidar!=None:
            if len(list_)!=0:
                datas=list_,self.target_lidar,None,None
                if matching_data(*datas,'len',self.id_or_axis,self.const,'lidar')==0:
                    if self.to_cam==False:
                        print('\nlidar missing {}\n'.format(self.cnt2))
                        if self.cnt2==self.searching_cnt_lim: 
                            self.to_cam=True
                            print("\n Missing the target human form LIDAR\n")
                        #     self.cnt2=0
                        # self.cnt2+=1 
                else:
                    self.to_cam=False
                    self.target_lidar_id,self.target_lidar=matching_data(*datas,'id_axis',self.id_or_axis,self.const,'lidar')
                    print("Track id from LIDAR {}".format(self.target_lidar_id ))
                    self.cnt2=0

            else:
                if self.to_cam==False:
                    print('\nlidar missing {}\n'.format(self.cnt2))
                    if self.cnt2==self.searching_cnt_lim: 
                        self.to_cam=True
                        print("\n Missing the target human form LIDAR\n")
                    #     self.cnt2=0
                    # self.cnt2+=1 

# ____________________LIDAR____________________ #

# ____________________Launch____________________ #

    def launch_callback(self, req):
        if req.data:
            self.is_service=True
            return SetBoolResponse(True, 'Success')
        else:
            self.is_service=False
            return SetBoolResponse(False, 'Fail')
        
# ____________________Launch____________________ #


# ___________________Velocity__________________ #

    def velocity(self,axis,cam_lidar):
        """
        Description: making a velocity
        Args:
            axis : the target's axis
            cam_lidar : None , lidar, cam
        """   
        if cam_lidar=='lidar' or cam_lidar=='cam':
            x,y,depth=axis[0],axis[1],axis[2]  ## x,y,depth

        elif cam_lidar=='None':
            x,y,depth=axis[0],axis[1],axis[2] ## x,y,depth
        
        try:
            theta=math.atan(y/x)
        except:
            theta=0.0
        
        self.angular=theta*self.angular_const+self.angular_const*0.1
        self.linear=depth*self.linear_const+self.linear_const*0.1

        if self.angular>self.max_angular:
            self.angular=self.max_angular
        if self.linear>self.max_linear:
            self.linear=self.max_linear
        velocity=cmd_decision(self.linear,self.angular,x,y,depth)
        self.cmd(velocity[0],velocity[1])


# ___________________Velocity__________________ #



# _____________________CMD_____________________ #

    def cmd(self, linear, angular):
        """
        Description: cmd publisher 
        Args:
            linear : linear velocity
            angular : angular velocity
        """
        cmd_msg=Twist()
        cmd_msg.linear.x=linear
        cmd_msg.angular.z=angular
        self.pubs['cmd_vel'].publish(cmd_msg)
# _____________________CMD_____________________ #



# ____________________Fusion____________________ #
    
    def merge_callback(self, timer):
        """
        Description: Final code
        """

        print('cam{}'.format(self.target_cam))
        print('lidar{}'.format(self.target_lidar))
        if not self.is_service: return
        
        if cam_aof_check(self.target_lidar_id,self.target_lidar,self.cam_aov)[0]!=None: 
            self.to_lidar=False
        else:
            pass

        if self.make_target_cam==True and self.make_target_lidar ==True:
            """
                Description : initiate code 
            """
            print('\n Make target by Cam and Lidar\n')
            self.cam_target_maker(self.cam_merge)
            self.lidar_target_maker(self.lidar_merge)
            
        elif self.make_target_cam and self.make_target_lidar ==False:
            print('\n Make target by Cam \n Lidar has it')
            self.lidar_missing_check(self.lidar_merge,self.target_lidar)
            self.cam_target_maker([[0,self.target_lidar_id,(self.target_lidar)]])
            
            self.cam_target_maker(self.cam_merge)
            self.visualize_target(self.target_lidar)
            self.velocity(self.target_lidar,'lidar')
            
        elif self.make_target_cam==False and self.make_target_lidar :
            print('\n Make target by Lidar \n Cam has it')
            self.cam_missing_check(self.cam_merge,self.target_cam)
            self.lidar_target_maker([[0,self.target_cam_id,(self.target_cam)]])
            self.visualize_target(self.target_cam)
            self.velocity(self.target_cam,'cam')



        else:
            """
                Description : tracking code 
            """
            if self.to_lidar ==False  and self.to_cam ==False:
                self.lidar_missing_check(self.lidar_merge,self.target_lidar)
            
                if cam_aof_check(self.target_lidar_id,self.target_lidar,self.cam_aov)[0]!=None: 
                    self.cam_missing_check(self.cam_merge,self.target_cam)
                else:
                    pass

                self.visualize_target(self.target_cam)
                if self.target_lidar==None:
                    self.velocity([0,0,0],'None')
                else:
                    self.velocity(self.target_cam,'cam')

            elif self.to_lidar ==False and self.to_cam:
                print('change data to cam')
                self.target_lidar=self.target_cam
                self.lidar_missing_check(self.lidar_merge,self.target_lidar)
                self.cam_missing_check(self.cam_merge,self.target_cam)
                self.visualize_target(self.target_cam)

                if  self.target_cam==None:
                    self.velocity([0,0,0],'None')
                else:
                    self.velocity(self.target_cam,'cam')

            elif self.to_lidar and self.to_cam==False:
                print('change data to lidar')
                self.target_cam=self.target_lidar
                
                self.lidar_missing_check(self.lidar_merge,self.target_lidar)
                self.visualize_target(self.target_lidar)
                if self.target_lidar==None:
                   self.velocity([0,0,0],'None')
                else:
                    self.velocity(self.target_lidar,'lidar')

            elif self.to_cam and self.to_lidar:
                self.lidar_target_maker(self.lidar_merge)
                self.cam_target_maker(self.cam_merge)
                self.velocity([0,0,0],'None')

# ____________________Fusion____________________ #




# ____________________Visualize__________________#  
    def visualize_target(self,axis):
        self.marker.ns='anythings'
        self.marker.id=1
        self.marker.header.frame_id="base_scan"
        self.marker.color.a=1.0
        self.marker.color.r,self.marker.color.g ,self.marker.color.b = 1.0,1.0,1.0
        self.marker.text='Target'
        self.marker.pose.position.x=axis[0]
        self.marker.pose.position.y=axis[1]
        self.marker.pose.position.z=1
        self.marker.frame_locked=True
        self.marker.scale.x , self.marker.scale.y,self.marker.scale.z  = 0.2,0.2,0.2
        self.marker.mesh_use_embedded_materials=True
        self.marker.mesh_resource=self.visual_mesh
        self.pubs['marker'].publish(self.marker)

if __name__ == '__main__':
    rospy.init_node('Tracking_and_Following', anonymous=True)
    cls_ = Track(config)
    rospy.spin()

