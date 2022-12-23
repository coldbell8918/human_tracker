#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('my_node', anonymous=True)
pub = rospy.Publisher('my_topic', Float64MultiArray, queue_size=10)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    my_msg = Float64MultiArray()
    d=[[1.2354567, 99.7890, 67.654236], [67.875, 90.6543, 76.5689], [65.3452, 45.873, 67.8956]]
    print("d : %s, type d : %s"%(d, type(d)))
    d=[[float(d[i][j]) for j in range(len(d))] for i in range(len(d[0]))]
    print("d : %s, type d : %s"%(d, type(d)))
    d = [65.3452, 45.873, 67.8956]
    my_msg.data = d
    pub.publish(my_msg)
    r.sleep()