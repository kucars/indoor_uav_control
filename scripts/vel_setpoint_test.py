#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify

#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233

import rospy
import thread
import threading
import time

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from math import *
from mavros.srv import CommandBool
from mavros.utils import *
from std_msgs.msg import Header
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0 

        try:
            thread.start_new_thread(self.navigate, () )
        except:
            print "Error: Unable to start thread"
        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()
        sub = rospy.Subscriber('/cmd_vel_test', TwistStamped, self.velocityCallback)

    def navigate(self):
        print("navigate") 
        rate = self.rospy.Rate(20) # 10hz
        msg = TwistStamped()
        msg.header = Header() 
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = rospy.Time.now()
    
        while 1:
            msg.twist.linear.x = self.x
            msg.twist.linear.y = self.y
            msg.twist.linear.z = self.z
            msg.twist.angular.z = self.yaw 
            self.pub.publish(msg) # it publish it to the hardware ( the motors) 
            print "Published velocity to auto x:" + str(self.x) + " y:" + str(self.y) + " z:" + str(self.z) + " yaw:" + str(self.yaw)   
            rate.sleep()

    def set(self, x, y, z,yaw, delay=0, wait=True):
        print("Set") 
        # I should call this function when I do subscribe to my data         
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw 

        if wait:
            rate = rospy.Rate(5)
            while not self.done:
                rate.sleep()

        time.sleep(delay)


    def velocityCallback(self, topic):
        print("Got vel");
        #print topic.twist.linear.x ;  
        self.set( topic.twist.linear.x ,  topic.twist.linear.y ,topic.twist.linear.z , topic.twist.angular.z,  0, False) 

    	self.done = False ; 
        self.done_evt.set() ;
         

def setpoint_demo():
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    rospy.init_node('vel', anonymous=True)
    rate = rospy.Rate(10) 
    setpoint = Setpoint(pub, rospy) # create an object and send the publisher and the node

    while not rospy.is_shutdown():
        rate.sleep()
#    print "Climb"
#    setpoint.set(0.1, 0.0, 1.0, 0) #  called a function from the class // I should send my data here 
    #print "DONE"
    print "Bye!"


if __name__ == '__main__':
	try:
  		setpoint_demo()
	except rospy.ROSInterruptException:
		pass

