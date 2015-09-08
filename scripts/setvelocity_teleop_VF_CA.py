 
 
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
 

#from tf.transformations import  quaternion_from_euler

import rospy
import thread
import threading
import time

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from std_msgs.msg import Bool
from math import *
from mavros.srv import * # CommandBool
from mavros.utils import *
from std_msgs.msg import Header
from std_msgs.msg import String
from tf.transformations import * # quaternion_from_euler

import cv2
import numpy as np
 
class Setpoint:
 
    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy
        # velocities 
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
	#posetions  
        self.pre_pose_x = 0.0 
        self.pre_pose_y = 0.0 
        self.pre_pose_z = 0.0 
        self.pre_vel_x = 0.0 
        self.pre_vel_y = 0.0 
        self.pre_vel_z = 0.0 
        self.current_pose_x = 0.0 
        self.current_pose_y = 0.0 
        self.current_pose_z = 0.0 
        self.current_vel_x = 0.0 
        self.current_vel_y = 0.0 
        self.current_vel_z = 0.0 
        self.current_yaw =0.0 
        self.pre_yaw = 0.0 
        self.init_data = True 
        self.inCollision = False ; 
        self.lastVelTime = time.time()
        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print "Error: Unable to start thread"
 
        self.done = False
        self.done_evt = threading.Event()
        sub = rospy.Subscriber('/mavros/local_position/local', PoseStamped, self.reached)
        sub2 = rospy.Subscriber('/uav/cmd_vel', TwistStamped, self.velocityCallback)
	sub3 =rospy.Subscriber('/collision_flag', Bool, self.collisionDetectionCallback)


    def collisionDetectionCallback(self, topic):
#      print ("InCollision callback")
      self.inCollision = topic.data 
 #     print ("Collision", self.inCollision)




    def point_inside_polygon(self,x,y,poly):

        n = len(poly)
        inside =False

        p1x,p1y = poly[0]
        for i in range(n+1):
            p2x,p2y = poly[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside



    def navigate(self):
        rate = self.rospy.Rate(10) # 10hz
 
        msg = TwistStamped()
        msg.header = Header() 
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = rospy.Time.now()
        
        while 1:
	    print "Time since last vel:", (time.time() - self.lastVelTime)
	    if (self.inCollision or (time.time() - self.lastVelTime)>1.0):
	     # print "inCollision"
	      msg.twist.linear.x = 0  #self.x
	      msg.twist.linear.y = 0 #self.y
	      msg.twist.linear.z = 0 #self.z
	      msg.twist.angular.z = 0 
            else:
	      msg.twist.linear.x = self.x
	      msg.twist.linear.y = self.y
	      msg.twist.linear.z = self.z
	      msg.twist.angular.z = self.yaw 
	      print "Sending vel"
	    #print "collision:", self.inCollision
            # For demo purposes we will lock yaw/heading to north.
            #yaw_degrees = 0  # North
            #yaw = radians(yaw_degrees)
            #quaternion = quaternion_from_euler(0, 0, yaw)
            #msg.pose.orientation = Quaternion(*quaternion)
            self.pub.publish(msg)
 
            rate.sleep()
 
    def set(self, x, y, z, yaw, delay=0, wait=False):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
 
        if wait:
            rate = rospy.Rate(10)
            while not self.done:
                rate.sleep()
 
        time.sleep(delay)
 
    def velocityCallback(self, topic):
        #print("Got vel");
        self.lastVelTime = time.time()
        self.set( topic.twist.linear.x ,  topic.twist.linear.y ,topic.twist.linear.z , topic.twist.angular.z,  0) 

 
    def reached(self, topic):
        poly = [(0.2,2.3), (-1.8,0.5), (0.0,-1.26), (2.5,0.1)]
        if (self.init_data == True ):
            self.pre_pose_x =  topic.pose.position.x ; 
            self.pre_pose_y =  topic.pose.position.y ; 
            self.pre_pose_z =  topic.pose.position.z ; 
            quaternion = (topic.pose.orientation.x , topic.pose.orientation.y ,topic.pose.orientation.z ,topic.pose.orientation.w )
            euler = euler = euler_from_quaternion(quaternion)#euler_from_quaternion(quaternion)
            self.pre_yaw = euler[2]
            self.pre_t = topic.header.stamp.to_sec()
            self.init_data = False
        else:
            self.current_pose_x =  topic.pose.position.x ; 
            self.current_pose_y =  topic.pose.position.y ; 
            self.fcurrent_pose_z =  topic.pose.position.z ; 
            self.current_t = topic.header.stamp.to_sec()
            self.current_vel_x = self.current_pose_x - self.pre_pose_x / (self.current_t - self.pre_t) 
            self.current_vel_y = self.current_pose_y - self.pre_pose_y / (self.current_t - self.pre_t) 
            self.current_vel_z = self.current_pose_z - self.pre_pose_z / (self.current_t - self.pre_t)
            quaternion = (topic.pose.orientation.x , topic.pose.orientation.y ,topic.pose.orientation.z ,topic.pose.orientation.w )
            euler = euler = euler_from_quaternion(quaternion)#euler_from_quaternion(quaternion)
            self.current_yaw = euler[2]
            

            timestamp = 3 ; 
            #if(self.point_inside_polygon(self.x,self.y,poly) == False):
                # set new set points Or publish a zero velocity 
            #    self.x = 0.0; 
            #    self.y = 0.0; 
            #    self.z = 0.0; 
            #else: 
                # calculate next position 
	    speed_x = self.x
	    #xp = speed_x * cos(self.current_yaw) * timestamp 
	    xp = speed_x * timestamp ; 
            newx = self.current_pose_x + xp ; 
	    speed_y = self.y
	    #yp = speed_y * sin(self.current_yaw) * timestamp 
	    yp = speed_y * timestamp ; 
            newy = self.current_pose_y + yp
	    self.done_evt.set()
 
            
	    if(self.point_inside_polygon(self.current_pose_x,self.current_pose_y,poly) == False):
	      # set new set points Or publish a zero velocity 
	      print "Robot OUTSIDE polygon"
	      self.x = 0.0 ;  
	      self.y = 0.0 ; 
	      self.z = 0.0 ; 
	      self.done = True
	    if(self.point_inside_polygon(newx,newy,poly) == False):
	      print "Next step OUTDIDE the polygon" 
	      # set new set points Or publish a zero velocity 
	      self.x = 0.0 ;  
	      self.y = 0.0 ; 
	      self.z = 0.0 ; 
	      self.done = True
	    else:
	      print "Robot still inside" 
              #print "yaw" , self.current_yaw ;       
	      #print topic.pose.position.z, self.z, abs(topic.pose.position.z - self.z)
                    #if abs(topic.pose.position.x - self.x) < 0.2 and abs(topic.pose.position.y - self.y) < 0.2 and abs(topic.pose.position.z - self.z) < 0.2:
                    #    self.done = True
    	            #print self.done
                    #print "Current Pose:",topic.pose.position.x,topic.pose.position.y,topic.pose.position.z
                    #print "Set Pose:",self.x,self.y,self.z
              
              
def setpoint_demo():
    #pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    rospy.init_node('pose', anonymous=True)
    rate = rospy.Rate(0.5) 
    setpoint = Setpoint(pub, rospy)
 
    '''
    print "move in x axis 1 meter "
    setpoint.set(-0.25, 0.0, 0.0, 0)
    
    time.sleep(5)
    
    print "move in -x axis 1 meter "
    setpoint.set(0.25, 0.0, 0.0, 0)
    
    time.sleep(5)
    
    setpoint.set(0, 0.0, 0.0, 0)
    time.sleep(1)
    
    print "You can stop now"
    '''
    while not rospy.is_shutdown():
      rate.sleep()
   # print "move in -x axis 1 meter "
   # setpoint.set(-1.0, 0.0, 1.0, 0)

#    print "Origin"
#    setpoint.set(0.0, 0.0, 1.0, 0)

 #   print "Outside boundry2"
  #  setpoint.set(0.0, 1.0, 1.0, 0)
    
  #  print "Origin"
  #  setpoint.set(0.0, 0.0, 1.0, 0)

    print "Bye!"
 
 
if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass 

