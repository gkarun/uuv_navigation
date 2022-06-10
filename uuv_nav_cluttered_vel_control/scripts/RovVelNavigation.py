#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#from __future__ import rospy.logdebug_function
import os
import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Accel, Vector3, Point
from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
import tf.transformations as trans
import geometry_msgs.msg as geometry_msgs

v = 0.1
Rc = 1
k_alpha = 0.21
k_beta = 0.21

k_rho = 0.21
k_delta = 0.1

rho_detect = 10
rho_safe = 5
rho_critical = 2



def sgn(x):
    if x>0:
        return 1.0
    else :
        return -1.0

def sat(x):
    return 2/(1+np.exp(-10*x)) -1

class RovNavigation:
    def __init__(self):

        # rospy.logdebug("[ROV_NAV] : ------------------- Start of Init -------------------")
        # Stub for the current local position of the vehicle
        self.local_pos = Point(0.0, 0.0, 0.0)

        # Stub for the current local orientation of the vehicle
        self.local_orientation = np.array([0.0, 0.0, 0.0, 0.0])

        # Stub for the previous local position of the drone
        self.prev_local_pos = Point(0.0, 0.0, 0.0)
        
        # Stub for the current local position of the vehicle
        if rospy.has_param('~target_point') :
            target_pos = np.array(rospy.get_param('~target_point'))
            self.target_pos=geometry_msgs.Point(*target_pos)
            rospy.logdebug("[ROV_NAV] : inside if -------------------------- > Target position is %s", self.target_pos)
        else :
            self.target_pos = Point(0.0, 0.0, 0.0)

        rospy.loginfo("[ROV_NAV] : Target position is %s", self.target_pos)

        #Variable for previous relative azimuth
        self.prev_rel_azimuth = 0.0

        #Variable for previous relative azimuth
        self.rel_azimuth = 0.0

        #Variable for previous relative azimuth
        self.rel_elevation = 0.0

        # Minimum distance to the obstacle
        self.rho = 100.0

        # Angle to the minimum distance point on the obstacle
        self.delta = np.pi
        
        # topic to publish the calculated velocities
        self._output_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Pose feedback
        self.sub_odometry = rospy.Subscriber('odom', Odometry, self.odometry_callback)
        
        # Pose feedback
        self.sub_sonar = rospy.Subscriber('sonar', LaserScan, self.sonar_callback)

        #Flag indicating boundary following 
        self.OA_MODE = False

        #Flag indicating task completion 
        self.NAV_MODE = True


        #Flag indicating task completion 
        self.COLLISION = False


        # rospy.logdebug("[ROV_NAV] : -------------------- End of Init -------------------")


    def odometry_callback(self, msg):
        """Handle odometry callback: Store position feedback."""
        # rospy.logdebug("[ROV_NAV] : ---------------- Odometry_callback start -------------------")
        self.local_pos.x = msg.pose.pose.position.x
        self.local_pos.y = msg.pose.pose.position.y
        self.local_pos.z = msg.pose.pose.position.z
        self.local_orientation[0] = msg.pose.pose.orientation.x
        self.local_orientation[1] = msg.pose.pose.orientation.y
        self.local_orientation[2] = msg.pose.pose.orientation.z
        self.local_orientation[3] = msg.pose.pose.orientation.w
        # rospy.logdebug("[ROV_NAV] : odm_call_back -> curr_pos is %s", self.local_pos)
        # rospy.logdebug("[ROV_NAV] : ---------------- Odometry_callback end -----------------")

           
    def sonar_callback(self, msg):
        """Handle sonar callback: Record and find closest distance point information"""
        # rospy.logdebug("[ROV_NAV] : --------------- sonar_callback start ------------------")
        ranges = np.array(msg.ranges)
        self.rho = np.min(ranges)
        if(self.rho<rho_critical) :
            self.COLLISION = True
            if self.NAV_MODE :
                rospy.loginfo("[ROV_NAV] : ----  ***** COLLISSION **** ------")
                rospy.loginfo("[ROV_NAV] : closest distance point on the obstacle is %s away,", self.rho)
                self.NAV_MODE = False
            return
        
        min_range_index = np.argmin(ranges)
        self.delta = msg.angle_min + min_range_index*msg.angle_increment
        mid = int(np.round(np.fabs(msg.angle_min)/msg.angle_increment))
        look_ahead_index = int(np.abs((mid - 100) % mid))
        rospy.logdebug("[ROV_NAV] : self.rho = %s, self.delta  = %s", self.rho, self.delta)
        # rospy.logdebug("mid = %s, look_ahead_index = %s", mid, look_ahead_index)
        if not self.OA_MODE:
            #check Oobstacle in the viscinity
            if(np.min(ranges[mid-look_ahead_index:mid+look_ahead_index])<rho_detect) :
                self.OA_MODE = True
        #elif((self.prev_rel_azimuth!=self.rel_azimuth) and  np.min(ranges[mid-look_ahead_index:mid+look_ahead_index])>rho_detect) :
        elif sgn(sgn(self.rel_azimuth)>0) :
            rospy.logdebug("[ROV_NAV] : The min value in the left side np.min(ranges[mid:])is %s ",np.min(ranges[mid:]))
            if (np.min(ranges[mid:])>rho_detect or  np.isinf(np.min(ranges[mid:]))) :
                self.OA_MODE = False
        elif sgn(sgn(self.rel_azimuth)<0) :
            rospy.logdebug("[ROV_NAV] : The min value in the right side np.min(ranges[:mid])is %s ",np.min(ranges[:mid]))
            if (np.min(ranges[:mid])>rho_detect or  np.isinf(np.min(ranges[:mid]))) :
                self.OA_MODE = False
        #rospy.logdebug("[ROV_NAV] : ---------- sonar_callback end --------------")
            
    
    def _calculate_vel(self):
        # rospy.logdebug("[ROV_NAV] : ------------------- start of _calculate_waypoint --------------------")
        
        cmd = Twist()
        cmd.linear = Vector3(0.0, 0.0, 0.0)
        cmd.angular = Vector3(0.0, 0.0, 0.0)
        
        #rospy.logdebug("curr_pos = %s", self.local_pos)


        ex = self.target_pos.x - self.local_pos.x
        ey = self.target_pos.y - self.local_pos.y
        ez = self.target_pos.z - self.local_pos.z

        self.R     = np.sqrt(ex**2 + ey**2 + ez**2)
        self.theta = np.arctan2(ey, ex)
        self.phi   = np.arctan(ez/np.sqrt(ex**2 + ey**2))
        
        euler_angles = trans.euler_from_quaternion(self.local_orientation)
        self.alpha = euler_angles[2]
        self.beta = euler_angles[1]
        #self.alpha = np.arctan2( dy, dx )
        #self.beta = np.arctan( dz/np.sqrt(dx**2 + dy**2) )


        self.rel_azimuth  = self.theta - self.alpha
        self.rel_azimuth  = np.arctan2(np.sin(self.rel_azimuth), np.cos(self.rel_azimuth))
        self.rel_elev = self.phi + self.beta
        #rel_elev = np.arctan(np.sin(rel_elev)/np.cos(rel_elev))


        if self.R < Rc :
            self.NAV_MODE = False
            rospy.loginfo_once("[ROV_NAV] : ---------------------- <<<<< TARGET REACHED >>>>> --------------------------------")
            rospy.loginfo_once("[ROV_NAV] : current position of the vehicle is %s", self.local_pos)
            return cmd
        else :
            self.NAV_MODE = True


        if self.OA_MODE:
            rospy.logdebug("[ROV_NAV OA MODE] : alpha = %s, theta = %s, beta = %s, phi = %s, rel_azimuth = %s, rel_elev = %s", self.alpha*180/np.pi, self.theta*180/np.pi, self.beta*180/np.pi, self.phi*180/np.pi, self.rel_azimuth*180/np.pi, self.rel_elev*180/np.pi)
            u_alpha = k_delta*sat(-sgn(self.delta)*np.pi/2 + self.delta) + k_rho*sgn(self.delta)*sat(self.rho-rho_safe)
            u_beta  = k_beta*sat(self.rel_elev)
        else : 
            rospy.logdebug("[ROV_NAV HOMING MODE] : alpha = %s, theta = %s, beta = %s, phi = %s, rel_azimuth = %s, rel_elev = %s", self.alpha*180/np.pi, self.theta*180/np.pi, self.beta*180/np.pi, self.phi*180/np.pi, self.rel_azimuth*180/np.pi, self.rel_elev*180/np.pi)
            u_alpha = k_alpha*sat(self.rel_azimuth)
            u_beta  = k_beta*sat(self.rel_elev)

        rospy.logdebug("[ROV_NAV] : u_alpha = %s, u_beta = %s", u_alpha,u_beta)
        
        #R_bw = trans.quaternion_matrix(self.local_orientation)[0:3, 0:3].transpose()
        cmd_l = np.array([v, 0, 0])
        #cmd_a = R_bw.dot(np.array([0, u_beta, u_alpha]))
        cmd_a = np.array([0, -u_beta, u_alpha])
        cmd.linear = geometry_msgs.Vector3(*cmd_l)
        cmd.angular = geometry_msgs.Vector3(*cmd_a)
        
        # Updating the previous relative_azimuth
        self._prev_rel_azimuth = self.rel_azimuth
        
        return cmd

if __name__ == '__main__':
    rospy.logdebug("[ROV_NAV] : --------------- vel_control -----------------")
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    nav = RovNavigation()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if nav.COLLISION : 
            rospy.loginfo_once("[ROV_NAV] : Collisison Detected. distance to obstacle reduced to %s", nav.rho)
            cmd = Twist()
            nav._output_pub.publish(cmd)
            continue
        cmd = nav._calculate_vel()
        rospy.logdebug("[ROV_NAV] : Value of cmd is %s", cmd)
        nav._output_pub.publish(cmd)
        rate.sleep()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
