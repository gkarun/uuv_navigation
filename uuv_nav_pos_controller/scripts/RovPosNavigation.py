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
from __future__ import print_function
import os
import rospy
import numpy as np
import tf

from std_msgs.msg import Bool
from scipy.integrate import solve_ivp

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Twist, Accel, Vector3

from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import tf.transformations as trans

v = 1
Rc = 1.5
k_alpha = 2.1
k_beta = 2.1

def sgn(x):
    if x>0:
        return 1.0
    else :
        return -1.0

def sat(x):
    return 2/(1+np.exp(-10*x)) -1

def dubins3D(t,x, u_alpha, u_beta):
    alpha = x[3];
    beta  = x[4];

    dx = v*np.cos(beta)*np.cos(alpha)
    dy = v*np.cos(beta)*np.sin(alpha)
    dz = -v*np.sin(beta)

    dalpha = u_alpha
    dbeta = -u_beta

    dx_dt = [dx, dy, dz, dalpha, dbeta]
    return dx_dt


class RovNavigation:
    def __init__(self):

        # print("[ROV_NAV]---------------------------------------------------- Start of Init --------------------------------------------------------------------------")
        # Stub for the current local position of the vehicle
        self.local_pos = Point(0.0, 0.0, 0.0)
        
        # Stub for the current local orientation of the vehicle
        self.local_orientation = np.array([0.0, 0.0, 0.0, 1.0])

        # Stub for the previous local position of the drone
        self.prev_local_pos = Point(0.0, 0.0, 0.0)
        
        # Stub for the current local position of the vehicle
        self.target_pos = Point(0.0, 0.0, -20.0)

        # topic to publish the calculated velocities
        self._output_pub = rospy.Publisher('cmd_pose', PoseStamped, queue_size=1)

        # Pose feedback
        self.sub_odometry = rospy.Subscriber('odom', Odometry, self.odometry_callback)
        # print("[ROV_NAV]---------------------------------------------------- End of Init --------------------------------------------------------------------------")



    def odometry_callback(self, msg):
        """Handle odometry callback: The actual control loop."""
        # rospy.logdebug("[ROV_NAV] : ---------------- Odometry_callback start -------------------")

        self.local_pos.x = msg.pose.pose.position.x
        self.local_pos.y = msg.pose.pose.position.y
        self.local_pos.z = msg.pose.pose.position.z
        self.local_orientation[0] = msg.pose.pose.orientation.x
        self.local_orientation[1] = msg.pose.pose.orientation.y
        self.local_orientation[2] = msg.pose.pose.orientation.z
        self.local_orientation[3] = msg.pose.pose.orientation.w
        # rospy.logdebug("[ROV_NAV] : ---------------- Odometry_callback end -----------------")
        # rospy.logdebug("[ROV_NAV] : odm_call_back -> curr_pos is %s", self.local_pos)

           
    def _calculate_waypoint(self):
        # rospy.logdebug("[ROV_NAV] : ------------------- start of _calculate_waypoint --------------------")
        cmd = PoseStamped()
        cmd.pose.position.x = self.local_pos.x
        cmd.pose.position.y = self.local_pos.y
        cmd.pose.position.z = self.local_pos.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        cmd.pose.orientation.x = self.local_orientation[0]
        cmd.pose.orientation.y = self.local_orientation[1]
        cmd.pose.orientation.z = self.local_orientation[2]
        cmd.pose.orientation.w = self.local_orientation[3]

        ex = self.target_pos.x - self.local_pos.x
        ey = self.target_pos.y - self.local_pos.y
        ez = self.target_pos.z - self.local_pos.z

        self.R     = np.sqrt(ex**2 + ey**2 + ez**2)
        self.theta = np.arctan2(ey, ex)
        self.phi   = np.arctan(ez/np.sqrt(ex**2 + ey**2))
        
        if self.R < Rc :
            insideRc = True
            rospy.loginfo_once("[ROV_NAV] : ---------------------- <<<<< TARGET REACHED >>>>> --------------------------------")
            rospy.loginfo_once("[ROV_NAV] : current position of the vehicle is %s", self.local_pos)
            return cmd
        else :
            insideRc = False

            
        euler_angles = trans.euler_from_quaternion(self.local_orientation)
        self.alpha = euler_angles[2]
        self.beta = euler_angles[1]

        
        self.rel_azimuth  = self.theta - self.alpha
        self.rel_azimuth  = np.arctan2(np.sin(self.rel_azimuth), np.cos(self.rel_azimuth))
        self.rel_elev = self.phi + self.beta
        #rel_elev = np.arctan(np.sin(rel_elev)/np.cos(rel_elev))
        
        #rospy.loginfo("[ROV_NAV] : alpha = %s, theta = %s, beta = %s, phi = %s, rel_azimuth = %s, rel_elev = %s", self.alpha*180/np.pi, self.theta*180/np.pi, self.beta*180/np.pi, self.phi*180/np.pi, self.rel_azimuth*180/np.pi, self.rel_elev*180/np.pi)

        u_alpha = k_alpha*sat(self.rel_azimuth)
        u_beta  = k_beta*sat(self.rel_elev)
        
        
        
        t_span = np.linspace(0, 0.5, 10000)
        x0 = [self.local_pos.x, self.local_pos.y, self.local_pos.z, self.alpha, self.beta]
        sol = solve_ivp(lambda t, x: dubins3D(t,x,u_alpha,u_beta),[t_span[0],t_span[-1]],x0, t_eval = t_span, rtol = 1e-5)
        
        cmd.pose.position.x = sol.y[0,-1]
        cmd.pose.position.y = sol.y[1,-1]
        cmd.pose.position.z = sol.y[2,-1]
        quaternion = tf.transformations.quaternion_from_euler(0, sol.y[4,-1], sol.y[3,-1])
        cmd.pose.orientation.x = quaternion[0]
        cmd.pose.orientation.y = quaternion[1]
        cmd.pose.orientation.z = quaternion[2]
        cmd.pose.orientation.w = quaternion[3]

        return cmd

if __name__ == '__main__':
    print("---------------------pos_control --------------------------------------------")
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    nav = RovNavigation()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd = nav._calculate_waypoint()
        #print("[ROV_NAV]----------------------------------------------------  after caculate_waypoint-------------------------------------------------------------------------")
        #print("[ROV_NAV] value of cmd ->",cmd)
        nav._output_pub.publish(cmd)
        rate.sleep()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
