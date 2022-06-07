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
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Accel, Vector3, Point
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import tf.transformations as trans
import geometry_msgs.msg as geometry_msgs

v = 0.5
Rc = 1
k_alpha = 0.6
k_beta = 0.6

def sgn(x):
    if x>0:
        return 1.0
    else :
        return -1.0
def sat(x):
    return 2/(1+np.exp(-10*x)) -1

class RovNavigation:
    def __init__(self):

        # print("[ROV_NAV]---------------------------------------------------- Start of Init --------------------------------------------------------------------------")
        # Stub for the current local position of the vehicle
        self.local_pos = Point(0.0, 0.0, 0.0)

        # Stub for the current local orientation of the vehicle
        self.local_orientation = np.array([0.0, 0.0, 0.0, 0.0])


        # Stub for the previous local position of the drone
        self.prev_local_pos = Point(0.0, 0.0, 0.0)
        
        # Stub for the current local position of the vehicle
        self.target_pos = Point(0.0, 0.0, -20.0)

        # topic to publish the calculated velocities
        self._output_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Pose feedback
        self.sub_odometry = rospy.Subscriber('odom', Odometry, self.odometry_callback)
        # print("[ROV_NAV]---------------------------------------------------- End of Init --------------------------------------------------------------------------")



    def odometry_callback(self, msg):
        """Handle odometry callback: The actual control loop."""
        # print("[ROV_NAV] ---------------------------------------------------- Odometry_callback start --------------------------------------------------------------------------")

        self.local_pos.x = msg.pose.pose.position.x
        self.local_pos.y = msg.pose.pose.position.y
        self.local_pos.z = msg.pose.pose.position.z
        self.local_orientation[0] = msg.pose.pose.orientation.x
        self.local_orientation[1] = msg.pose.pose.orientation.y
        self.local_orientation[2] = msg.pose.pose.orientation.z
        self.local_orientation[3] = msg.pose.pose.orientation.w
        # print("[ROV_NAV] ---------------------------------------------------- Odometry_callback end --------------------------------------------------------------------------")
        # print("[ROV_NAV] odm_call_back -> curr_pos", self.local_pos)
        # print("[ROV_NAV] odm_call_back -> curr_orientaiton", self.local_orientation)

           
    def _calculate_vel(self):
        # print("[ROV_NAV] ---------------------------------------------------- start of _calculate_vel --------------------------------------------------------------------------")
        
        cmd = Twist()
        cmd.linear = Vector3(0, 0, 0)
        cmd.angular = Vector3(0, 0, 0)
        
        #print("prev_pos", self.prev_local_pos)
        #print("curr_pos", self.local_pos)
        dx = self.local_pos.x - self.prev_local_pos.x
        dy = self.local_pos.y - self.prev_local_pos.y
        dz = self.local_pos.z - self.prev_local_pos.z

        if dx == 0 or dy == 0 or dz == 0 :
            print("[ROV_NAV] ---------------------------------------------------- dx or dy or dz zero --------------------------------------------------------------------------")
            print("Update error waiting for next : dx = ", dx, ", dy = ", dy, ", dz = ", dz)
            return cmd

        ex = self.target_pos.x - self.local_pos.x
        ey = self.target_pos.y - self.local_pos.y
        ez = self.target_pos.z - self.local_pos.z

        self.R     = np.sqrt(ex**2 + ey**2 + ez**2)
        self.theta = np.arctan2(ey, ex)
        self.phi   = np.arctan(ez/np.sqrt(ex**2 + ey**2))

        if self.R < Rc :
            print("Target Reached ")
            insideRc = True
            return cmd
        else :
            insideRc = False


        #print("Printing the pos difference : dx = ", dx, ", dy = ", dy, ", dz = ", dz)
        euler_angles = trans.euler_from_quaternion(self.local_orientation)
        #print("Printing euler angles :  ", euler_angles)

        #self.alpha = np.arctan2( dy, dx )
        #self.beta = np.arctan( dz/np.sqrt(dx**2 + dy**2) )
        
        self.alpha = euler_angles[2]
        self.beta = euler_angles[1]


        dyaw  = self.theta - self.alpha
        dyaw  = np.arctan2(np.sin(dyaw), np.cos(dyaw))
        dpitch = self.phi + self.beta
        #dpitch = np.arctan(np.sin(dpitch)/np.cos(dpitch))

        print("alpha", self.alpha*180/np.pi, "theta", self.theta*180/np.pi, "beta", self.beta*180/np.pi, "phi", self.phi*180/np.pi, "dyaw", dyaw*180/np.pi, "dpitch", dpitch*180/np.pi)

        u_alpha = k_alpha*sat(dyaw)
        u_z  = k_beta*sat(dpitch)


        #print "Setpoint : ", setPoint.position.x, setPoint.position.y, setPoint.position.z
        self.prev_local_pos.x = self.local_pos.x
        self.prev_local_pos.y = self.local_pos.y
        self.prev_local_pos.z = self.local_pos.z


        #Linear velocity
        #cmd.linear.x = v

        # Pitch control  
        #cmd.angular.y = u_beta

        # Yaw control
        #cmd.angular.z = u_alpha
        #print("before ------------------ u_beta - ",u_beta, "--------- u_alpha - ", u_beta)
        #print("local orientation  = ", self.local_orientation)
        R_bw = trans.quaternion_matrix(self.local_orientation)[0:3, 0:3].transpose()
        #print("Rotation matrix ------------------ ", R_bw)

        cmd_l = np.array([v, 0, u_z])
        cmd_a = R_bw.dot(np.array([0, 0, u_alpha]))
        cmd.linear = geometry_msgs.Vector3(*cmd_l)
        cmd.angular = geometry_msgs.Vector3(*cmd_a)
        #print("after ------------------ cmd",cmd)

        return cmd

if __name__ == '__main__':
    print("---------------------Vel_control --------------------------------------------")
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    nav = RovNavigation()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        cmd = nav._calculate_vel()
        #print("[ROV_NAV]----------------------------------------------------  after caculate_vel-------------------------------------------------------------------------")
        #print("[ROV_NAV] value of cmd ->",cmd)
        nav._output_pub.publish(cmd)
        rate.sleep()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
