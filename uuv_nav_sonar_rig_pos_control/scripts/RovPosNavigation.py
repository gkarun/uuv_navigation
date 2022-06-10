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
# from __future__ import rospy.loginfo_function
import os
import rospy
import numpy as np
import tf

from std_msgs.msg import Bool
from scipy.integrate import solve_ivp

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Twist, Accel, Vector3

from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
import tf.transformations as trans
import geometry_msgs.msg as geometry_msgs

# UUV forward velocity
v = 1

# Desired nearness to the target
Rc = 0.5

# Gain parameter for the homing control law
k_alpha = 2.1 # yaw
k_beta = 2.1 # pitch

# Gain parameters for the boundary follwoing control law
k_rho = 1.1    # Distance
k_delta = 1.2  # yaw

# The distance  at which the vehicle detects the obstacle
rho_detect = 10

# The distance at which the vehicle performs boundary following 
rho_safe = 5

# Critical distance. considered colissiin if distance less than this
rho_critical = 2

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

        # rospy.logdebug("[ROV_NAV] : ------------------- Start of Init -------------------")
        # Variable for the current local position of the vehicle
        self.local_pos = Point(0.0, 0.0, 0.0)
	
	# Variable for the desired/commanded pose of the vehicle
        self.des_pose = PoseStamped()
        if rospy.has_param('~init_pose') : # intiializing with the initial position
            init_pos = rospy.get_param('~init_pose')
            init_position = np.array(init_pos['position'])
	    init_orientation = np.array(init_pos['orientation'])
            quaternion = tf.transformations.quaternion_from_euler(init_orientation[0],init_orientation[1],init_orientation[2])
            self.des_pose.pose.position = geometry_msgs.Point(*init_position)
            self.des_pose.pose.orientation = geometry_msgs.Quaternion(*quaternion)	
	
        rospy.loginfo("[ROV_NAV] : Initial position is %s", self.des_pose)

        # Variable for the current local orientation of the vehicle
        self.local_orientation = np.array([0.0, 0.0, 0.0, 0.0])

        # Variable for the previous local position of the drone
        self.prev_local_pos = Point(0.0, 0.0, 0.0)
        
        # Variable for the targe position of the vehicle
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

        # Minimum distance to the obstacle (Complete)
        self.rho = 1000000.0

        # Minimum distance to the obstacle (Front)
        self.front_rho = 1000000.0

        # Minimum distance to the obstacle (Left)
        self.left_rho = 1000000.0

        # Minimum distance to the obstacle (Right)
        self.right_rho = 1000000.0

        # Angle to the minimum distance point on the obstacle (Complete)
        self.delta = np.pi
        
        # Angle to the minimum distance point on the obstacle (Front)
        self.front_delta = np.pi
        
        # Angle to the minimum distance point on the obstacle (Left)
        self.left_delta = np.pi
        
        # Angle to the minimum distance point on the obstacle (Right)
        self.right_delta = np.pi
        
        # topic to publish the calculated velocities
        self._output_pub = rospy.Publisher('cmd_pose', PoseStamped, queue_size=1)

        # Pose feedback
        self.sub_odometry = rospy.Subscriber('odom', Odometry, self.odometry_callback)
        
        # Front sonar feedback
        self.sub_front_sonar = rospy.Subscriber('sonar', LaserScan, self.sonar_front_callback)

	# Left sonar feedback
        self.sub_left_sonar = rospy.Subscriber('sss_left', LaserScan, self.sonar_left_callback)
        
	# right sonar feedback
        self.sub_right_sonar = rospy.Subscriber('sss_right', LaserScan, self.sonar_right_callback)

	
	# Front sonar feedback
        self.front_ranges = []
	
	# Front sonar min angle
        self.front_angle_min = 0.0

	# Front sonar max angle
        self.front_angle_max = 0.0

	# Front sonar angle  increment
        self.front_angle_increment = 0.0

	self.front_mid_index = -100000

	# Left sonar feedback
        self.left_ranges = []
        
	# Left sonar min angle
        self.left_angle_min = 0.0

	# Left sonar max angle
        self.left_angle_max = 0.0

	# Left sonar angle  increment
        self.left_angle_increment = 0.0
	
	# right sonar feedback
        self.right_ranges = []

	# Right sonar min angle
        self.right_angle_min = 0.0

	# Right sonar max angle
        self.right_angle_max = 0.0

	# Right sonar angle increment
        self.right_angle_increment = 0.0
	
	#Flag indicating boundary following or moving to goal() 
        self.OA_MODE = False

        #Flag indicating navigation task completion 
        self.NAV_MODE = True

        #Flag indicating collsision 
        self.COLLISION = False

        #Flag indicating initialization of the modules 
        self.INITIALIZED = False
        #Flag indicating initialization of the front sonar
        self.FRONT_SONAR_INITIALIZED = False
        #Flag indicating initialization of the left sonar
        self.LEFT_SONAR_INITIALIZED = False
        #Flag indicating initialization of the right sonar
        self.RIGHT_SONAR_INITIALIZED = False
        

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

           
    def sonar_front_callback(self, msg):
        """Handle front sonar callback"""
        # rospy.logdebug("[ROV_NAV] : --------------- front sonar_callback start ------------------")
        self.front_ranges = np.array(msg.ranges)

 	if not self.FRONT_SONAR_INITIALIZED :
	    # Front sonar min angle
            self.front_angle_min = msg.angle_min

	    # Front sonar max angle
            self.front_angle_max = msg.angle_max

	    # Front sonar angle increment
            self.front_angle_increment = msg.angle_increment

	    self.front_mid_index = int(np.round(np.fabs(self.front_angle_min)/self.front_angle_increment))

            self.FRONT_SONAR_INITIALIZED = True
 
    def sonar_left_callback(self, msg):
        """Handle left sonar callback"""
        # rospy.logdebug("[ROV_NAV] : --------------- left sonar_callback start ------------------")
        self.left_ranges = np.array(msg.ranges)
 	
 	if not self.LEFT_SONAR_INITIALIZED :
    	    # Left sonar min angle
            self.left_angle_min = msg.angle_min

	    # Left sonar max angle
            self.left_angle_max = msg.angle_max

	    # Left sonar angle increment
            self.left_angle_increment = msg.angle_increment
            
	    self.LEFT_SONAR_INITIALIZED = True

    def sonar_right_callback(self, msg):
        """Handle right sonar callback"""
        # rospy.logdebug("[ROV_NAV] : --------------- right sonar_callback start ------------------")
        self.right_ranges = np.array(msg.ranges)
 	if not self.RIGHT_SONAR_INITIALIZED :
 	
  	    # Right sonar min angle
            self.right_angle_min = msg.angle_min

	    # Right sonar max angle
            self.right_angle_max = msg.angle_max

	    # Right sonar min angle
            self.right_angle_increment = msg.angle_increment

            self.RIGHT_SONAR_INITIALIZED = True
           
    def _calculate_waypoint(self):
        rospy.logdebug("[ROV_NAV] : ------------------- start of _calculate_waypoint --------------------")
        rospy.logdebug("Current position of the vehicle is %s", self.local_pos)
	if not self.INITIALIZED :
	    if (self.FRONT_SONAR_INITIALIZED and self.LEFT_SONAR_INITIALIZED and self.RIGHT_SONAR_INITIALIZED) :
		self.INITIALIZED = True
	    return

        ex = self.target_pos.x - self.local_pos.x
        ey = self.target_pos.y - self.local_pos.y
        ez = self.target_pos.z - self.local_pos.z

        self.R     = np.sqrt(ex**2 + ey**2 + ez**2)
        self.theta = np.arctan2(ey, ex)
        self.phi   = np.arctan(ez/np.sqrt(ex**2 + ey**2))
        
        euler_angles = trans.euler_from_quaternion(self.local_orientation)
        self.alpha = euler_angles[2]
        self.beta = euler_angles[1]

        self.rel_azimuth  = self.theta - self.alpha
        self.rel_azimuth  = np.arctan2(np.sin(self.rel_azimuth), np.cos(self.rel_azimuth))
        self.rel_elev = self.phi + self.beta
        #self.rel_elev = np.arctan(np.sin(self.rel_elev)/np.cos(self.rel_elev))

        
        if self.R < Rc :
            self.NAV_MODE = False
            rospy.loginfo_once("[ROV_NAV] : ---------------------- <<<<< TARGET REACHED >>>>> --------------------------------")
            rospy.loginfo_once("[ROV_NAV] : current position of the vehicle is %s", self.local_pos)
            return
        else :
            self.NAV_MODE = True

	self.front_rho = np.min(self.front_ranges)
	if (np.isinf(self.front_rho)) :
	    self.front_rho = 1000000 #Setting it to be a large number, other than inf
	min_range_index = np.argmin(self.front_ranges)
        self.front_delta = self.front_angle_min + min_range_index*self.front_angle_increment

	self.left_rho = np.min(self.left_ranges)
	if (np.isinf(self.left_rho)) :
	    self.left_rho = 1000000 #Setting it to be a large number, other than inf
	min_range_index = np.argmin(self.left_ranges)
        self.left_delta = self.left_angle_min + min_range_index*self.left_angle_increment + np.pi/2

	self.right_rho = np.min(self.right_ranges)
	if (np.isinf(self.right_rho)) :
	    self.right_rho = 1000000 #Setting it to be a large number, other than inf
	min_range_index = np.argmin(self.right_ranges)
        self.right_delta = self.right_angle_min + min_range_index*self.right_angle_increment  - np.pi/2

	rho_array = np.array([self.front_rho,self.left_rho,self.right_rho])
	delta_array = np.array([self.front_delta,self.left_delta,self.right_delta])
	self.rho = np.min(rho_array)
	if (np.isinf(self.rho)) :
	    self.rho = 1000000 #Setting it to be a large number, other than inf
	min_range_index = int(np.argmin(rho_array))
	self.delta =delta_array[min_range_index] 
 
	if(self.rho<rho_critical) :
            self.COLLISION = True
            if self.NAV_MODE :
                rospy.loginfo("[ROV_NAV] : ----  <<<***** COLLISSION **** ------")
                rospy.loginfo("[ROV_NAV] : closest distance point on the obstacle is %s away,", self.rho)
                self.NAV_MODE = False
            return
        
	# Checking the switching condition for obstacle avoidance and move to goal        
	if not self.OA_MODE:
            if(np.min(self.front_ranges)<rho_detect) : #check if obstacles in the front
                self.OA_MODE = True
	elif(self.rho>rho_detect or  np.isinf(self.rho)) : #If no obstacles around switch back to move to target
            self.OA_MODE = False
	elif(self.rel_azimuth>0) :#Goal in the left and checking left
	    front_left_rho = np.min(self.front_ranges[self.front_mid_index:]) # checking the right side data of front sonar readings
            if (front_left_rho>rho_detect and self.left_rho>rho_detect) :
		self.OA_MODE = False
        elif(self.rel_azimuth<0) : #Goal in the right and checking right
	    front_right_rho = np.min(self.front_ranges[:self.front_mid_index]) # checking the right side data of front sonar readings
            if (front_right_rho>rho_detect and self.right_rho>rho_detect) :
 	        self.OA_MODE = False
	else : # Condition to account for unexpected conditions (temp)
	    self.OA_MODE = True
        # End of switch conditions 



        if self.OA_MODE:
            #rospy.loginfo("[ROV_NAV OA MODE]")
            rospy.logdebug("[ROV_NAV OA MODE] : rho = %s, delta = %s, front_rho = %s, front_delta = %s, left_rho = %s, left_delta = %s, right_rho = %s, right_delta = %s", self.rho, self.delta,self.front_rho, self.front_delta,self.left_rho, self.left_delta,self.right_rho, self.right_delta)
	    rospy.logdebug("[ROV_NAV OA MODE] : alpha = %s, theta = %s, beta = %s, phi = %s, rel_azimuth = %s, rel_elev = %s", self.alpha*180/np.pi, self.theta*180/np.pi, self.beta*180/np.pi, self.phi*180/np.pi, self.rel_azimuth*180/np.pi, self.rel_elev*180/np.pi)
            u_alpha = k_delta*sat(-sgn(self.delta)*np.pi/2 + self.delta) + k_rho*sgn(self.delta)*sat(self.rho-rho_safe)
            u_beta  = k_beta*sat(self.rel_elev)
        else : 
            #rospy.loginfo("[ROV_NAV HOMING MODE]")
            rospy.logdebug("[ROV_NAV HOMING MODE] : rho = %s, delta = %s, front_rho = %s, front_delta = %s, left_rho = %s, left_delta = %s, right_rho = %s, right_delta = %s", self.rho, self.delta,self.front_rho, self.front_delta,self.left_rho, self.left_delta,self.right_rho, self.right_delta)
	    rospy.logdebug("[ROV_NAV HOMING MODE] : alpha = %s, theta = %s, beta = %s, phi = %s, rel_azimuth = %s, rel_elev = %s", self.alpha*180/np.pi, self.theta*180/np.pi, self.beta*180/np.pi, self.phi*180/np.pi, self.rel_azimuth*180/np.pi, self.rel_elev*180/np.pi)
            u_alpha = k_alpha*sat(self.rel_azimuth)
            u_beta  = k_beta*sat(self.rel_elev)

        rospy.logdebug("[ROV_NAV] : u_alpha = %s, u_beta = %s", u_alpha,u_beta)

        
        t_span = np.linspace(0, 0.5, 10000)
        x0 = [self.local_pos.x, self.local_pos.y, self.local_pos.z, self.alpha, self.beta]
        sol = solve_ivp(lambda t, x: dubins3D(t,x,u_alpha,u_beta),[t_span[0],t_span[-1]],x0, t_eval = t_span, rtol = 1e-5)
        
        # Updating the desired/commanded position
        self.des_pose.pose.position.x = sol.y[0,-1]
        self.des_pose.pose.position.y = sol.y[1,-1]
        self.des_pose.pose.position.z = sol.y[2,-1]
        quaternion = tf.transformations.quaternion_from_euler(0, sol.y[4,-1], sol.y[3,-1])
        self.des_pose.pose.orientation.x = quaternion[0]
        self.des_pose.pose.orientation.y = quaternion[1]
        self.des_pose.pose.orientation.z = quaternion[2]
        self.des_pose.pose.orientation.w = quaternion[3]

        self.prev_rel_azimuth = self.rel_azimuth

        return

if __name__ == '__main__':
    rospy.loginfo("[ROV_NAV] : ----------------pos_control ---------------")
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    nav = RovNavigation()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if nav.COLLISION :
            rospy.loginfo_once("[ROV_NAV] : Collisison Detected. distance to obstacle reduced to %s", nav.rho)
            continue
        nav._calculate_waypoint()
        rospy.logdebug("[ROV_NAV] : Value of commanded position is %s", nav.des_pose)
        nav._output_pub.publish(nav.des_pose)
        rate.sleep()

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
