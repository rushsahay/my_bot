#!/usr/bin/env python3

import numpy as np
import array
import math
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion

import time

topic1 = "/cmd_vel"
topic2 = "/odom"
topic3 = "/scan"

class ControllerNode(Node):

    #Self: Constructor for class
    # (xdu, ydu): Position of setpoint
    # kau: Attractive force parameter
    # kru: Repulsive force parameter
    # kthetau: kP for theta controller
    # gstaru: Maximum distance to object where repulsive force is calculated
    # eps_orientu: Get inside epsilon angle threshold before moving robot linearly
    # eps_controlu: If inside a certain threshold of setpoint then stop movement from controller
    def __init__(self, xdu, ydu, kau, kru, kthetau, gstaru, eps_orientu, eps_controlu):
        super().__init__("Controller_Node")

        self.xdu = xdu
        self.ydu = ydu

        self.kau = kau
        self.kru = kru

        self.kthetau = kthetau

        self.gstaru = gstaru
        self.eps_orientu = eps_orientu
        self.eps_controlu = eps_controlu

        #Instantiate messages
        self.OdometryMsg = Odometry()
        self.ScanMsg = LaserScan()

        #Instantiate time for all messages and sim time
        self.initTime = time.time()
        self.msgOdometryTime = time.time()
        self.msgScanTime = time.time()

        #Instantiate twist to send velocities
        self.controlVel = Twist()

        #Set components of controlVel all to 0
        self.controlVel.linear.x = 0.0
        self.controlVel.linear.y = 0.0
        self.controlVel.linear.z = 0.0

        self.controlVel.angular.x = 0.0
        self.controlVel.angular.y = 0.0
        self.controlVel.angular.z = 0.0

        #Create publisher to put cmdVel. Pass in Type of input, topic, and buffer size
        self.ControlPublisher = self.create_publisher(Twist,
                                                      topic1,
                                                      10)
        
        #Create subscriber for odometry. Type, topic, and buffer size. Method describes what will happen when odometry message is recieved
        self.OdometrySubscriber = self.create_subscription(Odometry,
                                                           topic2,
                                                           self.SensorCallbackPose,
                                                           10)
        #Create subscriber for Lidar. Type, topic, and buffer size. Method describes what will happen when lidar message is recieved
        self.ScanSubscriber = self.create_subscription(LaserScan,
                                                       topic3,
                                                       self.SensorCallbackLidar,
                                                       10)

        #Specify period of messages. Every message sent at 0.05 seconds. 20msg per second
        self.period = 0.05
        
        self.timer = self.create_timer(self.period,self.ControlFunction)

    def orientationE(self, theta_, thetaD_):
        if(thetaD_>np.pi/2) and (thetaD_<=np.pi):
            if(theta_>-np.pi) and (theta_<= -np.pi/2):
                theta_=theta_+(np.pi*2)
        
        if(theta_>np.pi/2) and (theta_<=np.pi):
            if(thetaD_>-np.pi) and (thetaD_<= -np.pi/2):
                thetaD_= thetaD_+(np.pi*2)

        # if(thetaD_<0):
        #     thetaD_ = thetaD_+2*np.pi
        # if(theta_<0):
        #     theta_ = theta_+2*np.pi
        
        orientationError = thetaD_-theta_
        return orientationError
    
    def SensorCallbackPose(self, recievedMsg):
        #Odom message stored in self topic 
        self.OdometryMsg = recievedMsg
        #Time also stored into the topic
        self.msgOdometryTime = time.time()

    def SensorCallbackLidar(self, recievedMsg):
        #Scan message stored in self topic 
        self.ScanMsg = recievedMsg
        #Time also stored into the topic
        self.msgScanTime = time.time()

    #Runs periodically at 0.05 seconds because it is in the timer
    def ControlFunction(self):
        #Attractive and repulsive forces 
        ka = self.kau
        kr = self.kru

        #P conntrol and threshold points extracted
        ktheta = self.kthetau
        gstar = self.gstaru

        #Coordinates of the goal point
        kxd = self.xdu
        kyd = self.ydu

        #Current position and orientation
        x = self.OdometryMsg.pose.pose.position.x
        y = self.OdometryMsg.pose.pose.position.y
        
        #Extract quaternion object
        quat = self.OdometryMsg.pose.pose.orientation

        #Get all of the quaternion components out of object
        quatl = [quat.x,quat.y,quat.z,quat.w]

        (roll, pitch, yaw) = euler_from_quaternion(quatl)

        #Angle stored in radians and 0-180 = [0, pi] and 180-360 = [-pi, 0]
        theta = yaw

        LidarRanges = np.array(self.ScanMsg.ranges)

        #Minimum angle of Lidar measurements(Defined in description file)
        min_angle = self.ScanMsg.angle_min

        #Angle increment: What is the angular distance between rays of Lidar
        angle_increment = self.ScanMsg.angle_increment

        vectorD = np.array([[x-kxd],[y-kyd]])
        graduA = ka*vectorD
        AF = -graduA

        indicies_not_inf = np.where(~np.isinf(LidarRanges))[0]

        obstacleYES = ~np.all(np.isinf(indicies_not_inf))

        if(obstacleYES):
            #Detect if there's a difference more than 1 in the indecies
            diff = np.diff(indicies_not_inf)

            # Add one to get the split points in the original array
            split_indicies = np.where(np.abs(diff)>1)[0]+1

            #Array with subarrays where every new subarray are indecies of an object
            partitioned_array = np.split(indicies_not_inf,split_indicies)

            #Gets the absolute angle for all lidar measurements that have data
            angles = min_angle+indicies_not_inf*angle_increment+theta

            #Done for debugging
            distances = LidarRanges[indicies_not_inf]

            x0 = x*np.ones(distances.shape)+distances*np.cos(angles)
            y0 = y*np.ones(distances.shape)+distances*np.sin(angles)

            #List of minimum distances and their corresponding angles
            min_distances = []
            min_distances_angles = []

            for i in range(len(partitioned_array)):
                tmpArray = LidarRanges[partitioned_array[i]]
                min_index = np.argmin(tmpArray)
                min_distances.append(min(tmpArray))
                min_distances_angles.append(min_angle+angle_increment*partitioned_array[i][min_index])
        
            x0min = []
            y0min = []

            for i in range(len(min_distances)):
                x0min.append(x+min_distances[i]*np.cos(min_distances_angles[i]+theta))
                y0min.append(y+min_distances[i]*np.sin(min_distances_angles[i]+theta))

            #Compute gradient value for every obstacle
            g_values = []
            graduR = []

            for i in range(len(min_distances)):
                graduR_i = np.array([[0],[0]])
                g_val = np.sqrt((x-x0min[i])**2+(y-y0min[i])**2)
                g_values.append(g_val)

                if(g_val<=gstar):
                    pr=kr*((1/gstar)-(1/g_values[i]))*(1/((g_values[i])**3))
                    graduR_i = pr*np.array([[x-x0min[i]],[y-y0min[i]]])

                graduR.append(graduR_i)

            RF = np.array([[0],[0]])

            for i in range(len(graduR)):
                RF = RF+graduR[i]

            RF = -RF
        if(obstacleYES):
            F=AF+RF
        else:
            F=AF

        #Calculate the desired rotation for setpoint
        thetaD = math.atan2(F[1][0],F[0][0])

        eOrientation = self.orientationE(theta,thetaD)

        #Turn off controller if error smaller that tolerance
        if(np.linalg.norm(vectorD,2)<self.eps_controlu):
            thetavel = 0.0
            xvel = 0.0
        else:
            if(eOrientation>np.abs(self.eps_orientu)):
                thetavel = eOrientation*ktheta
                xvel = 0.0
            else:
                thetavel = eOrientation*ktheta
                xvel = np.linalg.norm(F,2)
            if(np.abs(xvel)>2.6):
                xvel = 2.5

        #Form Twist message you send to robot to control it
        self.controlVel.linear.x = xvel
        self.controlVel.linear.y = 0.0
        self.controlVel.linear.z = 0.0

        self.controlVel.angular.x = 0.0
        self.controlVel.angular.y = 0.0
        self.controlVel.angular.z = thetavel

        self.ControlPublisher.publish(self.controlVel)

        print("Retrieved Pose: ")
        timeDiff = self.msgOdometryTime-self.initTime

        print(f"Time,x,y,theta:({timeDiff:.3f}, {x:.3f}, {y:.3f}, {theta:.3f})")

def main(args=None):
    #Call rclpy init
    rclpy.init(args=args)
    #Set desired setpoint
    xdu = 10
    ydu = 10

    #Select control parameters
    kau = 0.6
    kru = 0.1
    kthetau = 4
    gstaru = 4

    #Set tolerances
    eps_orientu = np.pi #In radians
    eps_controlu = 0.2 #In meters

    #Create the node
    TestNode = ControllerNode(xdu, ydu, kau, kru, kthetau, gstaru, eps_orientu,eps_controlu)

    #Start the node
    rclpy.spin(TestNode)
    #Shut down and destroy node
    TestNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



        
            


