#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np
from . import behavior_gates as bg




class BehaviorGatesCollisionAvoidance(Node):

    def __init__(self):
        super().__init__('behavior_gates_collision_avoidance')

        # https://github.com/ros2/ros2cli/blob/master/ros2param/ros2param/api/__init__.py
        self.declare_parameter('cmd_vel_topic', 'cmd_vel') # default parameter value: 'cmd_vel'
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.v = 0.0
        self.omega = 0.0

        self.subscription = self.create_subscription(
            Odometry,
            'camera/odom/sample', # T265 realsense camera
            self.odom_callback,
            10)

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)

    def odom_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z


    def laser_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment) # sick laser scanner
        force = self.calculate_force(angles, ranges)
        v_ca = Twist()
        v_ca.linear.x  = force[0]
        v_ca.angular.z = force[1]
        self.publisher_.publish(v_ca)

    def calculate_force(self,angles,ranges):
        force = np.zeros(2)
        force[0]=0.1

        #force calculation using analogical gates
        angular_force_contrib=np.zeros(len(angles))
        linear_force_contrib=np.zeros(len(angles))

        # TODO: replace for loop (?) -> use numpy in normaliser()
        # angles = np.where(angles>=np.pi, angles-2*np.pi, angles)
        # ranges = np.where(ranges==0, 10000, angles)

        for i in range(len(angles)):
            if angles[i] >= np.pi:
                angles[i]=angles[i]-2*np.pi
            if ranges[i] == 0:
                ranges[i]=10000
            normalised_range=normaliser(ranges[i],0.4,0.07)
            normalised_angle=sigmoid(angles[i],3,0,symmetric=True)

            angular_force_contrib[i]= bg.AND(bg.AMP(bg.SNOT(normalised_range),-1*bg.SNOT(normalised_angle)),-1*bg.SNOT(normalised_angle))
            linear_force_contrib[i]= bg.AND(normalised_range,bg.SNOT(normalised_angle))

        force[1]=sigmoid(np.sum(angular_force_contrib), 10, 0, True)
        force[0]= sigmoid(np.sum(linear_force_contrib), 1, 0) * 0.1
        return force


def normaliser(x,max_val,min_val):
    if x >= max_val:
        return 1
    if x <= min_val:
        return 0
    else:
        return (x-min_val)/(max_val-min_val)

def sigmoid(x,steepness,midpoint,symmetric=False):
    if symmetric== True:
        return 2/(1+np.exp(-steepness*(x-midpoint)))-1
    else:
        return 1/(1+np.exp(-steepness*(x-midpoint)))






def main(args=None):
    rclpy.init(args=args)
    
    beh_gates_ca = BehaviorGatesCollisionAvoidance()
    rclpy.spin(beh_gates_ca)
    beh_gates_ca.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
