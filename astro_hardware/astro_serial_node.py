#!/usr/bin/env python3
import serial
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class AstroSerial(Node):
    def __init__(self):
        super().__init__("astro_serial_node")

        # qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        #     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #     depth=1
        # )

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.js_publisher = self.create_publisher(
            JointState, 
            "/joint_states",
            10
        )

        self.current_publisher = self.create_publisher(
            Float32,
            "/current",
            10
        )

        self.odom_publiher = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        self.reset_odom_srv = self.create_service(Empty, "reset_odometry", self.reset_odom_callback)
        
        period = 0.0001
        self.timer = self.create_timer(period, self.timer_callback, clock=self.get_clock())
        self.js_trimer = self.create_timer(0.02, self.publish_All, clock=self.get_clock())

        self.wheelRadius = 0.036
        self.wheelSeparation = 0.304
        self.maxLinearSpeed = 0.75
        self.maxAngularSpeed = 5.1
        self.serial = serial.Serial(port='/dev/ttyACM0', baudrate=57600, timeout=0.1)
        #self.serial.xonxoff = 1

        self.js_msg = JointState()
        self.current_msg = Float32()
        self.odom_msg = Odometry()

        self.odom_x = 0
        self.odom_y = 0
        self.odom_th = 0
        self.odom_current_time = 0
        self.odom_last_time = 0
        self.get_logger().info("Started astro_serial_node!")

    
    def sendVelocity(self, linearX, angularZ):
        w_l = (linearX / self.wheelRadius) - ((angularZ * self.wheelSeparation) / (2 * self.wheelRadius))
        w_r = (linearX / self.wheelRadius) + ((angularZ * self.wheelSeparation) / (2 * self.wheelRadius))
        if (abs(w_l) < 0.40):
            w_l = 0
        if (abs(w_r) < 0.40):
            w_r = 0
        command = f"v {w_l} {w_r}\n"

        self.get_logger().info(f"Sending linear: {linearX} angular: {angularZ} w_l: {w_l} w_r: {w_r}")
        self.serial.write(bytes(command, 'utf-8'))

    def callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.sendVelocity(linearX=linear_x, angularZ=angular_z)


    def timer_callback(self):
        if self.serial.in_waiting:
            rawJointStates = self.serial.readline().decode(errors='ignore').strip(" \n")
            # rawJointStates = "0 0 0 0 0 0 0"
            rawJointStates_list = [float(e) for e in rawJointStates.split()]

            self.js_msg.name = ["left_wheel_joint", "right_wheel_joint"]
            self.js_msg.position = rawJointStates_list[0:2]
            self.js_msg.velocity = rawJointStates_list[2:4]
            self.js_msg.effort = rawJointStates_list[4:6]
            self.js_msg.header.stamp = self.get_clock().now().to_msg()
            self.js_msg.header.frame_id = "base_link"

            # print(rawJointStates_list)

            self.current_msg.data = float(rawJointStates_list[-1])

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def publish_Odometry(self):

        self.odom_current_time = float(self.get_clock().now().nanoseconds / 1e9)

        wL = float(self.js_msg.velocity[0])
        wR = float(self.js_msg.velocity[1])

        velX = (self.wheelRadius / 2) * (wL + wR)
        velY = 0
        velTh = (self.wheelRadius/self.wheelSeparation) * (wR - wL)

        dt = self.odom_current_time - self.odom_last_time
        deltaX = velX * math.cos(self.odom_th) * dt
        deltaY = velX * math.sin(self.odom_th) * dt
        deltaTh = velTh * dt

        self.odom_x += deltaX
        self.odom_y += deltaY
        self.odom_th += deltaTh
        quat = self.quaternion_from_euler(0, 0, self.odom_th)

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"

        self.odom_msg.pose.pose.position.x = self.odom_x
        self.odom_msg.pose.pose.position.y = self.odom_y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = quat[0]
        self.odom_msg.pose.pose.orientation.y = quat[1]
        self.odom_msg.pose.pose.orientation.z = quat[2]
        self.odom_msg.pose.pose.orientation.w = quat[3]

        self.odom_msg.twist.twist.linear.x = velX
        self.odom_msg.twist.twist.angular.z = velTh

        self.odom_last_time = self.odom_current_time

        self.odom_publiher.publish(self.odom_msg)
    
    def reset_odom_callback(self, request, response):
        self.get_logger().info('Incoming request: reset_odometry')
        self.odom_x = 0
        self.odom_y = 0
        self.odom_th = 0
        self.get_logger().info('Odometry was reseted.')
        return response


    def publish_All(self):
        self.js_publisher.publish(self.js_msg) 
        self.current_publisher.publish(self.current_msg)
        self.publish_Odometry()




def main(args=None):
    rclpy.init(args=args)

    astroSerial = AstroSerial()
    rclpy.spin(astroSerial)

    astroSerial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()