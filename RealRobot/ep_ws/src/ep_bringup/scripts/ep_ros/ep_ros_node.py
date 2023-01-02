#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from roborts_msgs.msg import TwistAccel
from roborts_msgs.msg import Cost
from roborts_msgs.msg import SentryTarget
from roborts_msgs.msg import SentryTargetArray
import tf
import time
from udp_client import UDPClient
from udp_server import UDPServer
import threading
import numpy as np
class EPRobotROS:
    def __init__(self):
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.laser_data = [0.0]*61
        self.collision_info = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_w = 0.0
        self.vel_time = time.time()
        self.shutdown = False
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.goals_pub = rospy.Publisher("/sentry_array", SentryTargetArray, queue_size=10)
        self.odom_tf_br = tf.TransformBroadcaster()
        self.pose_tf_listener = tf.TransformListener()
        self.laser_sub = rospy.Subscriber("sample_scan", LaserScan, self.laser_callback)
        self.cmd_vel_acc_callback = rospy.Subscriber("cmd_vel_acc", TwistAccel, self.cmd_vel_acc_callback)
        self.robo_cost_callback = rospy.Subscriber("self_cost", Cost, self.cost_msg_callback)
        self.udp_send_handler = UDPServer(port=20000)
        self.udp_recv_handler = UDPClient(port=10000)

        self.send_msg_thread = threading.Thread(target=self.sendto_EP_msg, args=())
        self.send_msg_thread.start()
        self.recv_msg_thread = threading.Thread(target=self.recvfrom_EP_msg, args=())
        self.recv_msg_thread.daemon = True
        self.recv_msg_thread.start()
        

    def laser_callback(self, data):
        laser = data.ranges
        clip_laser = []
        for l in laser:
            lf = float(l)
            if lf>15.0:
                lf = 15.0
            clip_laser.append(lf)
        self.laser_data = clip_laser
       
    
    def cmd_vel_acc_callback(self, data):
        self.vel_x = data.twist.linear.x
        self.vel_y = data.twist.linear.y
        self.vel_w = data.twist.angular.z
        self.vel_time = time.time()
    
    def cost_msg_callback(self, data):
        cost = data.value
        if cost > 200:
            self.collision_info = 1.0
        else:
            self.collision_info = 0.0
        
    def pose_lookup(self):
        try:
            (trans,rot) = self.pose_tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
            self.pose_x = trans[0]
            self.pose_y = trans[1]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            print("lookup: ", self.pose_x, self.pose_y, yaw)
            self.pose_yaw = yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Cannot lookup transform between /map and /odom")

    def odom_tf_msg_pub(self, odom_x, odom_y, odom_yaw, vx, vy, vw):
        self.odom_tf_br.sendTransform((odom_x, odom_y, 0.0), 
                                      (tf.transformations.quaternion_from_euler(0, 0, odom_yaw)),
                                      rospy.Time.now(), "base_link", "odom")
        odom = Odometry()
        odom.child_frame_id = "odom"
        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, odom_yaw)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vw
        self.odom_pub.publish(odom)
    
    def goals_publish(self, goals):
        goals_array = SentryTargetArray()
        i = 0
        for g in goals:
            i += 1
            if np.sqrt(g[0]**2+g[1]**2)<0.1:
                continue
            g_t = SentryTarget()
            g_t.id = 0
            g_t.x = g[0]
            g_t.y = g[1]
            g_t.yaw = 0.0
            goals_array.targets.append(g_t)
        self.goals_pub.publish(goals_array)
    
    def sendto_EP_msg(self):
        while True:
            if self.shutdown:
                break
            self.pose_lookup()
            current_time = time.time()
            delta_time_to_last_vel = current_time - self.vel_time
            vel_x = self.vel_x
            vel_y = self.vel_y
            vel_w = self.vel_w
            if(delta_time_to_last_vel>0.5):
                print("vel is old, last delta time ", delta_time_to_last_vel)
                vel_x = 0.0
                vel_y = 0.0
                vel_w = 0.0
            msg = {"time": current_time, "odom_in_map_x": self.pose_x, "odom_in_map_y": self.pose_y, 
                "odom_in_map_yaw": self.pose_yaw,
                "laser": self.laser_data, "collision_info": self.collision_info, 
                "vel_x": vel_x, "vel_y": vel_y, "vel_w": vel_w}
            self.udp_send_handler.send(msg)
            time.sleep(0.02)
    
    def recvfrom_EP_msg(self):
        while True:
            if self.shutdown:
                break
            recv_msg = self.udp_recv_handler.recv()
            odom_x = recv_msg["odom_x"]
            odom_y = recv_msg["odom_y"]
            odom_yaw = recv_msg["odom_yaw"]
            vx = recv_msg["vx"]
            vy = recv_msg["vy"]
            vw = recv_msg["vw"]
            goals = recv_msg["goals"]
            self.odom_tf_msg_pub(odom_x, odom_y, odom_yaw, vx, vy, vw)
            self.goals_publish(goals)
    
    def close(self):
        self.shutdown = True
        self.udp_send_handler.close()
        self.udp_recv_handler.close()

import sys
if __name__ == "__main__":
    rospy.init_node("ep_ros_node")
    ep_ros_base = EPRobotROS()
    print("ep_ros_node started!")
    while True:
        if rospy.is_shutdown():
            ep_ros_base.close()
            break
        time.sleep(0.01)
    sys.exit()