# from importlib import import_module
from robomaster import robot
from robomaster import camera
from robomaster import config
from robomaster import led
from robomaster import armor
from robomaster import blaster
import numpy as np

import time
from udp_client import UDPClient
from udp_server import UDPServer
import threading
import cv2
import ctypes
import sys
ll = ctypes.cdll.LoadLibrary
lib = ll("/home/nvidia/COG_ep/ep_base/libopencv_example.so")
lib.dump_.restype = ctypes.c_int

class EPRobotBase:
    def __init__(self):
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type='rndis')
        self.ep_robot.set_robot_mode(mode='chassis_lead')
        self.ep_chassis = self.ep_robot.chassis
        self.ep_gimbal = self.ep_robot.gimbal

        self.timestep = 0.04
        self.laser_num = 61
        self.vector_num = 28

        self.ep_camera = self.ep_robot.camera
        self.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P) # 读取视频流
        self.img = None
        for _ in range(10):
            self.img = self.ep_camera.read_cv2_image()

        self.remain_hp = 800
        self.remain_bullet = 24
        self.enemy_pose = [0.0, 0.0, 0.0]
        self.enemy_activated = False
        self.enemy_remain_hp = 800
        self.enemy_remain_bullet = 24
        self.goals = [[0.0, 0.0, False], [0.0, 0.0, False], [0.0, 0.0, False], [0.0, 0.0, False], [0.0, 0.0, False]]
        self.total_collision_time = 0.0
        self.taken_time = 0.0
        self.last_time = time.time()
        self.last_recv_msg_time = time.time()
        self.last_recv_msg_enemy_time = time.time()
        self.shoot_time = time.time()
        self.agent_action = [0.0, 0.0, 0.0, False]
        self.action_load = False

        # send msg
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        

        # recv msg
        self.odom_in_map_x = 0.0
        self.odom_in_map_y = 0.0
        self.odom_in_map_yaw = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.laser = None
        self.collision_info = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_w = 0.0

        self.shutdown = False

        self.render = False

        self.ep_chassis.sub_imu(freq=50, callback=self.imu_callback)
        self.ep_chassis.sub_position(cs=1, freq=50, callback=self.odom_callback)
        self.ep_chassis.sub_attitude(freq=50, callback=self.angle_callback)
        self.ep_chassis.sub_velocity(freq=50, callback=self.velocity_callback)

        self.ep_armor = self.ep_robot.armor
        self.ep_led = self.ep_robot.led
        self.ep_led.set_led(comp='all', r=0, g=0, b=255, effect='on', freq=1)
        self.hit_cnt = 0
        self.ep_armor.set_hit_sensitivity(comp='all', sensitivity=5) # 设置灵敏度
        self.ep_armor.sub_ir_event(callback=self.hit_callback)
        self.ep_blaster = self.ep_robot.blaster
        
        self.udp_send_handler = UDPServer(port=10000)
        self.udp_recv_handler = UDPClient(port=20000)

        self.udp_sendto_enemy_handler = UDPServer(port=10001, ip="192.168.1.57")
        self.udp_recvfrom_enemy_handler = UDPClient(port=20001, ip="0.0.0.0")

        self.send_msg_thread = threading.Thread(target=self.sendto_ros_msg, args=())
        self.send_msg_thread.daemon = True
        self.send_msg_thread.start()
        self.recv_msg_thread = threading.Thread(target=self.recvfrom_ros_msg, args=())
        self.recv_msg_thread.daemon = True
        self.recv_msg_thread.start()
        self.sendto_enemy_thread = threading.Thread(target=self.sendto_enemy_msg, args=())
        self.sendto_enemy_thread.daemon = True
        self.sendto_enemy_thread.start()
        self.recvfrom_enemy_thread = threading.Thread(target=self.recvfrom_enemy_msg, args=())
        self.recvfrom_enemy_thread.daemon = True
        self.recvfrom_enemy_thread.start()
        self.get_img_thread = threading.Thread(target=self.get_img, args=())
        self.get_img_thread.start()
        self.agent_control_thread = threading.Thread(target=self.control_thread, args=())
        self.agent_control_thread.start()

    def sendto_ros_msg(self):
        while True:
            if self.shutdown:
                break
            goals = []
            for g in self.goals:
                goals.append([g[0], g[1]])
            msg = {"time": time.time(), "odom_x": self.odom_x, "odom_y": self.odom_y, 
                    "odom_yaw": self.odom_yaw, "vx": self.vx, "vy": self.vy, "vw": self.vw,
                    "goals": goals}
            self.udp_send_handler.send(msg)
            time.sleep(0.02)

    def recvfrom_ros_msg(self):
        while True:
            if self.shutdown:
                break
            recv_msg = self.udp_recv_handler.recv()
            if recv_msg is None:
                continue
            self.odom_in_map_x = recv_msg["odom_in_map_x"]
            self.odom_in_map_y = recv_msg["odom_in_map_y"]
            self.odom_in_map_yaw = recv_msg["odom_in_map_yaw"]
            # print("received odom in map: ", self.odom_in_map_x, self.odom_in_map_y, self.odom_in_map_yaw)
            self.laser = recv_msg["laser"]
            self.laser.reverse()
            self.collision_info = recv_msg["collision_info"]
            # print(self.laser)
            self.vel_x = recv_msg["vel_x"]
            self.vel_y = recv_msg["vel_y"]
            self.vel_w = recv_msg["vel_w"]
            # print("recv vel: ", self.vel_x, self.vel_y, self.vel_w)

            current_time = time.time()
            if self.collision_info > 0.5:
                self.total_collision_time += current_time - self.last_recv_msg_time
            self.last_recv_msg_time = current_time
    
    def sendto_enemy_msg(self):
        while True:
            if self.shutdown:
                break
            activated_num = 0
            for g in self.goals:
                if g[-1]:
                    activated_num += 1
            msg = {"time": time.time(), "robot_x": self.pose_x, "robot_y": self.pose_y, "robot_yaw": self.pose_yaw,
                    "remain_hp": self.remain_hp,
                    "remain_bullet": self.remain_bullet, "activated_num": activated_num}
            self.udp_sendto_enemy_handler.send(msg)
            time.sleep(0.02)
    
    def recvfrom_enemy_msg(self):
        while True:
            if self.shutdown:
                break
            activated_num = 0
            for g in self.goals:
                if g[-1]:
                    activated_num += 1
            recv_msg = self.udp_recvfrom_enemy_handler.recv()
            if recv_msg is None:
                continue
            self.enemy_pose[0] = recv_msg["robot_x"]
            self.enemy_pose[1] = recv_msg["robot_y"]
            self.enemy_pose[2] = recv_msg["robot_yaw"]
            self.enemy_remain_hp = recv_msg["remain_hp"]
            self.enemy_remain_bullet = recv_msg["remain_bullet"]
            self.enemy_activated = activated_num >= 5
            self.last_recv_msg_enemy_time = time.time()
    
    
    def reset(self, goals):
        # 1. should check enemy connection
        # 2. should check localization system
        print("Check connection to ROS ...")
        while True:
            pose_ready = (self.pose_x is not None) and (self.pose_y is not None) and (self.pose_yaw is not None)
            laser_ready = (self.laser is not None)
            if pose_ready and laser_ready:
                break
        print("Connected to ROS!")

        self.taken_time = 0.0
        self.total_collision_time = 0.0
        self.init_goals(goals)
        self.init_enemy()
        self.last_time = time.time()
        self.last_recv_msg_time = time.time()
        self.last_recv_msg_enemy_time = time.time()
        self.shoot_time = time.time() - 1.0
        self.agent_action = [0.0, 0.0, 0.0, False]
        # self.hit_cnt = 0
        self.action_load = False

        obs = self.get_observation()
        return obs
        

    def get_observation(self):
        if self.img is not None:
            image_data = np.asarray(self.img, dtype=np.uint8)
            image_data = image_data.ctypes.data_as(ctypes.c_void_p)
            value = lib.dump_(0,self.img.shape[0], self.img.shape[1], image_data)
            # print("value is {}".format(value))
            # img = cv2.resize(self.img, (300, 300))
            img = self.img[30:330,110:410] 
        else:
            img = None
        # vector_state = [[pose_x, pose_y, pose_yaw], [remain_hp, remain_bullet], enemy_act(Bool), 
        # [enemy_x, enemy_y, enemy_yaw], [remain_hp, remain_bullet], 
        # goal1[x, y, activated], goal2, goal3, goal4, goal5,
        # collision_info[collision_times, continuous collision time]]
        if self.pose_yaw > np.pi:
            self.pose_yaw -= 2 * np.pi 
        if self.pose_yaw < -np.pi:
            self.pose_yaw += 2 * np.pi
        self_pose = [self.pose_x, self.pose_y, self.pose_yaw]
        self_info = [self.remain_hp, self.remain_bullet]
        enemy_act = self.enemy_activated
        enemy_pose = self.enemy_pose
        enemy_info = [self.enemy_remain_hp, self.enemy_remain_bullet]
        goal1 = self.goals[0]
        goal2 = self.goals[1]
        goal3 = self.goals[2]
        goal4 = self.goals[3]
        goal5 = self.goals[4]
        collision_info = [0, self.total_collision_time]
        odom = [self.odom_x, self.odom_y, self.odom_yaw]
        velocity = [self.vx, self.vy, self.vw]
        vector_state = [self_pose, self_info, enemy_act, enemy_pose, enemy_info,
                        goal1, goal2, goal3, goal4, goal5, collision_info, velocity, odom]
        obs = {"color_image": img, "laser": self.laser, "vector":vector_state}
        return obs
    
    def get_info(self):
        info = []
        return info

    def get_reward(self):
        return 0.0
    
    def get_game_state(self):
        done = False
        if self.taken_time >= 180 or self.remain_hp <=0 or self.enemy_remain_hp <=0:
            print("taken time: {}, remain_hp: {}, enemy_remain_hp: {}".format(self.taken_time, self.remain_hp, self.enemy_remain_hp))
            done = True
        else:
            if self.enemy_remain_hp < 800 and (time.time() - self.last_recv_msg_enemy_time) >= 5.5:
                # done = True
                # print("Connection to enemy timeout !!!! stop !!!")
                done = False # for navigation test
            else:
                done = False
        return done
    
    def shoot(self, current_time):
        if (current_time - self.shoot_time) >= 1.0 and self.remain_bullet > 0:
            self.ep_blaster.fire(fire_type=blaster.INFRARED_FIRE)
            self.remain_bullet -= 1
            self.shoot_time = current_time

    
    def step_once(self, action):
        # clip_action = self.clip_action(action)
        clip_action = action
    
        self.ep_chassis.drive_speed(x=clip_action[0], y=-clip_action[1], z=-clip_action[2] * 180.0 / np.pi, timeout=1)
        if action[-1]:
            self.shoot(time.time())
        time.sleep(self.timestep)
        self.action_load = True

        # self.ep_gimbal.recenter()
        # update state
        # ----------------------------------------
        self.check_achieved_goal()

        # ----------------------------------------

        
    
    def auto_control(self):
        action = [self.vel_x, self.vel_y, self.vel_w, 0]

        self.agent_action = action
        while not self.action_load:
            time.sleep(0.01)
        self.action_load = False
        # self.step_once(action)
        # obs = self.get_observation()

        obs = self.get_observation()
        reward = self.get_reward()
        done = self.get_game_state()
        
        current_time = time.time()
        self.taken_time += current_time - self.last_time
        self.last_time = current_time
        time_taken = self.taken_time
        dmg = self.get_attack_damage()
        flag_ach = self.get_flag_achieved()
        score = self.get_score(flag_ach, self.remain_hp, dmg, self.taken_time, self.total_collision_time)

        judge_result = [score, time_taken, dmg, flag_ach]
        return obs, reward, done, [action, judge_result]
    
    def step(self, action):
        self.agent_action = action
        while not self.action_load:
            time.sleep(0.01)
        self.action_load = False
        # time.sleep(self.timestep)

        obs = self.get_observation()
        reward = self.get_reward()
        done = self.get_game_state()
        
        current_time = time.time()
        self.taken_time += current_time - self.last_time
        self.last_time = current_time
        time_taken = self.taken_time
        dmg = self.get_attack_damage()
        flag_ach = self.get_flag_achieved()
        score = self.get_score(flag_ach, self.remain_hp, dmg, self.taken_time, self.total_collision_time)

        judge_result = [score, time_taken, dmg, flag_ach]
        return obs, reward, done, [[], judge_result]
    

    # -------------------------- useful function -----------------------
    def clip_action(self, action):
        action[0] = np.clip(action[0], -1, 1)
        action[1] = np.clip(action[1], -1, 1)
        action[2] = np.clip(action[2], -np.pi/4, np.pi/4)

        return action
    

    def odom_callback(self, sub_info):
        odom_x, odom_y, _ = sub_info
        self.odom_x = odom_x
        self.odom_y = -odom_y
        self.pose_x = self.odom_in_map_x + self.odom_x * np.cos(self.odom_in_map_yaw) - self.odom_y * np.sin(self.odom_in_map_yaw)
        self.pose_y = self.odom_in_map_y + self.odom_x * np.sin(self.odom_in_map_yaw) + self.odom_y * np.cos(self.odom_in_map_yaw)
        # print("odom: ", self.odom_x, self.odom_y)
 
    
    def angle_callback(self, sub_info):
        yaw, _, _ = sub_info
        self.odom_yaw = -np.pi * yaw / 180.0
        self.pose_yaw = self.odom_in_map_yaw + self.odom_yaw
        # print("yaw: ", yaw)

    
    def velocity_callback(self, sub_info):
        _, _, _, vbx, vby, _ = sub_info
        self.vx = vbx
        self.vy = -vby
        # self.vw = vbz
        # print("vel: ", vbx, vby)
    
    def imu_callback(self, sub_info):
        acc_x, acc_y, _, _, _, gyro_z = sub_info
        self.vw = -gyro_z
        # print("acc: ", acc_x, acc_y)
        # print("gyro_z: ", gyro_z)
    
    def hit_callback(self, sub_info):
        hit_cnt = sub_info
        delta_hit = hit_cnt - self.hit_cnt
        if delta_hit>0:
            self.remain_hp -= delta_hit * 100
        self.hit_cnt = hit_cnt
        print("hit event: hit_cnt:{0}".format(self.hit_cnt))
        # [0~7]
        leds = []
        if hit_cnt < 8:
            for item in range(7, hit_cnt-1, -1):
                leds.append(item)
        else:
            leds = []
        self.ep_led.set_gimbal_led(comp='top_all', r=0, g=0, b=255, led_list=leds, effect='on')
    
    def get_img(self):
        while True:
            if self.shutdown:
                break
            self.img = self.ep_camera.read_cv2_image()
            if self.render:
                cv2.imshow("camera image", self.img)
            time.sleep(0.01)
    
    def init_enemy(self):
        self.enemy_pose = [0.0, 0.0, 0.0]
        self.enemy_activated = False
        self.enemy_remain_hp = 800
        self.enemy_remain_bullet = 24
    
    def init_goals(self, goals):
        for i in range(len(goals)):
            self.goals[i][0] = goals[i][0]
            self.goals[i][1] = goals[i][1]
            self.goals[i][2] = False # False  True is for test confrontation
    
    def check_achieved_goal(self):
        for i in range(len(self.goals)):
            if self.goals[i][-1]:
                continue
            else:
                goal_x = self.goals[i][0]
                goal_y = self.goals[i][1]
                dist = np.sqrt((goal_x-self.pose_x)**2 + (goal_y-self.pose_y)**2)
                delta_yaw = np.arctan2((goal_y-self.pose_y), (goal_x-self.pose_x)) - self.pose_yaw
                if(delta_yaw < -np.pi):
                    delta_yaw += 2*np.pi
                if(delta_yaw>np.pi):
                    delta_yaw -= 2*np.pi
                if (dist <= 1.0) and (np.fabs(delta_yaw)<np.pi/6):
                    self.goals[i][-1] = True
                # if self.laser is not None:
                #     laser_index = int((delta_yaw - (-0.75 * np.pi)) / (4.5 * np.pi / 180.0))
                #     if laser_index < 0 or laser_index >= self.laser_num:
                #         self.goals[i][-1] = False
                #     else:
                #         if self.laser[laser_index] <= (dist-0.3):
                #             self.goals[i][-1] = False
                    
                # if self.laser is not None:
                #     laser_dist = self.laser[30]
                #     if laser_dist <
                break
    
    def get_flag_achieved(self):
        num = 0
        for g in self.goals:
            if g[-1]:
                num += 1
        return num
    
    def get_attack_damage(self):
        return (800 - self.enemy_remain_hp)
    
    def get_score(self, num_ach_goals, remain_hp, attach_dmg, taken_time, collision_time):
        a = 0.0
        if num_ach_goals >4:
            a = 1.0
        # score = float(num_ach_goals) * 60 + a * (remain_hp + attach_dmg) - taken_time - 10 * 2 * collision_time
        score = float(num_ach_goals) * 60 + a * 0.5 * (remain_hp + attach_dmg) - taken_time - 10 * 2 * collision_time
        return score
    

    def control_thread(self):
        while True:
            if self.shutdown:
                break
            if (time.time() - self.last_time) > 0.5:
                self.agent_action = [0.0, 0.0, 0.0, False]
            self.step_once(self.agent_action)
    
    
    
    def stop_robot(self):
        print("stop the robot!")
        self.ep_chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=1)
    
    def close(self):
        self.shutdown = True
        self.get_img_thread.join()
        self.agent_control_thread.join()
        # self.send_msg_thread.join()
        # self.recv_msg_thread.join()
        self.stop_robot()
        self.ep_robot.close()
        self.udp_send_handler.close()
        self.udp_recv_handler.close()
        self.udp_sendto_enemy_handler.close()
        self.udp_recvfrom_enemy_handler.close()

    
import signal
import sys
if __name__ == "__main__":
    ep_env = EPRobotBase()
    stop = False
    def breakdown(e1, e2):
        global stop
        ep_env.close()
        print("closed ep_env")
        stop = True
    # time.sleep(5.0)
    signal.signal(signal.SIGINT, breakdown)
    while True:
        if stop:
            break
        else:
            ep_env.auto_control()
    sys.exit()
    # ep_env.close()
