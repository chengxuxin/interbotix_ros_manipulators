from __future__ import print_function, division
from interbotix_xs_modules.arm import InterbotixManipulatorXS

# This script commands some arbitrary positions to the arm joints
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
# Then change to this directory and type 'python joint_position_control.py'
import time
import numpy as np
import sys
import rospy
import socket
import pickle
import multiprocessing
from multiprocessing import Process, Array
from collections import deque
import argparse

class ArmClient():
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock.bind((self.addr, port))
        self.sock_recv.bind(("192.168.123.15", 6677))
        self.sock_recv.settimeout(1.0)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock_send.connect((addr, port))
        self.start_flag = True
        self._actions = Array('f', np.zeros(7, dtype=np.float))
        self.receive_thread = Process(target=self.recv, args=(self._actions,))
        self.receive_thread.start()

    def recv(self, actions_global):
        while self.start_flag:
            try:
                data, addr = self.sock_recv.recvfrom(4096)
                actions = data.decode("utf-8")
                actions = actions.split(' ')
                actions_global[:] = np.array((map(float, actions)))[:]
            except KeyboardInterrupt:
                self.stop()
                exit()
    
    def send(self, joint_states):
        if np.any( np.abs(joint_states[:8])> 2.5):
            print(joint_states)
            print("exiting due to incorrect joint states")
            exit()
        data_string = ("{:.3f} "*15 + "{:.3f}").format(*joint_states)
        self.sock_send.sendto(data_string.encode("utf-8"), (self.addr, self.port))

    @property
    def actions(self):
        return np.array(self._actions[:6])

    @property
    def gripper_actions(self):
        return self._actions[-1]

    def stop(self):
        self.start_flag = False
    
def main(args):
    sock = ArmClient("192.168.123.161", 9998)
    torque2mA = 1000 * np.array([1.68/2.905]*5 + [0.9/0.78]) 
    
    # kp = np.array([2000, 2000, 2000, 100, 1000, 600], dtype=np.float)
    # kd = np.array([250, 250, 50, 0, 50, 0], dtype=np.float)#2*kp**0.5

    # kp = np.array([2000, 2000, 7000, 100, 1000, 600], dtype=np.float)
    # kd = np.array([250, 250, 80, 0, 50, 0], dtype=np.float)#2*kp**0.5

    kp = [500, 2000, 2000, 300, 300, 0]
    ki = [0, 0, 0, 0, 0, 0]
    kd = [0, 200, 200, 0, 0, 0]

    kp = [1000, 2000, 2000, 300, 300, 0]
    ki = [0, 0, 0, 0, 0, 0]
    kd = [0, 200, 200, 0, 0, 0]

    kp_torque = kp / torque2mA
    kd_torque = kd / torque2mA
    print(kp_torque)
    print(kd_torque)
    # exit()
    current_offset = np.array([60.]*5 +[60.])
    # i_error = 0
    # kp = 5 * torque2mA
    # kd = 2*kp**0.5# * torque2mA
    # print(kp, kd)
    # exit()
    # u_min = np.array([-1000, -1000, -1000, -1000, -1000, -1000])
    # u_max = np.array([1000, 1000, 1000, 1000, 1000, 1000])
    u_min = np.array([-485, -485, -585, -285, -285, -285])
    u_max = np.array([485, 485, 585, 285, 285, 285])
    #ref_joint_positions = np.array([-1.0, 0.5 , 0.5, 0, -0.5, 1.57])
    # end_ref_joint_positions = np.array([0., 0., 0., 0, 0, 0])
    start_ref_joint_positions = np.array([0]*6)
    end_ref_joint_positions = np.array([-1.57, 0. , 0., 0, -0., 0.])#np.array([-1.0, 0. , 0., 0, -0., 0])
    move_time = 200.0
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    # bot.dxl.robot_torque_enable("group", "arm", False)
    # start_ref_joint_positions = np.array(bot.arm.group_info.joint_sleep_positions)
    bot.dxl.robot_set_operating_modes("group", "arm", "position", profile_type="time")
    bot.arm.go_to_home_pose(moving_time=3.0, accel_time=1.0)
    # bot.arm.set_ee_pose_components(x=0.4, z=0.2)
    bot.dxl.robot_set_operating_modes("group", "arm", "pwm", profile_type="time")
    
    dof_indices = []
    for name in bot.arm.group_info.joint_names:
        dof_indices.append(bot.dxl.js_index_map[name])
    
    # actions_buf = deque(maxlen=20)
    # for i in range(10):
    # 	actions_buf.append(sock.actions)
    ema = np.zeros(6)
    ema_ratio = 0.98
    actions_to_save = deque(maxlen=10000)
    r = rospy.Rate(500)
    for i in range(300000):
        try:
            ref_joint_positions = sock.actions
            if i % 10 == 0:
                ema = ema_ratio * ema + ref_joint_positions * (1.0 - ema_ratio)
            # ref_joint_positions[-3:] = 0.
            ema[-3:] = 0.
            joint_pos = np.array(bot.dxl.joint_states.position)[dof_indices]
            joint_vel = np.array(bot.dxl.joint_states.velocity)[dof_indices]
            if np.any(np.abs(joint_pos)) > 2.5:
                break
            # i_error = i_error + (ref_joint_positions - joint_pos)
            
            u = kp * (ema - joint_pos) - kd * joint_vel
            u = u.clip(2*u_min, 2*u_max)
            
            
            bot.dxl.robot_write_commands("arm", u)
            # bot.dxl.robot_write_commands("arm", ema)
            
            sock.send(np.concatenate([joint_pos, np.zeros((2)), joint_vel, np.zeros((2))]))
            
            if np.isclose(sock.gripper_actions, 1.0):
                bot.gripper.close(0.0)
            elif np.isclose(sock.gripper_actions, 2.0):
                bot.gripper.open(0.0)
            elif np.isclose(sock.gripper_actions, 9.0):
                sock.stop()
                bot.dxl.robot_torque_enable("group", "arm", False)
                exit()
            
            if i % 50 == 0:
                print("u: ", u)
                print("received actions:", sock.actions)
                print("ref_pos: ", ref_joint_positions)
                print("joint_pos:", joint_pos)
                print("gripper action:", sock.gripper_actions)
            if i % 10 == 0 and args.save_actions:
                actions_to_save.append(sock.actions)
            
        except KeyboardInterrupt or not sock.start_flag:
            print("exit")
            sock.stop()
            bot.dxl.robot_torque_enable("group", "arm", False)
            sys.exit(0)
        r.sleep()


    bot.dxl.robot_set_operating_modes("group", "arm", "position", profile_type="time")
    sock.stop()
    if args.save_actions:
        np.savetxt("received_actions.txt", actions_to_save, fmt='%.3f')
    # bot.arm.set_ee_pose_components(x=0.4, z=0.3)
    bot.arm.go_to_home_pose(moving_time=3.0, accel_time=1.0)
    bot.arm.go_to_sleep_pose(moving_time=3.0, accel_time=1.0)
    bot.dxl.robot_torque_enable("group", "arm", False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--save_actions", action='store_true')
    # parser.add_argument("--kp", type=float, required=True)
    # parser.add_argument("--kd", type=float, required=True)
    args = parser.parse_args()
    main(args)

