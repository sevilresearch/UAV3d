#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
import math

import logging
import sys
import time
import threading
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

drone = "\"Crazyflie1\""
drone_x = 0
drone_y = 0
drone_z = 0
drone_xr = 0
drone_yr = 0
drone_zr = 0
drone_wr = 0
wand = "\"wand\""
wand_x = 0
wand_y = 0
wand_z = 0
wand_xr = 0
wand_yr = 0
wand_zr = 0
wand_wr = 0

def crazyflie():
    global drone_x
    global drone_y
    global drone_z25
    window = 0.3
    step = 0.3
    inner_circle = 1.1
    outter_circle = 2.8 * 2.8
    previous_move = ''

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf) as mc:
            time.sleep(2)
            while True:
                target = [0, -1.8]
                up = [float(drone_x), float(drone_y) + step]
                down = [float(drone_x), float(drone_y) - step]
                left = [float(drone_x) - step, float(drone_y)]
                right = [float(drone_x) + step, float(drone_y)]

                #Straight-line distance to target.
                up_distance = math.dist(up, target)
                down_distance = math.dist(down, target)
                left_distance = math.dist(left, target)
                right_distance = math.dist(right, target)
                
                #Checks to see if option is in bounds.
                verify = up[0] * up[0] + up[1] * up[1]
                if verify < inner_circle or verify > outter_circle or previous_move == 'down':
                    up_distance = 100
                verify = down[0] * down[0] + down[1] * down[1]
                if verify < inner_circle or verify > outter_circle or previous_move == 'up':
                    down_distance = 100
                verify = left[0] * left[0] + left[1] * left[1]
                if verify < inner_circle or verify > outter_circle or previous_move == 'right':
                    left_distance = 100
                verify = right[0] * right[0] + right[1] * right[1]
                if verify < inner_circle or verify > outter_circle or previous_move == 'left':
                    right_distance = 100

                #chooses the next path node
                if up_distance < down_distance and up_distance < left_distance and up_distance < right_distance:
                    mc.forward(step, 1)
                    previous_move = 'up'
                elif down_distance < up_distance and down_distance < left_distance and down_distance < right_distance:
                    mc.back(step, 1)
                    previous_move = 'down'
                elif left_distance < up_distance and left_distance < down_distance and left_distance < right_distance:
                    mc.left(step, 1)
                    previous_move = 'left'
                else:
                    mc.right(step, 1)
                    previous_move = 'right'
                if float(drone_x) < target[0] + window and float(drone_x) > target[0] - window and float(drone_y) < target[1] + window and float(drone_y) > target[1] - window:
                    side_length = 1.1
                    mc.up(0.6)
                    for x in range(2):
                        mc.turn_right(15,45)
                        time.sleep(1)
                        for i in range(12):
                            mc.left(side_length, .75)
                            time.sleep(2)
                            mc.turn_right(30,45)
                            time.sleep(1)
                        mc.turn_left(15,45)
                        time.sleep(1)
                        mc.up(0.3)
                        time.sleep(1)
                    mc.land()



def optitrack(msg):
    global drone
    global drone_x
    global drone_y
    global drone_z
    global drone_xr
    global drone_yr
    global drone_zr
    global drone_wr
    global wand
    global wand_x
    global wand_y
    global wand_z
    global wand_xr
    global wand_yr
    global wand_zr
    global wand_wr

    crazy1 = msg.transforms.pop().__str__().replace(" ","").split("\n")
    title = crazy1[6].split(":")[1]
    if title == drone:
        drone_x = crazy1[9].split(":")[1]
        drone_y = crazy1[10].split(":")[1]
        drone_z = crazy1[11].split(":")[1]
        drone_xr = crazy1[13].split(":")[1]
        drone_yr = crazy1[14].split(":")[1]
        drone_zr = crazy1[15].split(":")[1]
        drone_wr = crazy1[16].split(":")[1]
    elif title == wand:
        wand_x = crazy1[9].split(":")[1]
        wand_y = crazy1[10].split(":")[1]
        wand_z = crazy1[11].split(":")[1]
        wand_xr = crazy1[13].split(":")[1]
        wand_yr = crazy1[14].split(":")[1]
        wand_zr = crazy1[15].split(":")[1]
        wand_wr = crazy1[16].split(":")[1]


if __name__ == '__main__':
    rospy.init_node('crazyflie_tracker')
    cflib.crtp.init_drivers()
    t1 = threading.Thread(target=crazyflie)
    position_topic = '/tf'
    pose_subscriber = rospy.Subscriber(position_topic, TFMessage, optitrack)
    t1.start()
    rospy.spin()
