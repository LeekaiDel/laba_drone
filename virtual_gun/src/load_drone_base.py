#!/usr/bin/env python
# coding=utf8
# Система виртуального боя. Включает в себя сервер / клиент на базе TCP/IP. База даннных берётся с внешнего сервера
# перед отправкой данные серриализуются и отправляеются на сервер

from socket import *
import os
import pickle
import socket

from threading import Thread

import rospy
from std_msgs.msg import Float32, Int8
from drone_msgs.msg import DroneInfoArray, DroneInfo, DronePose, Strike
import copy

droneList_topic = "drone/list"

ip = '192.168.128.101'
drone_list = DroneInfoArray()

drone_1 = DroneInfo()
drone_2 = DroneInfo()
drone_3 = DroneInfo()


drone_1.ip = ip
drone_1.id_drone = 1
drone_1.id_marker = 1
drone_1.team_num = 0
drone_1.health = 100.0
drone_list.drones.append(drone_1)


drone_2.ip = ip
drone_2.id_drone = 2
drone_2.id_marker = 2
drone_2.team_num = 1
drone_2.health = 20.0
drone_list.drones.append(drone_2)

drone_3.ip = ip
drone_3.id_drone = 3
drone_3.id_marker = 3
drone_3.team_num = 0
drone_3.health = 30.0
drone_list.drones.append(drone_3)

def health_cb(data):
    drone_1.health = data.data


if __name__ == '__main__':

    rospy.init_node("virt_data_base")

    rate = rospy.Rate(10)


    list_pub = rospy.Publisher(droneList_topic, DroneInfoArray, queue_size=10)
    rospy.Subscriber("drone/health", Float32, health_cb)

    try:
        while not rospy.is_shutdown():
                print drone_1.health
                list_pub.publish(drone_list)
                rate.sleep()
    except:
        print("exit")
