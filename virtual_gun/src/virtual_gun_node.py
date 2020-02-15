#!/usr/bin/env python
# coding=utf8
# Система виртуального боя. Включает в себя сервер / клиент на базе TCP/IP. База даннных берётся с внешнего сервера
# перед отправкой данные серриализуются и отправляеются на сервер

from socket import *
import os
import pickle
import socket
import time
import numpy as np
import math
from threading import Thread

import rospy
from std_msgs.msg import Float32, Int8
from drone_msgs.msg import DroneInfoArray, DroneInfo, DronePose, Strike
from perception_msgs.msg import MarkerList, Marker

droneList_topic = "drone/list"
droneHealth_topic = "drone/health"
droneAim_topic = "drone/aim"
markers_topic = "/aruco_eye/aruco_observation"

# init params
PORT = 5010
drone_healht = 100.0
recharge_time = 1.0     # recharge time in seconds
shoot_own_flag = False
deathmatch_flag = True
force_shot = 20.0

aim_drone = DroneInfo()
droneList = DroneInfoArray()
droneId = None
droneTeam = None

########## ROS methods ###########

def droneList_cb(data):
    """
    Callback base of drones
    :type data: DroneInfoArray
    """
    global droneList
    droneList = data

def droneAim_cb(data):
    """
    Callback the goal aim num
    :type data: Strike
    """
    global aim_drone
    aim_drone= data

def get_markers_fb_cb(data):
    """
    get find markers_list
    :type aim_drone:
    :type data: MarkerList
    :return:
    """
    global droneList, aim_drone

    hitList = list()

    # # мочить всех
    # if aim_drone = -1:
    #     # расчёт силы для каждого дрона
    #     if aim_drone == -1:


    for marker in data.markers:
        hitList.append((int(marker.id),calculation_shot_force(marker)))


    max_marker,drone = get_drone_and_power_shot(hitList, aim_drone)

    if max_marker and drone:
        shot(max_marker[1],drone.ip)
        print("shot:",max_marker[0], max_marker[1], drone.ip)



def get_drone_and_power_shot(list, aim_drone):
    """
    Return marker id with the strongest impact
    :type aim_drone: Strike
    :param list: [int id, float dist]
    :return: [int id, float dist]
    """

    global droneId, droneTeam

    maxItem = None
    _drone = None


    for item in list:
        _drone = find_drone_by_marker(item[0])
        if check_aim_restrictions(_drone, aim_drone):

            if deathmatch_flag:
                # shoot to any drone
                if not maxItem:
                        maxItem = item
                elif maxItem:
                    if maxItem[1] < item[1]:
                        maxItem = item
            else:
                # shoot to the goal drone
                if _drone.id_drone == aim_drone.id_drone and _drone.team_num == id_drone.team_num:
                    return item, _drone
        else:
            continue

    return maxItem, _drone


def check_aim_restrictions(_drone, aim_strike):
    """
    check of restictions

    :type drone: DroneInfo
    :type aim_strike: Strike
    :return: bool
    """
    global  droneId, droneTeam

    # search our database of drone
    if not _drone or _drone.health <= 0.0:
        rospy.logwarn("drone.health <= 0.0",)
        return False
    # exclude own a drone myself
    elif _drone.id_drone == droneId and _drone.team_num == droneTeam:
        rospy.logwarn(str.format("the same drone: id = %s, team = %s", droneId, droneTeam))
        return False
    # don't shoot your own
    elif shoot_own_flag == False and _drone.team_num == droneTeam:
        rospy.logwarn("shot_own_flag:=false - drones on the same team #%s", droneTeam)
        return False

    return True




### Server

def receive_from_server():
    """
    receive hit

    :return:
    """
    global server
    while True:
        client_socket, address = server.accept()
        print 'Accepted connection from {}:{}'.format(address[0], address[1])
        request = client_socket.recv(1024)
        client_socket.close()

        data = deserialize_recv(request)
        if data:
            set_health(data)

def set_health(strike):
    """
    hit calculation

    :type strike: Strike
    :return:
    """
    global drone_healht
    drone_healht -= strike.shot
    if drone_healht < 0.0:
        drone_healht = 0.0
    print("Get chot:", strike.shot, "health:",drone_healht)

def deserialize_recv(binData):
    try:
        data = pickle.loads(binData)

        if data._type == Strike()._type:
            return data
        else:
            return None
    except:
        return None

def shot(force, ip):
    """
    send shoot to server

    :param force: damage
    :param ip: ip of the goal drone
    """
    global recharge_current_time, recharge_time


    if recharge_time > recharge_current_time:
            return
    else:
        recharge_current_time = 0.0

    shoot = Strike()
    shoot.team_num = droneTeam
    shoot.id_drone = droneId
    shoot.shot = force

    binMsg = pickle.dumps(shoot)
    try:
        # send to server

        sock = socket.socket()
        sock.connect((ip, PORT))
        sock.send(binMsg)
        sock.close()
    except:
        # error

        pass

def find_ip_drone_frome_base(aim_num, aim_team):

    """
    Поиск ip дрона в базе данных
    :param aim_id: id заданного дрона
    :param aim_team: номер команды дрона
    :type droneList: DroneInfoArray
    :return: Drone
    """
    global droneList

    for drone in droneList.drones:
        if drone.id_drone == aim_num and aim_team == drone.team_num:
            return drone.ip

    for drone in droneList.drones:
        if drone.id_drone == aim_num and aim_team == -1:
            return drone.ip
    return None

def find_drone_by_marker(marker_id):

    """
    Поиск ip дрона в базе данных
    :param aim_id: id заданного дрона
    :param aim_team: номер команды дрона
    :type droneList: DroneInfoArray
    :return: DroneInfo
    """
    global droneList

    for drone in droneList.drones:
        if drone.id_marker == marker_id:
            return drone
    return None

def e1_k(dist):
    """
    Возвращает коэффициент мощности
    :return: 0..1
    """
    offset_x = 3.0
    offset_y = 0.5

    scale_x = 1
    scale_y = 0.4

    scale = 1.0

    f = math.atan((-dist + offset_x) * scale_x) * scale_y + offset_y
    f *= scale
    f = 1.0 if f > 1.0 else f
    f = 0.0 if f < 0 else f
    return f

def e2_k(dist):
    """
    Возвращает коэффициент радиуса
    :return: 0..1
    """
    offset_x = 1.0
    offset_y = 0.5

    scale_x = 3
    scale_y = 0.4
    scale = 2.0

    f = math.atan((-dist + offset_x) * scale_x) * scale_y + offset_y
    f *= scale
    f = 1.0 if f > 1.0 else f
    f = 0.0 if f < 0 else f
    return f

def calculation_shot_force(marker):
    """
    calculation force of shot

    :type marker: Marker
    :param marker::The marker of the find drone
    :return:
    """
    vec = [marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z]
    dist = np.linalg.norm(vec)
    r = np.linalg.norm([vec[0],vec[1]])


    # calculate
    e1 = e1_k(dist)
    e2 = e2_k(dist)

    force = e1 * force_shot

    return force


if __name__ == '__main__':

    rospy.init_node("brovirtual_gun_node")

    rate = rospy.Rate(10)

    # init params
    PORT = rospy.get_param('~port', PORT)
    recharge_time = rospy.get_param('~recharge_time', recharge_time)
    drone_healht = rospy.get_param('~healht', drone_healht)
    force_shot = rospy.get_param('~force', force_shot)
    shoot_own_flag = rospy.get_param('~shoot_own_flag', shoot_own_flag)
    deathmatch_flag = rospy.get_param('~recharge_time', recharge_time)

    droneId = int(os.environ["DRONE_ID"])
    droneTeam = int(os.environ["TEAM"])

    # init subscriber / publisher
    rospy.Subscriber(droneList_topic, DroneInfoArray, droneList_cb)
    rospy.Subscriber(markers_topic, MarkerList, get_markers_fb_cb)

    rospy.Subscriber(droneAim_topic, Strike, droneAim_cb)
    health_pub = rospy.Publisher(droneHealth_topic, Float32, queue_size=10)

    # inti udp server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("", PORT))
    server.listen(5)  # max backlog of connections

    # # run udp server
    server_recv_thrd = Thread(target=receive_from_server)
    server_recv_thrd.daemon = True
    server_recv_thrd.start()

    # init time
    old_time = time.time()
    recharge_current_time = 0.0
    try:
        while not rospy.is_shutdown():
                # calculate time
                dt = time.time()-old_time
                old_time = time.time()
                recharge_current_time += dt

                # listener from server
                health_pub.publish(drone_healht)
                rate.sleep()
    except:
        print("exit")
        server.close()