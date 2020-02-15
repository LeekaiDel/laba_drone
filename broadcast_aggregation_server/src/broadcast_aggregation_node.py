#!/usr/bin/env python
# coding=utf8
# Сервер для бродкаста состояний дроной. Передаёт класс в бинарном формате и серриализует его

import os
from socket import *
import pickle
from threading import Thread

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations as t

from drone_msgs.msg import DroneInfo, DroneInfoArray
import os

view_mode = False

drone = DroneInfo()
listOfDrones = DroneInfoArray()


if os.environ['USE_MESH'] == 'True':
    UDP_MASK = '192.168.1.255'
else:
    UDP_MASK = '255.255.255.255'
UDP_PORT = 5006

droneList_topic = "drone/list"


def health_cb(data):
    """
    Get health callback
    :type data: Float32
    :return:
    """
    global drone
    drone.health = data.data

def pose_cb(data):
    """
    Get pose of the drone
    :type data: PoseStamped
    :type drone: DroneInfo
    :return:
    """
    global drone
    drone.pose.point = data.pose.position
    drone.pose.course = get_yaw_from_quat(data.pose.orientation)

def send_brodcast_msgs(data):
    """
    Отправляем данные о дроне всем клиетам сети
    :param data: текущее состояние дрона
    :type data: DroneInfo
    """
    global udp_clien_sock
    MESSAGE = pickle.dumps(drone)
    udp_clien_sock.sendto(MESSAGE, (UDP_MASK, UDP_PORT))

def get_env_val():
    """
    Get values from env
    :type: DroneInfo
    """
    global drone

    drone.id_drone = int(os.environ["DRONE_ID"])
    drone.id_marker = int(os.environ["DRONE_MARKER_ID"])
    drone.ip = os.environ["DRONE_IP"]
    drone.team_num = int(os.environ["TEAM"])

def udp_server():
    # get data from broadcast
    global udp_server_sock

    print("Start UDP listener broadcast")
    while True:
        binData, addr = udp_server_sock.recvfrom(1024)  # buffer size is 1024 bytes
        data = pickle.loads(binData)
        set_drone_list(data)

def set_drone_list(data):
    """
    Added and update data of list
    :type data: DroneInfo
    :return: None
    """
    global listOfDrones

    # Added and update data of list
    find_item_flag = False
    if len(listOfDrones.drones) == 0:
        listOfDrones.drones.append(data)
        find_item_flag = True
    else:
        for index in range(len(listOfDrones.drones)):
            if listOfDrones.drones[index].id_drone == data.id_drone and \
                    listOfDrones.drones[index].team_num == data.team_num:
                listOfDrones.drones[index] = data
                print("find Id: %s" % (listOfDrones.drones[index].id_drone))
                find_item_flag = True

    if not find_item_flag:
        listOfDrones.drones.append(data)

### External functions
def get_yaw_from_quat(quat):
    """
    Функция возвращает угол рысканья полученный из кватерниона.

    @param quat: кватернион
    @type quat: Quaternion
    @return: угол рысканья
    """
    q = [quat.x, quat.y, quat.z, quat.w]

    return t.euler_from_quaternion(q, axes='sxyz')[2]



if __name__ == '__main__':
    rospy.init_node("broadcast_aggregation_node")

    rate = rospy.Rate(10)

    # init params
    view_mode = rospy.get_param('~view_mode', view_mode)
    UDP_MASK = rospy.get_param('~udp_mask', UDP_MASK)
    UDP_PORT = rospy.get_param('~udp_port', UDP_PORT)
    get_env_val()

    if not view_mode:
        rospy.Subscriber("drone/health", Float32, health_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)

        # init client params
        udp_clien_sock = socket(AF_INET, SOCK_DGRAM)  # Internet,UDP
        udp_clien_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        udp_clien_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

    pub_drone_list = rospy.Publisher(droneList_topic,DroneInfoArray,queue_size=10)

    # init server params
    udp_server_sock = socket(AF_INET, SOCK_DGRAM)  # Internet,UDP
    udp_server_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    udp_server_sock.bind(("", UDP_PORT))

    # run udp server
    t1 = Thread(target=udp_server)
    t1.daemon = True
    t1.start()

    while not rospy.is_shutdown():
        if not view_mode:
            send_brodcast_msgs(drone)
        # pub list of drones
        pub_drone_list.publish(listOfDrones)
        rate.sleep()
