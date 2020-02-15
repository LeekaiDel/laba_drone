#!/usr/bin/env python
# coding=utf8

import math
import numpy as np
import rospy
import time
from drone_msgs.msg import DronePose, Goal
from geometry_msgs.msg import Vector3, PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import tf.transformations as t
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Глобальные переменные
path_to_file  = "/home/op/Desktop/drone video/3.12.18/unstable/"
prefix = ""
mavros_root = "/mavros"
drone_pos_topic = mavros_root + "/local_position/pose"
lidar_topic = "/scan"
state_topic = "/planner_state"

drone_pose = DronePose()        # Координаты дрона
prev_drone_pose = DronePose()   # Предыдущие координаты дрона
goal = DronePose()              # Целевая позиция дрона
cur_nav_vel = Vector3()         # Текущая скорость дрона
current_ctr = TwistStamped()    # Текущее управление по V
cur_course = 0.0

current_min = 0.0               # текущий минимум лидара
d_goal = 0.3                    # дельта окрестность целевой точки
r_dist = 1.2                    # безопасное расстояние

path = float()                  # пройденный путь
received_goal = False           # флаг получения целевой точки
state_now = bool()              # текущее состояние планировщика


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Функции подписки на топики

def goal_cb(data):
    """
    Функция считывания целевой точки.

    @param data: Целевая точка
    @type data: drone_reg.msg.Goal
    """
    global goal
    global received_goal
    goal = data.pose
    received_goal = True


def imu_cb(data):
    """
    Функция считывания данных инерциальной системы дрона.

    @param data: данные инерциальной ситсемы
    @type data: sensor_msgs.msg.Imu
    """
    global drone_pose
    drone_pose.course = get_yaw_from_quat([data.orientation.x, data.orientation.y,
                                           data.orientation.z, data.orientation.w])


def nav_vel_cb(data):
    """
    Функция считывания текущей скорости дрона.

    :param data: скорость
    :type data: geometry_msgs.msg.TwistStamped
    """
    global cur_nav_vel
    cur_nav_vel = data.twist.linear


def ctr_cb(data):
    """
    Функция сичтывает значения управляющих команд дрона.

    :param data: команды управления
    :type data: std_msgs.msg.Float32MultiArray
    """
    global current_ctr
    current_ctr = data


def set_nav_gps(data):
    """
       Функция задания координат

       @param data: координаты
       @:type data: PoseStamped
       """
    global drone_pose, prev_drone_pose, cur_course
    set_path()

    prev_drone_pose = copy.deepcopy(drone_pose)

    drone_pose.point.x = data.pose.position.x
    drone_pose.point.y = data.pose.position.y
    drone_pose.point.z = data.pose.position.z

    cur_course = get_yaw_from_quat([data.pose.orientation.x, data.pose.orientation.y,
                                    data.pose.orientation.z, data.pose.orientation.w])


def set_path():
    global path, drone_pose, prev_drone_pose
    vec_pose = [prev_drone_pose.point.x - drone_pose.point.x,
                prev_drone_pose.point.y - drone_pose.point.y,
                prev_drone_pose.point.z - drone_pose.point.z]
    dt_pose = math.sqrt(vec_pose[0] ** 2 + vec_pose[1] ** 2 + vec_pose[2] ** 2)
    if dt_pose > 1:
        return
    path += dt_pose


def laser_scan_cb(data):
    global current_min

    current_min = min(data.ranges)  # find current min value


def state_cb(data):
    global state_now
    state_now = data.data

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Вспомогательные функции


def get_yaw_from_quat(q):
    """
    Функция возвращает угол рысканья полученный из кватерниона.

    @param q: кватернион
    @type q: list
    @return: угол рысканья
    """
    return t.euler_from_quaternion(q, axes='sxyz')[2]


def get_dist_to(current_pose, target_pose):
    """
    :type current_pose: DronePose()
    :type target_pose: DronePose()
    :return:
    """
    vec_pose = [current_pose.point.x - target_pose.point.x,
                current_pose.point.y - target_pose.point.y,
                current_pose.point.z - target_pose.point.z]
    dist = math.sqrt(vec_pose[0] ** 2 + vec_pose[1] ** 2 + vec_pose[2] ** 2)
    return dist


def plt_color(st_list):
    cols = list()
    # for ll in st_list:
    #     if ll == 1:
    #         cols.append('red')
    #     else:
    #         cols.append('black')
    for ll in st_list:
        if ll > 0:
            cols.append('red')
        else:
            cols.append('black')
    return cols


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Главная функция
def main():
    """
    Основной цикл узла ROS.

    @return: код завершения программы
    """
    global drone_pose, goal
    global cur_nav_vel, current_ctr
    global path, t_
    global received_goal
    global d_goal, current_min

    # Инициализируем узел ROS
    rospy.init_node("drone_reg_plotter")

    # Подписываемся на топики
    rospy.Subscriber("/goal_pose", Goal, goal_cb)
    rospy.Subscriber(mavros_root + "/local_position/velocity", TwistStamped, nav_vel_cb)
    rospy.Subscriber(mavros_root + "/setpoint_velocity/cmd_vel", TwistStamped, ctr_cb)
    rospy.Subscriber(lidar_topic, LaserScan, laser_scan_cb)
    rospy.Subscriber(drone_pos_topic, PoseStamped, set_nav_gps)
    rospy.Subscriber(state_topic, Bool, state_cb)

    # Инициализируем списки для построения графиков
    # Координаты
    drone_x = list()
    drone_y = list()
    drone_z = list()
    course = list()

    # Скорости дрона
    drone_vx = list()
    drone_vy = list()
    drone_vz = list()
    vel_tr = list()

    # Управление по скоростям
    ctr_x = list()
    ctr_y = list()
    ctr_z = list()

    time_plot = list()
    lidar_min = list()
    state = list()
    beta = list()

    t_ = 0.0
    rate = rospy.Rate(10)

    # Основной цикл
    # Пока не полетели - не записываем
    while math.sqrt(cur_nav_vel.x ** 2 + cur_nav_vel.y ** 2) < 0.05:
        pass
    # Пока не приехали в целевую точку
    old_ros_time = time.time()
    while get_dist_to(drone_pose, goal) > d_goal:

        # Рассчитываем время
        dt = time.time() - old_ros_time
        old_ros_time = time.time()
        t_ += dt

        coord_x = -drone_pose.point.x
        coord_y = -drone_pose.point.y
        coord_z = drone_pose.point.z

        vel_x = -cur_nav_vel.x
        vel_y = -cur_nav_vel.y
        vel_z = cur_nav_vel.z

        # Проверка на выбросы навигации
        if len(drone_x) > 3:
            if abs(coord_x - drone_x[-1]) > 0.5:
                coord_x = (drone_x[-1] + (drone_x[-1] - drone_x[-2]))
            if abs(coord_y - drone_y[-1]) > 0.5:
                coord_y = (drone_y[-1] + (drone_y[-1] - drone_y[-2]))

        # Заполняем данные для графиков
        # Координаты
        drone_x.append(coord_x)
        drone_y.append(coord_y)
        drone_z.append(coord_z)
        course.append(math.degrees(cur_course))

        # Скорости
        drone_vx.append(vel_x)
        drone_vy.append(vel_y)
        drone_vz.append(vel_z)
        vel_tr.append(math.sqrt(vel_x ** 2 + vel_y ** 2 + vel_z ** 2))

        # Управление
        ctr_x.append(-current_ctr.twist.linear.x)
        ctr_y.append(-current_ctr.twist.linear.y)
        ctr_z.append(current_ctr.twist.linear.z)

        # Разная лажа
        lidar_min.append(current_min)
        time_plot.append(t_)
        state.append(int(state_now))

        delta = current_min-r_dist
        beta.append(abs(delta)-delta)

        rate.sleep()

    # Вычисляем длину прямой между стартом и целью
    path_vec = [drone_x[0] - drone_x[-1], drone_y[0] - drone_y[-1], drone_z[0] - drone_z[-1]]
    path_min = np.linalg.norm(path_vec)

    # cols = plt_color(state)
    cols = plt_color(beta)

    print ('t = %s' % t_)
    print ('path = %s' % path)
    print ('path_min = % s' % path_min)
    print 'lidar_min = %s' % min(lidar_min)
    print ('dist to point = %s' % get_dist_to(drone_pose, goal))

    # координаты по осям
    ax1 = plt.subplot2grid((2, 3), (0, 0))
    ax1.plot(time_plot, drone_x)
    ax1.set_title('drone X')
    ax1.set_ylabel('x, m')
    ax1.set_xlabel('t, s')
    ax1.legend()
    ax1.grid()

    ax2 = plt.subplot2grid((2, 3), (0, 1))
    ax2.plot(time_plot, drone_y)
    ax2.set_title('drone Y')
    ax2.set_ylabel('y, m')
    ax2.set_xlabel('t, s')
    ax2.legend()
    ax2.grid()

    ax3 = plt.subplot2grid((2, 3), (0, 2))
    ax3.plot(time_plot, drone_z)
    ax3.set_title('drone Z')
    ax3.set_ylabel('z, m')
    ax3.set_xlabel('t, s')
    ax3.legend()
    ax3.grid()

    # # Управление по осям
    # ax4 = plt.subplot2grid((3, 3), (1, 0))
    # ax4.plot(time_plot, ctr_x)
    # ax4.set_title('control X')
    # ax4.set_ylabel('vx, m/s')
    # ax4.set_xlabel('t, s')
    # ax4.legend()
    # ax4.grid()
    #
    # ax5 = plt.subplot2grid((3, 3), (1, 1))
    # ax5.plot(time_plot, ctr_y)
    # ax5.set_title('control Y')
    # ax5.set_ylabel('vy, m/s')
    # ax5.set_xlabel('t, s')
    # ax5.legend()
    # ax5.grid()
    #
    # ax6 = plt.subplot2grid((3, 3), (1, 2))
    # ax6.plot(time_plot, ctr_z)
    # ax6.set_title('control Z')
    # ax6.set_ylabel('vz, m/s')
    # ax6.set_xlabel('t, s')
    # ax6.legend()
    # ax6.grid()

    # # траектория ху
    # ax7 = plt.subplot2grid((2, 3), (1, 0))
    # ax7.axis('equal')
    # ax7.scatter(drone_x, drone_y, s=40, c=cols)
    # ax7.plot(drone_x, drone_y, color='blue')
    # ax7.set_title('trajectory XY')
    # ax7.set_xlabel('x, m')
    # ax7.set_ylabel('y, m')
    # ax7.legend()
    # ax7.grid()

    # курс
    ax7 = plt.subplot2grid((2, 3), (1, 0))
    ax7.plot(time_plot, course)
    ax7.set_title('Phi')
    ax7.set_xlabel('t, s')
    ax7.set_ylabel('phi, degrees')
    ax7.legend()
    ax7.grid()

    # траекторная скорость
    ax8 = plt.subplot2grid((2, 3), (1, 1))
    ax8.plot(time_plot, vel_tr)
    ax8.set_title('Drone velocity')
    ax8.set_xlabel('t, s')
    ax8.set_ylabel('v, m/s')
    ax8.legend()
    ax8.grid()

    # beta
    ax9 = plt.subplot2grid((2, 3), (1, 2))
    ax9.plot(time_plot, beta)
    ax9.set_title('beta')
    ax9.set_xlabel('t, s')
    ax9.set_ylabel('beta')
    ax9.legend()
    ax9.grid()

    # 3d траектория
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.axis('equal')
    ax.plot(drone_x, drone_y, drone_z)
    ax.legend()


    np.save(path_to_file + prefix+"_time_dt", time_plot)
    np.save(path_to_file + prefix+"_drone_x", drone_x)
    np.save(path_to_file + prefix+"_drone_y", drone_y)
    np.save(path_to_file + prefix+"_drone_y", drone_y)
    np.save(path_to_file + prefix+"_drone_z", drone_z)
    np.save(path_to_file + prefix+"_course", course)
    np.save(path_to_file + prefix+"_vel_tr", vel_tr)
    np.save(path_to_file + prefix+"_beta", beta)

    plt.show()

    return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
