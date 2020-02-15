#!/usr/bin/env python
# coding=utf8

import math
import rospy
import time
import copy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose2D
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Глобальные переменные
lidar_topic = "/scan"
pose_topic = "/pose2D"
vel_topic = "/velocity"
beta_topic = "/beta"

cur_car_pose = Pose2D()             # Координаты машинки
prev_car_pose = Pose2D()        # Предыдущие координаты машинки
cur_vel = TwistStamped()        # Текущая скорость машинки
cur_vel_ctr = TwistStamped()    # Текущее управление по скорости

current_min = float()           # текущий минимум лидара
beta = float()                  # Бифуркационный параметр
path = 0.0                      # пройденный путь

# ЦТ ЗАДАЕТСЯ ВРУЧНУЮ!!!
goal_x = 10.0
goal_y = 10.0


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Функции подписки на топики

# функция считывания текущей скорости
def velocity_cb(data):
    global cur_vel
    cur_vel = data


# функция считывания текущей позиции
def pose_cb(data):
    global cur_car_pose
    cur_car_pose = data


# функция вычисления длины траектории
def set_path():
    global path, cur_car_pose, prev_car_pose
    vec_pose = [prev_car_pose.x - cur_car_pose.x,
                prev_car_pose.y - cur_car_pose.y]
    dt_pose = math.sqrt(vec_pose[0] ** 2 + vec_pose[1] ** 2)
    path += dt_pose
    prev_car_pose = copy.deepcopy(cur_car_pose)


# функция считываниия бифуркационного параметра
def beta_cb(data):
    global beta
    beta = data.data


# функция получения минимальной дистанции в лазерскане
def laser_scan_cb(data):
    global current_min
    current_min = min(data.ranges)  # find current min value


# функция получения дистанции до ЦТ
def dist_to_goal(current_pose):
    # global goal_x, goal_y
    vec_pose = [current_pose.x - goal_x,
                current_pose.y - goal_y]
    dist = math.sqrt(vec_pose[0] ** 2 + vec_pose[1] ** 2)
    return dist


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Главная функция
def main():

    global cur_car_pose, prev_car_pose, cur_vel, cur_vel_ctr
    global path, t_
    global current_min, beta

    # Инициализируем узел ROS
    rospy.init_node("car_plotter")

    # Подписываемся на топики
    # beta
    rospy.Subscriber(beta_topic, Float64, beta_cb)
    # лидар
    rospy.Subscriber(lidar_topic, LaserScan, laser_scan_cb)
    # позиция машинки
    rospy.Subscriber(pose_topic, Pose2D, pose_cb)
    # скорость машинки
    rospy.Subscriber(vel_topic, TwistStamped, velocity_cb)

    # Инициализируем списки для построения графиков
    # Координаты
    car_x = list()
    car_y = list()
    course = list()

    # Скорости
    car_vx = list()
    car_vy = list()
    vel_tr = list()

    time_plot = list()
    lidar_min = list()
    beta_list = list()

    t_ = 0.0
    rate = rospy.Rate(10)

    # Основной цикл
    # Пока не полетели - не записываем
    while math.sqrt(cur_vel.twist.linear.x ** 2 + cur_vel.twist.linear.y ** 2) < 0.2:
        pass
    # Пока не приехали в целевую точку
    old_ros_time = time.time()
    while math.sqrt(cur_vel.twist.linear.x ** 2 + cur_vel.twist.linear.y ** 2) > 0.1:

        # Рассчитываем время
        dt = time.time() - old_ros_time
        old_ros_time = time.time()
        t_ += dt

        # Заполняем данные для графиков
        # Координаты
        car_x.append(cur_car_pose.x)
        car_y.append(cur_car_pose.y)
        course.append(math.degrees(cur_car_pose.theta))

        # Скорости
        vel_x = cur_vel.twist.linear.x
        vel_y = cur_vel.twist.linear.y
        car_vx.append(vel_x)
        car_vy.append(vel_y)
        vel_tr.append(math.sqrt(vel_x ** 2 + vel_y ** 2))

        # Разная лажа
        lidar_min.append(current_min)
        time_plot.append(t_)
        beta_list.append(beta)
        set_path()

        # print 'beta: ', beta

        rate.sleep()

    # Вычисляем длину прямой между стартом и целью
    # path_vec = [car_x[0] - car_x[-1], car_y[0] - car_y[-1]]
    path_vec = [0 - goal_x, 0 - goal_y]
    path_min = np.linalg.norm(path_vec)

    # cols = plt_color(state)
    # cols = plt_color(beta)

    print ('t = %s' % t_)
    print ('path = %s' % path)
    print ('path_min = % s' % path_min)
    print ('lidar_min = %s' % min(lidar_min))
    print ('dist to point = %s' % dist_to_goal(cur_car_pose))

    # # координаты по осям
    # ax1 = plt.subplot2grid((2, 3), (0, 0))
    # ax1.plot(time_plot, car_x, color='black')
    # ax1.set_title('car X')
    # ax1.set_ylabel('x, m')
    # ax1.set_xlabel('t, s')
    # ax1.legend()
    # ax1.grid()
    #
    # ax2 = plt.subplot2grid((2, 3), (0, 1))
    # ax2.plot(time_plot, car_y, color='black')
    # ax2.set_title('car Y')
    # ax2.set_ylabel('y, m')
    # ax2.set_xlabel('t, s')
    # ax2.legend()
    # ax2.grid()

    # курс
    ax3 = plt.subplot2grid((2, 2), (0, 1))
    ax3.plot(time_plot, course, color='black')
    ax3.set_title('Theta')
    ax3.set_xlabel('t, s')
    ax3.set_ylabel('theta, degrees')
    ax3.legend()
    ax3.grid()

    # траектория ху
    ax4 = plt.subplot2grid((2, 2), (0, 0))
    ax4.axis('equal')
    ax4.plot(car_x, car_y, color='black')
    ax4.set_title('trajectory XY')
    ax4.set_xlabel('x, m')
    ax4.set_ylabel('y, m')
    ax4.legend()
    ax4.grid()

    # траекторная скорость
    ax8 = plt.subplot2grid((2, 2), (1, 1))
    ax8.plot(time_plot, vel_tr, color='black')
    ax8.set_title('Car velocity')
    ax8.set_xlabel('t, s')
    ax8.set_ylabel('v, m/s')
    ax8.legend()
    ax8.grid()

    # # beta
    # ax9 = plt.subplot2grid((2, 3), (1, 2))
    # ax9.plot(time_plot, beta_list)
    # ax9.set_title('beta')
    # ax9.set_xlabel('t, s')
    # ax9.set_ylabel('beta')
    # ax9.legend()
    # ax9.grid()

    plt.show()

    return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
