# virtual gun system

## vvitrual_gun_node.py
Система виртуального боя.

## Алгоритм работы

![Алгоритм](https://lh4.googleusercontent.com/owk63zAbvSiz6gvo-OtdslKk8m7GRYSvm9WDv7x1_rSh4SAXQXtZoNr3PZv44hoPx7PH6Vw_eWg3nr9h4POi=w1920-h1069)


#### Subscribed Topics:

aruco_eye/aruco_observation ([perception_msgs:MarkerList](https://github.com/joselusl/perception_msgs/blob/master/msg/MarkerList.msg)): Список найденных маркеров<br/>
drone/aim ([drone_msgs:Strike](http://10.131.99.36/Laba_Drone/laba_drone/blob/master/drone_msgs/msg/Strike.msg)) id и номер команды целевого дрона.<br>
drone/health ([drone_msgs:DroneInfoArray](http://10.131.99.36/Laba_Drone/laba_drone/blob/master/drone_msgs/msg/DroneInfoArray.msg)): База координат всех дронов в сети <br/> 

#### Publisher Topics:

drone/health ([std_msgs:Float32](http://docs.ros.org/lunar/api/std_msgs/html/msg/Float32.html)): Остаток жизней дрона.<br/>

#### Parameters:

~healht (float, default: 100.0)<br>
&emsp;&emsp;*Начальный уровень здоровья.<br/>*
~force_shot (float, default: 20.0)<br>
&emsp;&emsp;*Максимальная сила выстрела.<br/>*
~recharge_time(float: default: 1.0)<br>
&emsp;&emsp;*Время перезаряда оружия.<br/>*
~shoot_own_flag (bool, default: false)<br>
&emsp;&emsp;*Стрелять ли по своим.<br/>*
~deathmatch_flag (bool, default: true)<br>
&emsp;&emsp;*Средять по первой попавшейся цели.<br/>*
~udp_port (int, default: 5010)<br/>
&emsp;&emsp;*Порт сервера.<br/>*

**Также ряд параметров испульзуется из переменных окружения:**
   
"DRONE_ID" (int)<br>
&emsp;&emsp;*ID дрона.<br/>*
"TEAM" (int)<br>
&emsp;&emsp;*Номер команды, которой принадлежит дрон.<br/>*
