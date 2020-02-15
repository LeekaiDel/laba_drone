# broadcast_aggregation_server

## brodcast_aggregation_node.py
Пакет для широковешательной передачи данных в сети
Позволяет передавать данные о своём положении, а также о состоянии дрона

## Алгоритм работы

![Алгоритм](https://lh3.googleusercontent.com/Q9CcGrKJiuggv9KURs5s3-eyZ39q0UAHLfRL_QzM-7JLiPQLky6MM_JpJS1ING11rDdqfPBicwQnVdxLUVq6MfqEbPx94gHA6FpIRikpYozQziYE0QGWnUthv2etqp7rU1J4WtUEMQEiOwu8MkITqPpYFcU4CqfHO-1gKfK9-JTGrSl8czD3TRwXFsZECCHqg3o8lbfksHx3tNw9uxYtmskXVqwwvjAXBRiYZpTA1M5b2nja1q8CqFwiTCIrzCXvdBm_2xV1o4xbYxyxJFfP-FmBeF2KFSW_haKeTa-kb93N995jd0pKU1Lqgo6FxgCFfvGEu6MGB8yC1SjzHo9FRKtlBR1B-WwU-t6PrsIDkMKg6HYeDkZ3k19iZaBEpT9bwDyfQFkP_YApoHzW7ReunRM-_bL1DYBO9UyIWIiLWVdnkx35uPS5aS8pRmfoND9FdLgjARy5dESfAEm-XFjL_LLhsmGWwXjBtFFYImhiK7DreIAhVoxLdhWK1z4XmkUODWkbrtIGTbG-Xow4dWlF0n45tZT_jtyb2I26CAUnrkktln276J9eRii9bN2eXfgLgNxwCd4yJC5NBx-DFkZs4AvuVYkAyZJR=w1849-h922)



#### Subscribed Topics:

mavros/local_position/pose ([geometry_msgs:PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)): текущее положение<br/>
drone/health ([std_msgs:Float32](http://docs.ros.org/lunar/api/std_msgs/html/msg/Float32.html)): Остаток жизней дрона. Значение формируется отдельной системой боя<br/>

#### Publisher Topics:
drone/list ([drone_msgs:DroneInfoArray](http://10.131.99.36/Laba_Drone/laba_drone/blob/master/drone_msgs/msg/DroneInfoArray.msg)): База координат всех дронов в сети <br/> 

#### Parameters:

~udp_mask (string, default: "255.255.255.255")<br/>
&emsp;&emsp;*Маска подсети.<br/>*
~udp_port (int, default: 5006)<br/>
&emsp;&emsp;*Порт сервера.<br/>*

**Также ряд параметров испульзуется из переменных окружения:**
   
"DRONE_ID" (int)<br>
&emsp;&emsp;*ID дрона.<br/>*
"DRONE_MARKER_ID" (int)<br>
&emsp;&emsp;*ID маркера, который установлен на дроне.<br/>*
"DRONE_IP" (string)<br>
&emsp;&emsp;*IP адрес дрона.<br/>*
"TEAM" (int)<br>
&emsp;&emsp;*Номер команды, которой принадлежит дрон.<br/>*
