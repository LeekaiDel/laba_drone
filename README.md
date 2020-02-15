# Установка


<details><summary>подробнее..</summary><p>

перед установкой, поставить пакет следующие пакеты:

#### ros_geofencing:
Конвертирует WGS84 в ENU

```
git clone http://10.131.99.36/Other-projects/ros_geofencing.git
```

### Удаленный доступ к рабочему столу по VNC      

1. Подключиться к бортовому ПК дрона по ssh:       
```      
ssh op@адрес      
```     
пароль: 1     
2. Запустить VNC сервер:     
```    
vncserver    
```    
P.S. Для отключения сервера выполнить:     
```    
vncserver -kill :1    
```    
3. С удаленного ПК через VNC клиента подключиться к адресу адрес:5901.        
Пароль: 123456     
В качестве VNC клиента нормально подходит remmina (sudo apt-get install reminna)   
  
#### Octomap

Удобный контейнер для хранения облака точек 

```  bash
sudo apt install ros-${ROS_DISTRO}-octomap*
```

#### Google_carograpther

Локальная навигация по лидару, imu, pointcloud<br>
[полная инструкция](https://google-cartographer-ros.readthedocs.io/en/latest/)


``` bash
# Install dependences
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build libgflags-dev libgoogle-glog-dev liblmdb-dev 


sudo apt-get install -y \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    ninja-build \
    python-sphinx \
    stow
    
    
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build
cd build
cmake .. 
make
sudo make install

# Create a new workspace in 'catkin_ws'.
cd catkin_ws
wstool init src

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

# Install proto3.
src/cartographer/scripts/install_proto3.sh

# В ~/catkin_ws/src/cartographer/CMakeLists.txt заменить 32 строку на:<br>
find_package(Ceres REQUIRED)



catkin build

# lib for python
pip install serial
```


# Добавить файл параметров

Файл **drone_params.sh** поместить в домашнюю директорию и добавить в **~/bashrc**:

```bash
source ~/drone_params.sh
```

<details><summary>формат файла.</summary><p>

|Имя <br>параметра| Описание  | Тип  | Значение<br> по умолчанию |
|---|---|---|---|
| DRONE_ID          |   ID дрона 0..n   | int |
| DRONE_MARKER_ID   | ID маркера 0..n   | int |
| DRONE_IP          | Ip дрона          | str (ip) |   
| TEAM              | ID комынды 0..n   | int |
|DRONE_SAFE_MAX_ANGLE |Максимальный угол для отключения моторов | float | 85 |
|DRONE_SAFE_DISARM_DELAY | Время отключения моторов в safe mode | float | 0.2|
|ROSBRIDGE_SERVER_IP  | IP сервера rosbridge | str (ip) | ' ' |
|ROSBRIDGE_SERVER_PORT | Порт сервера rosbridge | int | 9090 |
|MAVROS_GCS_IP  | IP для автоподключения к QGC | str (ip) | '' |
|MAVROS_FCU_URL | UART адрес: скорость pixhawk | str | '/dev/ttySAC0:921600'|
|MAVROS_FCU_PORT | Port сервера QGC | int | 14101 |
|DRONE_REG_USE_PLANNER  | Флаг для использования локального планировщика | bool | False |
|DRONE_REG_USE_GEO_MODE | Флаг для использования данных от GPS | bool | False |


</p></details>
</p></details>

# Содержание папок
 * [drone_bringup](/drone_bringup): основной пакет для запуска и настройки параметров дрона
 * [drone_reg](/drone_reg): позиционный ругялятор
 * [unstable_planner](/unstable_planner): локаPointCloudльный планировщик на неустойчивых режимах
 * [planner_neuro](/planner_neuro): нейросетевой локальный планировщик (не тестировался)
 * [px4 files](/px4 files): файлы настройки для полётного контроллера
 * [drone_msgs](/drone_msgs): Пользователькие сообщения
 * [mavros_link](/mavros_link): Нод для связи ROS и Pixhawk через Mavlink(UART)
 * [brodcast_aggregation_server](/brodcast_aggregation_server): Пакет для широковешательной передачи данных в сети
 * [rtk_brodcast](/rtk_brodcast): сервер обмена широковещательными сообщениями
 * [virtual_gun](/virtual_gun): Система виртаульного боя на AR метках
 * [drone_remote_contol](/drone_remote_contol): пакет для управления дроном с телефона
 * [drone_data_analysis](/drone_data_analysis) : пакет для анализа полётов (вывод графика и т.д.)
 
 * [common_drone_pkgs](/common_drone_pkgs): Папка с едиными пакетами для разных дронов 
 
 **[common_drone_pkgs](/common_drone_pkgs) Включает:**
 
 * [potential_planner](common_drone_pkgs/potential_planner): локальный планировщик на потенциальных полях 
 * [interactive_goal](common_drone_pkgs/interactive_goal): интерактивный маркер для за задания целевой точки в rviz
 * [drone_statistics](common_drone_pkgs/drone_statistics): Ноды для сбора статистики полёта
 * [ground_control](common_drone_pkgs/ground_control): НПУ для RVIZ
 * [point_cloud_converter](common_drone_pkgs/point_cloud_converter): Конвертер PointCloud в PointCloud2 и обратно
 * [stereo_image_proc](common_drone_pkgs/stereo_image_proc): Image processing для стереопары
 * [window_detector](common_drone_pkgs/window_detector): Детектор окон стереопарой
 * [drone_global_planner](common_drone_pkgs/drone_global_planner): Глобавльный планировщик

 
# Структура проекта

![drone_diag-main](http://10.131.99.36/Laba_Drone/laba_drone/wikis/uploads/2c251e1bed1762b1b972f3acf7e1de34/drone_diag-main.png)
