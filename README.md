# turtlebot3_autorace2023_team_ma_ah

## install

```python
sudo apt install ros-noetic-flexbe-behavior-engine
```

```python
git clone -b noetic https://github.com/FlexBE/flexbe_app.git
```

### 바로가기 생성

```python
rosrun flexbe_app shortcut create  # or "remove" to remove it again
```

### 새 프로젝트 생성

```python
cd ~/catkin_ws/src
rosrun flexbe_widget create_repo [your_project_name]

# 위 명령으로 새 패키지가 생성되었으므로 다시 빌드하고 소싱합니다.
cd ~/catkin_ws
catkin build
source devel/setup.bash 
```

### 편집기 실행

```python
roslaunch flexbe_app flexbe_ocs.launch
```



## turtlebot3 bringup


### install 

```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```



### setup bashrc 

```bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

alias cs='cd ~/catkin_ws/src'
alias cw='cd ~/catkin_ws'
alias cm='cd ~/catkin_ws && catkin_make'
alias sb='source ~/.bashrc'
alias eb='nano ~/.bashrc'
export TURTLEBOT3_MODEL=burger
```



### launch gazebo

```bash
roslaunch turtlebot3_gazebo turtlebot3_autorace_2020.launch
```


### launch teleop 

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```


### launch slam 

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
#### save map

```bash
rosrun map_server map_saver -f ~/map
```


### launch navi  - 맵은 따서 홈 디렉토리에

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

## Flexbe

## Perception

### lane_detector 

```bash
rosrun ma_ah_perception lane_detector.py
```

### yolov7_TensorRT (Jetson Xavier)
```
```

### yolov7 for gazebo
```
roslaunch yolov7 yolov7.launch
```



### Ultrasonic
```
pip install Jetson.GPIO
```
```
rosrun ultrasonic ROS_sonar_sensor.py
```

