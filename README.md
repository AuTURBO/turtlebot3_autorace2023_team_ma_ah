# turtlebot3_autorace2023_team_ma_ah
---
![model](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/75b4d80b-bfca-4eae-9d56-95999b6d2fe8)

![dimension](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/3c673414-4247-446c-9cab-794d504c7967)
![map](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/0a314161-f460-4823-b085-5f71954e6f5c)

## 1) Line Tracking

> line detecting and line control
> ![lane_following_gazebo](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/7befdb41-05a5-4d46-bc4f-be8b824ee494)


---

### workflow

lane detection mode

1. Bird eye view
2. HSV filtering
3. sliding window
4. 왼쪽, 오른쪽 차선 구분
5. 차선 각도 예외처리 && 오차값 생성
6. 위 오차값을 이용한 조향각 PID 제어

## 2) Yolo detecting

> YOLOv7 + TensorRT
> ![yolo](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/81260570-0127-49b1-8518-8803514ff553)

---

## 3) Mode Control

> behavior planning using flexbe
> ![mode_control](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/6c21eeba-94f1-43c6-9840-634e8b5e23d3)

https://www.youtube.com/watch?v=2PzjZ15FKa0

> 각각의 미션을 flexbe를 이용하여 서브 트리로 구현 하였음.
> 
1. 미션 1 신호등 인식 
2. 미션 2 갈림길 
3. 미션 3 장애물 회피 
4. 미션 4 주차 
5. 미션 5 정지바 
6. 미션 6 터널 

# Mission Driving
![mission_map](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/a1a2ee94-2e65-4e16-9913-e13a99f7c49a)

## **Mission 1** - Traffic Light

> Traffic light detect and control
> 

---

![mission1](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/af8b4bb3-938c-4ba9-98e8-1eb1bca09980)

### workflow

> 모든 표지판 인식 state는 lane control를 기반으로 합니다.
> 
1. 신호등 인식 state를 통해 green일 경우 종료
2. 다음 표지판을 인식하기 전까지 lane control

## **Mission 2** - Cross Line

> Cross traffic sign detect and control
> 

---

![mission2](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/d0099146-919c-4251-9589-cd60d71857a5)

### workflow

> 모든 표지판 인식 state는 lane control를 기반으로 합니다.
> 
1. 갈림길 시작 표지판 인식 state 
2. 왼, 오른쪽 표지판 인식 state 
    1. 왼쪽 : 노랑 선 lane control 
    2. 오른쪽 : 흰선 lane control 
3. stop 표지판 인식 state

## **Mission 3** - Obstacle Avoidence

> Obstacle traffic sign detect and avoidence
> 

---
![mission3](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/00063626-8a48-4300-86b4-8c47f4663020)

### workflow

> 모든 표지판 인식 state는 lane control를 기반으로 합니다.
> 
1. 장애물 표지판 인식 state 
    1. 흰선 lane control 
2. 장애물 인식 state 
3. 휴리스틱한 장애물 회피
    1. pid 기반 각도 및 일정거리 이동 
4. 표지판 인식 state 

## **Mission 4** - Parking

> Parking traffic sign detect and parking
> 

---
![mission4](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/dcdcf02d-7967-4141-ba60-48be8a3b71cf)
![parking ](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/aea4f8f2-1082-489c-99bc-c54742b69540)(https://youtube.com/shorts/lqK7hY0W3qg?feature=shared)

### workflow

> 모든 표지판 인식 state는 lane control를 기반으로 합니다.
> 
1. 주차 표지판 인식 state 
    1. 노랑선 lane control 
2. 주차공간 진입 state 
3. 휴리스틱 한 주차 후 탈출 state 
    1. pid 기반 각도 및 일정거리 이동 
4. 오른쪽 노랑선 lane control state 

## **Mission 5** - Stop Bar

> StopBar detect and stop control
> 

---

![mission5](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/8910b44f-ec0e-4ac9-93ea-de4fda27f845)
### workflow

> 모든 표지판 인식 state는 lane control를 기반으로 합니다.
> 
1. stopbar 인식 state 
    1. 인식하면 정지
    2. 다른 표지판 인식 시 다음 state로 전환 

## **Mission 6** - Tunnel

> Tunnel detect and movebase control
> 

---

![mission6](https://github.com/AuTURBO/turtlebot3_autorace2023_team_ma_ah/assets/26687721/2e04fcfa-a4b3-4b6a-b7f3-ea2b179f5caa)

### workflow

> 모든 표지판 인식 state는 lane control를 기반으로 합니다.
> 
1. 터널 진입 표지판 인식 state 
2. initpose state 
3. clear costmap state 
4. goal pose state 
5. lane control state

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

