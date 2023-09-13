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

### launch gazebo

```jsx
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### launch navi  - 맵은 따서 홈 디렉토리에

```jsx
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

