# turtlebot3_autorace2023_team_ma_ah

### install

```jsx
sudo apt-get install ros-noetic-teb-local-planner
sudo apt-get install ros-noetic-behaviortree-cpp
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

```

### launch gazebo

```jsx
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### launch navi  - 맵은 따서 홈 디렉토리에

```jsx
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

### launch bt control

```jsx
roslaunch bt_sample bt_sample.launch
```

### pub topic - ex) 신호등이 있다는 표시판이 인식된 상황

```jsx
rostopic pub /interrupt_event std_msgs/String "traffic"
```
