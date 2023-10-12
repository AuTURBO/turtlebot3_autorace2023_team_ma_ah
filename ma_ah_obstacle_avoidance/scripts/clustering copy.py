#!/usr/bin/env python3

import rospy
import numpy as np
import random
import sys

from collections import deque

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Duration 
from geometry_msgs.msg import Twist, Point
from sklearn.cluster import KMeans, DBSCAN
from sklearn.linear_model import RANSACRegressor

from KalmanFilter import KalmanFilter

from pure_pursuit import Robot

class LaserKMeansClustering:
    def __init__(self):
        # Publishers
        self.marker_pub = rospy.Publisher('/cluster_markers', MarkerArray, queue_size=10)
        self.obstacle_marker_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('/goal_markers', MarkerArray, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.line_pub = rospy.Publisher('/line_marker', Marker, queue_size=10)
        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.filtered_scan_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)
        self.origin_marker_pub = rospy.Publisher('/origin_marker', Marker, queue_size=10)
        self.points = None
        self.prev_objecet_position = None
        self.current_objecet_position = None
        self.object_change_flag = False
        self.move_direction = "left"
        self.prev_slope = 0
        self.prev_cetner_x = 0
        self.prev_cetner_y = 0

        self.previous_cluster_centers = []
        self.tracked_objects = deque(maxlen=5)  # maxlen can be adjusted as per requirements

        #Create KalmanFilter object KF
        #KalmanFilter(dt, u_x, u_y, std_acc, x_std_meas, y_std_meas)
        self.robot = Robot(x=0.0, y=0.0, yaw=0.0, velocity=0.01)

    def filter_laserscan(self, original_scan, min_distance, max_distance, min_angle, max_angle):
        filtered_scan = LaserScan()
        filtered_scan.header = original_scan.header
        filtered_scan.angle_min = original_scan.angle_min
        filtered_scan.angle_max = original_scan.angle_max
        filtered_scan.angle_increment = original_scan.angle_increment
        filtered_scan.time_increment = original_scan.time_increment
        filtered_scan.scan_time = original_scan.scan_time
        filtered_scan.range_min = original_scan.range_min
        filtered_scan.range_max = original_scan.range_max

        angle = original_scan.angle_min
        for r in original_scan.ranges:
            if min_distance <= r <= max_distance and min_angle <= angle <= max_angle:  
                filtered_scan.ranges.append(r)
            else:
                filtered_scan.ranges.append(float('inf'))
            angle += original_scan.angle_increment
        
        return filtered_scan


    def publish_filtered_scan(self, original_scan, filtered_points):
        # 원본 스캔 메시지를 복사하고 거리 데이터를 초기화
        filtered_scan = LaserScan()
        filtered_scan.header = original_scan.header
        filtered_scan.angle_min = original_scan.angle_min
        filtered_scan.angle_max = original_scan.angle_max
        filtered_scan.angle_increment = original_scan.angle_increment
        filtered_scan.time_increment = original_scan.time_increment
        filtered_scan.scan_time = original_scan.scan_time
        filtered_scan.range_min = original_scan.range_min
        filtered_scan.range_max = original_scan.range_max
        filtered_scan.ranges = [float('inf')] * len(original_scan.ranges)
        filtered_scan.intensities = list(original_scan.intensities)

        for x, y in filtered_points:
            angle = np.arctan2(y, x)
            idx = int((angle - filtered_scan.angle_min) / filtered_scan.angle_increment)
            distance = np.sqrt(x**2 + y**2)
            if 0 <= idx < len(filtered_scan.ranges):
                filtered_scan.ranges[idx] = distance

        self.filtered_scan_pub.publish(filtered_scan)


    def laserscan_to_points(self, msg):
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:  # valid range data
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
            angle += msg.angle_increment
        return np.array(points)

    def create_marker(self, x, y, cluster_id, marker_id, scale_x = 0.1, scale_y = 0.1):
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = 0.1

        duration = rospy.Duration(0.1)
        marker.lifetime = duration

        # 무작위로 RGB 색상을 생성
        random.seed(cluster_id)  # 같은 cluster_id는 항상 같은 색상을 갖도록
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()
        marker.color.a = 1.0  # Alpha

        return marker

    def publish_origin_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_scan"  # 원점이 위치하는 프레임 ID. 필요에 따라 변경
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.scale.x = 0.2  # 원점 마커의 크기
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha

        self.origin_marker_pub.publish(marker)

    def callback(self, msg):
        # 1m ~ 5m , -90 ~ 90 of degree 
        if self.move_direction == "left":
            filtered_scan = self.filter_laserscan(msg, 0.0, 1.0, np.pi/2 + (1.57 / 9) * 1.5, np.pi + np.pi/2 - (1.57 / 9) * 4)
        elif self.move_direction == "right":
            filtered_scan = self.filter_laserscan(msg, 0.0, 1.0, np.pi/2 + (1.57 / 9) * 4, np.pi + np.pi/2 - (1.57 / 9) * 1.5)
        self.filtered_scan_pub.publish(filtered_scan)

        # filtered_points = self.filter_laserscan(msg, 0.0, 2.5, 0, 1.57 * 2  )
        # self.publish_filtered_scan(msg, filtered_points)
        self.publish_origin_marker()
        self.points = self.laserscan_to_points(filtered_scan)

    def process(self):
        # 클러스터링
        points = self.points

        if len(points) > 0:
            clustering = DBSCAN(eps=0.1, min_samples=5).fit(points)
            cluster_centers = []

            labels = np.unique(clustering.labels_)

            for label in labels:
                if label != -1:  # 노이즈 데이터는 무시
                    mean = np.mean(points[clustering.labels_ == label], axis=0)
                    cluster_centers.append([mean, label])

            if len(cluster_centers) == 0:
                return None  # 아무런 클러스터도 발견되지 않았을 경우

            current_cluster_centers = self.track_clusters(cluster_centers)

            marker_array = MarkerArray()
            ## 클러스터링된 모든 클러스터 중앙점 표시
            for i, (mean_coords, label) in enumerate(current_cluster_centers):
                x, y = mean_coords  # Extract x and y from mean_coords
                if label != -1:  # -1은 노이즈에 해당
                    marker = self.create_marker(x, y, label, i)
                    marker_array.markers.append(marker)
            self.marker_pub.publish(marker_array)
            
            # 모든 클러스터 marker 시각화

            origin = np.array([0, 0])
            closest_center, closest_center_label = self.get_closet_point(origin, current_cluster_centers) # obstacle로 간주

            
            self.current_objecet_position = closest_center
            
            if self.current_objecet_position is not None and self.prev_objecet_position is not None:
                object_position_error = abs(self.current_objecet_position[0] - self.prev_objecet_position[0])
               # print(f"object_position_error = {object_position_error}")
                if object_position_error > 0.1:
                    print("----------------object change!!----------------")
                    self.object_change_flag = True
            else:
                pass
            self.prev_objecet_position = self.current_objecet_position

            # obstacle marker 시각화
            obstacle_marker_array = MarkerArray()
            marker = self.create_marker(closest_center[0], closest_center[1], closest_center_label, 0)
            obstacle_marker_array.markers.append(marker)

            closest_center_points = points[clustering.labels_ == closest_center_label]
            # smoothed_ranges = self.moving_average(closest_center_points)
            
            # RANSAC 적용
            try:
                ransac = RANSACRegressor(max_trials=100, min_samples=3, residual_threshold=0.05)
                ransac.fit(closest_center_points[:, 0].reshape(-1, 1), closest_center_points[:, 1])
            # 예측 범위 설정
                line_X = np.arange(closest_center_points[:, 0].min() - 0.1, closest_center_points[:, 0].max() + 0.1, 0.01)
                line_y_ransac = ransac.predict(line_X[:, np.newaxis])

                center_x = (line_X[0] + line_X[-1]) / 2.0
                center_y = (line_y_ransac[0] + line_y_ransac[-1]) / 2.0


                slope = (line_y_ransac[-1] - line_y_ransac[0]) / (line_X[-1] - line_X[0])
                angle = np.arctan(slope)  # 직선의 기울기로부터 각도를 찾습니다.
                angle_degrees = np.degrees(angle)

                # print("직선의 기울기:", slope)
                # print("직선의 각도 (라디안):", angle)
                # print("직선의 각도 (도):", angle_degrees)

                self.prev_cetner_x = center_x
                self.prev_cetner_y = center_y
                self.prev_slope = slope
                start_x, start_y, end_x, end_y = self.get_limited_line_points(slope, center_x, center_y, length=0.5)

                if start_y < end_y:
                    left_point_x = start_x
                    left_point_y = start_y
                    right_point_x = end_x
                    right_point_y = end_y
                else:
                    left_point_x = end_x
                    left_point_y = end_y
                    right_point_x = start_x
                    right_point_y = start_y

                self.publish_line(start_x, start_y, end_x, end_y)

                if self.object_change_flag == True:
                    # input("Object change")
                    # print(self.move_direction)
                    if self.move_direction == "left":
                        self.move_direction = "right"
                    elif self.move_direction == "right":
                        self.move_direction = "left"
                    # print(self.move_direction)
                    self.object_change_flag = False
                    # input("Object change Finish")

                goal_margin = 0.02
                print(f"self.move_direction = {self.move_direction}")
                if self.move_direction == "left":
                    sorted_point = sorted(closest_center_points, key=lambda closest_center_points: closest_center_points[1])
                    target_x = left_point_x #sorted_point[0][0] 
                    target_y = left_point_y - goal_margin #sorted_point[0][1] - goal_margin
                elif self.move_direction == "right":
                    sorted_point = sorted(closest_center_points, key=lambda closest_center_points: closest_center_points[1], reverse=True)
                    target_x = right_point_x #sorted_point[0][0] 
                    target_y = right_point_y + goal_margin#sorted_point[0][1] + goal_margin

                # obstacle left point marker 시각화
                marker = self.create_marker(sorted_point[0][0], sorted_point[0][1], 15, 5)
                obstacle_marker_array.markers.append(marker)
                
                self.obstacle_marker_pub.publish(obstacle_marker_array)

                goal_marker_array = MarkerArray()
                marker = self.create_marker(target_x, target_y, 16, 5)
                goal_marker_array.markers.append(marker)
                self.goal_marker_pub.publish(goal_marker_array)
                
                distance = self.euclidean_distance(origin, (target_x, target_x))
                print(f"closet obstacle distance: {distance}")

                cross_track_error = 0 - target_y
                print(f"cross_track_error:{cross_track_error}")
                cmd_vel = Twist()
                cmd_vel.linear.x = distance * 0.2 # 0.17
                cmd_vel.angular.z = cross_track_error * 4.0
                self.cmd_vel_pub.publish(cmd_vel)
            except:
                pass

            

    def get_object_coordinate(self, points):
            labels = np.unique(object.labels_)
            print(object.labels_)

            for label in labels:
                if label != -1:  # 노이즈 데이터는 무시
                    object_left = points[object.labels_ == label]

            goal_marker_array = MarkerArray()
            marker = self.create_marker(object_left[0][0], object_left[1][1], 30, 100, 0.1, 0.05)
            goal_marker_array.markers.append(marker)
            self.goal_marker_pub.publish(goal_marker_array)

    def euclidean_distance(self, a, b):
        return np.linalg.norm(a - b)

    def get_closet_point(self, origin, cluster_centers):       
        closest_center = None
        closest_center_label = None
        min_distance = float('inf')

        for i, (mean_coords, label) in enumerate(cluster_centers):
            distance = self.euclidean_distance(origin, mean_coords)
            if distance < min_distance:
                min_distance = distance
                closest_center = mean_coords
                closest_center_label = label

        return closest_center, closest_center_label
    

    def track_clusters(self, current_cluster_centers):
        points = self.points
        tracking_threshold = 0.2  # Adjust the value as per requirements
        y_left_threshold = -0.4 # Adjust the value as per requirements
        y_right_threshold = 0.4  # Adjust the value as per requirements
        dist_threshold = 0.1
        cluster_list = []

        # Track cluster centers 
        for current in current_cluster_centers:
            mean , label = current
            x, y = mean

            origin = np.array([0, 0])


            if not self.previous_cluster_centers:
                self.tracked_objects.append((x, y, label))
            else:
                tracked = False
                for previous in self.previous_cluster_centers:
                    px, py, plabel = previous
                    if np.sqrt((x - px)**2 + (y - py)**2) < tracking_threshold:
                        tracked = True
                        self.tracked_objects.append((x, y, label))
                        break

                if not tracked:
                    if (x, y, label) in self.tracked_objects:
                        self.tracked_objects.remove((x, y, label))

            dist = self.euclidean_distance(origin, mean)
            print(f"dist= {dist}")
            if dist < dist_threshold:
                if (x, y, label) in self.tracked_objects:
                    print(label)
                    self.tracked_objects.remove((x, y, label))
                continue

        print(self.tracked_objects)
        for x, y, label in self.tracked_objects:
            mean = x, y
            cluster = mean, label
            cluster_list.append(cluster)
        return cluster_list

        # Update previous cluster centers for next frame
        self.previous_cluster_centers = current_cluster_centers[:]

    def setup_line_marker(self, start_x, start_y, end_x, end_y):
        marker = Marker()
        marker.header.frame_id = "base_scan"  # laser_scan의 frame_id로 바꿔주세요.
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lines"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0

        point = Point()
        point.x = start_x
        point.y = start_y
        marker.points.append(point)

        point = Point()
        point.x = end_x
        point.y = end_y
        marker.points.append(point)

        return marker

    def publish_line(self, start_x, start_y, end_x, end_y):
        marker = self.setup_line_marker(start_x, start_y, end_x, end_y)
        self.line_pub.publish(marker)


    def get_limited_line_points(self, slope, center_x, center_y, length=1.0):
        # 방향 벡터 계산 (단위 벡터로 정규화)
        dx = 1.0
        dy = slope
        magnitude = (dx**2 + dy**2)**0.5
        dx /= magnitude
        dy /= magnitude

        # 원하는 길이의 1/2만큼 방향 벡터를 확장/축소
        half_length = length / 2.0
        start_x = center_x - dx * half_length
        start_y = center_y - dy * half_length
        end_x = center_x + dx * half_length
        end_y = center_y + dy * half_length

        return start_x, start_y, end_x, end_y

    def moving_average(self, data, window_size=5):
        return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

if __name__ == '__main__':
    rospy.init_node('laser_kmeans_clustering')
    node = LaserKMeansClustering()

    try:
        while not rospy.is_shutdown():  # Ctrl+C를 누를 때까지 실행
            # 여기에 로봇 제어 코드를 작성합니다.
            rospy.loginfo("Running...")  # 루프가 실행 중임을 표시
            if node.points is not None:
                    node.process()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
