#!/usr/bin/env python3

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
import numpy as np
from co_robot.srv import vision_robot


class PointCloudVisualizer:
    def __init__(self):
        rospy.init_node('realsense_pointcloud_visualizer')
        self.listener = tf.TransformListener()

        rospy.loginfo("Waiting for /service server...")
        try:
            rospy.wait_for_service('/service', timeout=10)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for /service server. Exiting.")
            exit(1)

        self.service_client = rospy.ServiceProxy("/service", vision_robot)

        self.cloud_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pointcloud_callback)
        self.center_sub = rospy.Subscriber('/detected_objects', Point, self.callback)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

        self.selected_x = None
        self.selected_y = None
        self.processing = False         # 현재 감지에 대해 서비스 호출 처리 중
        self.awaiting_100 = False       # 서비스 응답이 100 올 때까지 새로운 좌표 무시

    def pointcloud_callback(self, pointcloud_msg):
        # 좌표가 없거나 처리 중이면 아무것도 하지 않음
        if self.selected_x is None or self.processing or self.selected_x == -1:
            return

        # TF 변환 시도
        try:
            (trans, rot) = self.listener.lookupTransform('/base', '/camera_link1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", e)
            return

        gen = pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True,
                              uvs=[[self.selected_x, self.selected_y]])
        for p in gen:
            # 좌표 처리 시작
            self.processing = True
            camera_point = np.array([p[0], p[1], p[2], 1.0])
            transform_matrix = tf.transformations.quaternion_matrix(rot)
            transform_matrix[0:3, 3] = trans
            world_point = np.dot(transform_matrix, camera_point)

            # Marker 표시
            marker = Marker()
            marker.header.frame_id = "base"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=world_point[0], y=world_point[1], z=world_point[2])
            marker.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # 좌표 계산
            real_x = world_point[0] * 1000
            real_y = world_point[1] * 1000
            real_z = world_point[2] * 1000

            grip_x = 200
            home_x = -0.336
            home_y = 0
            home_z = 494.672

            x = real_x - home_x - grip_x
            y = real_y - home_y
            z = real_z - home_z

            # 서비스 호출
            try:
                response = self.service_client(x, y, z)
                rospy.loginfo("Service response: %s", response)

                if response.state == 100:
                    rospy.loginfo("✅ 작업 완료 (return=100), 다음 좌표 허용")
                    self.awaiting_100 = False
                else:
                    rospy.loginfo("🕓 작업 진행 중 (return=%d), 좌표 무시 대기", response.state)
                    self.awaiting_100 = True

            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
            except Exception as e:
                rospy.logerr("Unexpected error: %s", e)

            # 마커 발행 및 상태 초기화
            self.marker_pub.publish(marker)
            self.selected_x = None
            self.selected_y = None
            self.processing = False
            break

    def callback(self, data):
        # return=100이 오기 전에는 감지 무시
        if self.awaiting_100:
            rospy.loginfo("감지 무시 중 (return=100 대기)")
            return

        if data.x == -1 and data.y == -1:
            rospy.loginfo("No object detected.")
            self.selected_x = -1
            self.selected_y = -1
        else:
            rospy.loginfo("New object detected at (%d, %d)", int(data.x), int(data.y))
            self.selected_x = int(data.x)
            self.selected_y = int(data.y)


if __name__ == '__main__':
    try:
        visualizer = PointCloudVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
