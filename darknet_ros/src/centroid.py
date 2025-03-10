#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, CameraInfo
import tf2_ros
import tf2_geometry_msgs
from image_geometry import PinholeCameraModel
from std_msgs.msg import Bool 
from darknet_ros_msgs.srv import TogglePoseStreaming, TogglePoseStreamingResponse

class CentroidDepthPublisher:
    def __init__(self):
        rospy.init_node('centroid_publisher')
        
        # Initialize CV bridge and camera model
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False
        
        # TF buffer setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Point cloud data, initialization
        self.latest_cloud = None
        
        # Streaming starting point
        self.streaming_enabled = True

        #active replay starting point
        self.robot_moving = False ##

        # Publisher
        self.pose_pub = rospy.Publisher('/robot_ur10/centroid_points', PoseStamped, queue_size=10)
        
        # Camera frames (RealSense)
        self.camera_frame = "camera_depth_frame"  # depth frame
        self.base_frame = "robot_ur10/C_1"

        # Subscribers
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_box_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/robot_ur10/active_replay', Bool, self.robot_movement_callback) 

        # Service
        self.toggle_service = rospy.Service('/robot_ur10/toggle_streaming', TogglePoseStreaming, self.toggle_streaming)

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True

    def pointcloud_callback(self, cloud_msg):
        self.latest_cloud = cloud_msg

    def toggle_streaming(self, req):
        self.streaming_enabled = req.start_streaming  # Toggle based on request
        status = "resumed" if self.streaming_enabled else "stopped"
        rospy.loginfo(f"Pose publishing {status}")
        return TogglePoseStreamingResponse(success=True, message=f"Streaming {status}")
    
    def robot_movement_callback(self, msg):
        self.robot_moving = msg.data

    def get_depth_at_point(self, x, y):
        if self.latest_cloud is None:
            return None
        x, y = int(x), int(y)
        gen = pc2.read_points(self.latest_cloud, field_names=["x", "y", "z"], skip_nans=True, uvs=[[x, y]])
        try:
            point = next(gen)   
            return point  # (X, Y, Z) in camera frame
        except StopIteration:
            return None
        
    def bounding_box_callback(self, msg):
        if not msg.bounding_boxes:
            return
        if not self.streaming_enabled or self.robot_moving:
            return

        for box in msg.bounding_boxes:
            if box.probability < 0.95:
                continue

            x_center = (box.xmin + box.xmax) / 2.0
            y_center = (box.ymin + box.ymax) / 2.0

            point_3d = self.get_depth_at_point(x_center, y_center)
            if point_3d is None:
                continue

            point_in_camera = tf2_geometry_msgs.PointStamped()
            point_in_camera.header.frame_id = self.camera_frame
            point_in_camera.header.stamp = rospy.Time.now()
            point_in_camera.point.x = point_3d[0]
            point_in_camera.point.y = point_3d[1]
            point_in_camera.point.z = point_3d[2]

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, rospy.Time(0), rospy.Duration(1.0)
                )
                point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)

                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = self.base_frame
                pose_msg.pose.position = point_in_base.point
                pose_msg.pose.orientation.w = 1.0
                
                # print(f"Detected {box.Class} at {point_3d[0]:.4f}, {point_3d[1]:.4f}, {point_3d[2]:.4f} with the probability of {box.probability}")
                print(f"Detected {box.Class} at {point_3d[0]:.4f}, {point_3d[1]:.4f}, {point_3d[2]:.4f}")
                self.pose_pub.publish(pose_msg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"TF transformation failed: {e}")

if __name__ == '__main__':
    try:
        node = CentroidDepthPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass