# TC_EM001
import unittest
from unittest.mock import MagicMock
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, Detection2D, BoundingBox2D, Pose2D
import numpy as np
import cv2

from environmental_model.environmental_model import SensorFusionNode  # Assuming your node class is in environmental_model.py

class TestEnvironmentalModelWithMockData(unittest.TestCase):

    def setUp(self):
        # Initialize the ROS client library
        rclpy.init(args=None)
        # Initialize the SensorFusionNode
        self.node = SensorFusionNode()
        # Initialize CvBridge for image conversions
        self.bridge = CvBridge()
        # Mock the create_subscription method
        self.node.create_subscription = MagicMock()
        
        # Mocking the publisher
        self.mock_image_pub = MagicMock()
        self.node.create_publisher = MagicMock(return_value=self.mock_image_pub)
        self.node.image_pub = self.mock_image_pub

        # Mocking the transform lookup
        self.node.get_transform = MagicMock(return_value=MagicMock())
        self.node.get_transform.return_value.transform.translation.x = 0
        self.node.get_transform.return_value.transform.translation.y = 0
        self.node.get_transform.return_value.transform.translation.z = 0

    def tearDown(self):
        # Destroy the node and shutdown ROS client library
        self.node.destroy_node()
        rclpy.shutdown()

    def create_mock_image(self):
        # Create a mock image with zero values (black image)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        return image_msg

    def create_mock_lidar_scan(self):
        # Create a mock LaserScan message with a range of 1 meter
        scan = LaserScan()
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 0.01
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [1.0] * 314  # Simulate a range of 1 meter
        return scan

    def create_mock_detections(self):
        # Create a mock Detection2DArray message
        detection_array = Detection2DArray()
        detection = Detection2D()
        detection.results = []
        detection_result = ObjectHypothesisWithPose()
        detection_result.hypothesis.class_id = chr(6 - 1)
        detection_result.hypothesis.score = 80.0 / 100.0
        detection.results.append(detection_result)
        detection_array.detections.append(detection)

        # Define bounding box for the detection
        bbox = BoundingBox2D()
        bbox.center.position.x = 320.0
        bbox.center.position.y = 240.0
        bbox.center.theta = 0.0
        bbox.size_x = 50.0
        bbox.size_y = 50.0
        detection.bbox = bbox
        detection_array.detections.append(detection)

        return detection_array

    def create_mock_camera_info(self):
        # Create a mock CameraInfo message with intrinsic parameters
        camera_info = CameraInfo()
        camera_info.k = [525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0]
        return camera_info

    def test_with_mock_data(self):
        # Create mock messages
        mock_image = self.create_mock_image()
        mock_scan = self.create_mock_lidar_scan()
        mock_detections = self.create_mock_detections()
        mock_camera_info = self.create_mock_camera_info()

        # Simulate callbacks
        self.node.camera_info_callback(mock_camera_info)
        self.node.image_callback(mock_image)
        self.node.lidar_callback(mock_scan)
        self.node.detection_callback(mock_detections)

        # Check if the annotated image was published
        self.mock_image_pub.publish.assert_called()
        annotated_image_msg = self.mock_image_pub.publish.call_args[0][0]
        annotated_image = self.bridge.imgmsg_to_cv2(annotated_image_msg, "bgr8")
        
        # Verify that the bounding box is drawn on the annotated image
        self.assertTrue(np.any(annotated_image != self.node.current_image))

if __name__ == '__main__':
    unittest.main()