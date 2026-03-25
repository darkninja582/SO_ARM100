# Blue Cube Detector using OpenCV
# This node reads images from an overhead camera in Gazebo,
# detects the blue cube using HSV color filtering,
# and publishes its 3D world position

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class BlueCubeDetector(Node):
    def __init__(self):
        super().__init__('blue_cube_detector')

        # CvBridge converts ROS image messages to OpenCV format
        self.bridge = CvBridge()
        self.camera_info = None

        # Camera is mounted 1.2m high, table surface is at 0.4m
        self.camera_height = 1.2
        self.table_height  = 0.4

        # Subscribe to camera image and camera info topics
        self.image_sub = self.create_subscription(
            Image, '/overhead_camera/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/overhead_camera/camera_info', self.info_callback, 10)

        # Publish the detected cube's 3D position
        self.position_pub = self.create_publisher(
            PointStamped, '/blue_cube_position', 10)

        # Publish debug image so we can see what the camera sees
        self.debug_pub = self.create_publisher(
            Image, '/blue_cube_debug', 10)

        self.get_logger().info('Blue Cube Detector started!')

    def info_callback(self, msg):
        # Save camera calibration data (needed for 3D position calculation)
        self.camera_info = msg

    def image_callback(self, msg):
        # Step 1: Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Step 2: Convert BGR color to HSV color space
        # HSV is better for color detection than BGR
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Step 3: Create a mask that keeps only BLUE pixels
        # HSV range: Hue=90-140 (blue), Saturation=50-255, Value=30-255
        lower_blue = np.array([90,  50,  30])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Step 4: Clean up the mask (remove noise)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)  # remove small dots
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # fill small holes

        # Step 5: Find the blue object in the mask
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug_frame = frame.copy()

        if contours:
            # Get the largest blue blob (most likely the cube)
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 100:  # ignore tiny noise blobs
                # Get bounding box and center pixel of the cube
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2  # center x pixel
                cy = y + h // 2  # center y pixel

                # Step 6: Convert pixel position to real world coordinates
                world_x, world_y = self.pixel_to_world(cx, cy, frame.shape)

                if world_x is not None:
                    # Publish 3D position of blue cube
                    point = PointStamped()
                    point.header.stamp = self.get_clock().now().to_msg()
                    point.header.frame_id = 'world'
                    point.point.x = world_x
                    point.point.y = world_y
                    point.point.z = self.table_height  # cube is on table
                    self.position_pub.publish(point)
                    self.get_logger().info(
                        f'Blue cube at world: x={world_x:.3f}, y={world_y:.3f}')

                # Draw green box around detected cube on debug image
                cv2.rectangle(debug_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(debug_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(debug_frame, f'BLUE ({cx},{cy})',
                    (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(debug_frame, 'No blue cube detected',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Show camera view and mask side by side for debugging
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([debug_frame, mask_bgr])
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(combined, encoding='bgr8'))

    def pixel_to_world(self, cx, cy, shape):
        """
        Convert pixel coordinates (cx, cy) to real world (x, y) coordinates.
        Uses camera intrinsics and known height of camera above table.
        """
        if self.camera_info is None:
            return None, None

        # Camera focal lengths and principal point from calibration
        fx = self.camera_info.k[0]  # focal length x
        fy = self.camera_info.k[4]  # focal length y
        px = self.camera_info.k[2]  # principal point x
        py = self.camera_info.k[5]  # principal point y

        # Height from camera down to table surface
        dz = self.camera_height - self.table_height  # = 0.8m

        # Back-project pixel to world coordinates
        cam_x = (cx - px) * dz / fx
        cam_y = (cy - py) * dz / fy

        # Camera is positioned at world (0.3, 0.0)
        world_x = 0.3 + cam_x
        world_y = 0.0 + cam_y

        return world_x, world_y

def main(args=None):
    rclpy.init(args=args)
    node = BlueCubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
