# Pick and Place State Machine for SO-ARM100
# This node moves the robot arm through 7 states to pick a blue cube
# and place it in a different location

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # Publisher to send joint angle commands to the arm
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Publisher to open/close the gripper
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        # Subscribe to blue cube position from the perception node
        self.cube_pos = None
        self.create_subscription(
            PointStamped, '/blue_cube_position',
            self.cube_callback, 10)

        # Names of the 5 arm joints
        self.arm_joints = [
            'shoulder_pan_joint',   # rotates base left/right
            'shoulder_lift_joint',  # lifts shoulder up/down
            'elbow_flex_joint',     # bends elbow
            'wrist_flex_joint',     # bends wrist up/down
            'wrist_roll_joint'      # rolls wrist
        ]

        # Pre-defined joint poses (angles in radians)
        # Each list = [pan, lift, elbow, wrist_flex, wrist_roll]
        self.HOME     = [1.26,  0.0,  0.0,  0.0,  0.0]  # safe resting position
        self.APPROACH = [1.26, -0.5,  0.8,  0.3,  0.0]  # above the cube
        self.GRASP    = [1.26, -0.7,  1.0,  0.5,  0.0]  # at the cube level
        self.LIFT     = [1.26, -0.3,  0.6,  0.2,  0.0]  # raise cube up
        self.PLACE    = [0.0,  -0.5,  0.8,  0.3,  0.0]  # drop zone position

        self.get_logger().info('Pick and Place node started!')

        # Start the task after 3 seconds
        self.timer = self.create_timer(3.0, self.run_once)
        self.started = False

    def cube_callback(self, msg):
        # Called whenever perception node detects blue cube
        self.cube_pos = msg.point

    def move_arm(self, positions, secs=3):
        # Send joint angle command to the arm controller
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = positions          # target joint angles
        pt.velocities = [0.0] * 5        # stop smoothly at target
        pt.time_from_start = Duration(sec=secs)  # move duration
        msg.points = [pt]
        self.arm_pub.publish(msg)
        time.sleep(secs + 1)  # wait for motion to complete

    def move_gripper(self, pos):
        # pos = 0.0 means open, pos = 0.6 means closed
        msg = Float64MultiArray()
        msg.data = [pos]
        self.gripper_pub.publish(msg)
        time.sleep(1.5)

    def run_once(self):
        # Make sure we only run the task once
        if self.started:
            return
        self.started = True
        self.timer.cancel()
        self.execute()

    def execute(self):
        # STATE 1: Move to safe home position
        self.get_logger().info('STATE 1: HOME')
        self.move_arm(self.HOME)
        self.move_gripper(0.0)  # open gripper

        # STATE 2: Wait for camera to detect blue cube
        self.get_logger().info('STATE 2: WAITING FOR BLUE CUBE...')
        start = time.time()
        while self.cube_pos is None and time.time() - start < 10:
            rclpy.spin_once(self, timeout_sec=0.5)

        if self.cube_pos is None:
            self.get_logger().error('No blue cube found! Stopping.')
            return

        self.get_logger().info(
            f'Blue cube detected at x={self.cube_pos.x:.3f} y={self.cube_pos.y:.3f}')

        # STATE 3: Move above the blue cube
        self.get_logger().info('STATE 3: APPROACH')
        self.move_arm(self.APPROACH)

        # STATE 4: Lower down and close gripper to grab cube
        self.get_logger().info('STATE 4: GRASP')
        self.move_arm(self.GRASP)
        self.move_gripper(0.6)  # close gripper

        # STATE 5: Lift the cube up
        self.get_logger().info('STATE 5: LIFT')
        self.move_arm(self.LIFT)

        # STATE 6: Move to drop zone and release cube
        self.get_logger().info('STATE 6: PLACE')
        self.move_arm(self.PLACE)
        self.move_gripper(0.0)  # open gripper - release cube

        # STATE 7: Return to home position
        self.get_logger().info('STATE 7: HOME')
        self.move_arm(self.HOME)

        self.get_logger().info('PICK AND PLACE COMPLETE!')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
