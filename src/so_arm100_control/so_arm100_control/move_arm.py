import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import ikpy.chain

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander_node')
        # 1. Create the Publisher targeting your specific controller topic
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10)
        
        # Short delay to allow the publisher to establish connection with Gazebo
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

    def move_to_coordinate(self, target_xyz):
        self.get_logger().info(f"Calculating IK for target: {target_xyz}")

        # 1. The 20-item mask based on your exact joint indices
        # True ONLY for indices: 4, 7, 10, 13, 16
        custom_mask = [
            False, False, False, False, True,   # 0-4 (Pan is 4)
            False, False, True,                 # 5-7 (Lift is 7)
            False, False, True,                 # 8-10 (Elbow is 10)
            False, False, True,                 # 11-13 (Wrist Flex is 13)
            False, False, True,                 # 14-16 (Wrist Roll is 16)
            False, False, False                 # 17-19 (Gripper is 19)
        ]

        # 2. Setup IK Chain using the clean URDF and the new mask
        my_arm_chain = ikpy.chain.Chain.from_urdf_file(
            "/home/sonu/so_arm100_ws/src/so_arm100_description/urdf/so_arm100_clean.urdf",
            active_links_mask=custom_mask,
            base_elements=["root"]
        )
        # NEW: Remove all mathematical joint limits to give the solver total freedom
        for link in my_arm_chain.links:
            link.bounds = (None, None)
        # 3. Calculate Inverse Kinematics
        ik_solution = my_arm_chain.inverse_kinematics(target_xyz)
        
        # 4. Pluck out ONLY the 5 active joint angles for Gazebo
        target_angles = [
            ik_solution[4],  # shoulder_pan
            ik_solution[7],  # shoulder_lift
            ik_solution[10], # elbow_flex
            ik_solution[13], # wrist_flex
            ik_solution[16]  # wrist_roll
        ]
        
        self.get_logger().info(f"Target Angles (rad): {target_angles}")

        # 5. Build the Trajectory Message
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_flex_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = target_angles
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(point)

        # 6. Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info("Message published! The arm should be moving.")

        # Print out the exact links ikpy found in the path
        self.get_logger().info(f"Total links found: {len(my_arm_chain.links)}")
        for i, link in enumerate(my_arm_chain.links):
            self.get_logger().info(f"Index {i}: {link.name}")

        # 3. Create a starting "hint" pose to prevent math singularities
        # This creates an array of 20 zeros (matching your chain length)
        initial_pose = [0.0] * len(my_arm_chain.links)
        # Add a slight 0.5 radian (~28 degree) bend to the shoulder and elbow
        initial_pose[7] = -0.5  
        initial_pose[10] = -0.5

        # Calculate Inverse Kinematics using the hint as a starting point
        ik_solution = my_arm_chain.inverse_kinematics(
            target_xyz,
            initial_position=initial_pose
         )

        # 4. Build the Trajectory Message
        msg = JointTrajectory()
        # These names must exactly match your ros2_controllers.yaml
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_flex_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = target_angles
        
        # Tell the arm to take exactly 2.0 seconds to reach the destination
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(point)

        # 5. Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info("Message published! The arm should be moving.")

def main(args=None):
    rclpy.init(args=args)
    commander = ArmCommander()

    try:
        # Prompt the user for coordinates in the terminal
        print("\n🎯 --- Enter Target Coordinates (in meters) ---")
        user_x = float(input("X (forward/back): "))
        user_y = float(input("Y (left/right): "))
        user_z = float(input("Z (up/down): "))
        
        user_target = [user_x, user_y, user_z]
        
        # Send the user's target to the IK solver
        commander.move_to_coordinate(user_target)

        # Keep the node alive just long enough to ensure the message is sent
        rclpy.spin_once(commander, timeout_sec=1.0)
        
    except ValueError:
        # This catches errors if someone accidentally types letters instead of numbers
        commander.get_logger().error("Invalid input! Please enter numbers only.")
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
