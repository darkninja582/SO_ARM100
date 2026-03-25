============================================================
       SO-ARM100 PICK AND PLACE SIMULATION - RUN GUIDE
============================================================

PREREQUISITES - Run Once
----------------------------------------------------------
# Source ROS 2 (add to ~/.bashrc so you don't repeat)
source /opt/ros/humble/setup.bash
source ~/so_arm100_ws/install/setup.bash


============================================================
STEP 1: Visualize Robot in RViz
============================================================
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/so_arm100_ws/install/setup.bash
ros2 launch so_arm100_description display.launch.py

In RViz:
- Set Fixed Frame to: Base_08q
- Click Add → RobotModel
- Set Description Topic to: /robot_description
- Use joint sliders to verify joints move correctly


============================================================
STEP 2: Launch Gazebo Simulation
============================================================
# Kill any leftover processes first
pkill -f gzserver; pkill -f gzclient; sleep 2

# Terminal 1
source /opt/ros/humble/setup.bash
source ~/so_arm100_ws/install/setup.bash
ros2 launch so_arm100_description gazebo.launch.py

Wait until you see:
    Successfully spawned entity [so_arm100]

This automatically:
- Launches Gazebo with table + 3 colored cubes
- Spawns SO-ARM100 robot on the table
- Starts joint_state_broadcaster, arm_controller, gripper_controller


============================================================
STEP 3: Run Blue Cube Detector
============================================================
# Terminal 2
source /opt/ros/humble/setup.bash
source ~/so_arm100_ws/install/setup.bash
ros2 run so_arm100_perception blue_cube_detector

Expected output:
    [INFO] Blue Cube Detector started!
    [INFO] Blue cube at world: x=0.265, y=0.368


============================================================
STEP 4: View Camera Debug Feed
============================================================
# Terminal 3
source /opt/ros/humble/setup.bash
source ~/so_arm100_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view

- Select /blue_cube_debug from dropdown
- Left side: camera view with green box around blue cube
- Right side: binary mask showing detected blue pixels


============================================================
STEP 5: Monitor Blue Cube Position
============================================================
# Terminal 4
source /opt/ros/humble/setup.bash
source ~/so_arm100_ws/install/setup.bash
ros2 topic echo /blue_cube_position

Expected output:
    header:
      frame_id: world
    point:
      x: 0.265
      y: 0.368
      z: 0.4
============================================================
STEP 6: clean urdf
============================================================

Run the Interactive IK Commander
Leave Gazebo running. Go back to your first terminal (Terminal 2) where you built the workspace, and run the movement script.


ros2 run so_arm100_control move_arm

Step 5: Enter Target Coordinates 
When prompted in the terminal, enter the target coordinates in meters.

Important Note on Coordinate Frames:
The IK solver treats the base of the robot as Z = 0.0. If your robot is sitting on a table in Gazebo that is 0.40m tall, and you want to reach a cube sitting on that table at 0.05m tall, your relative Z target for this script should be 0.05, not 0.45.

Safe Test Coordinate (Straight Forward):

X: 0.15

Y: 0.0

Z: 0.15


============================================================
IF SOMETHING GOES WRONG
============================================================
# Kill everything and restart
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f ros2
sleep 3

# Rebuild workspace
cd ~/so_arm100_ws
colcon build --symlink-install
source install/setup.bash

# Relaunch
ros2 launch so_arm100_description gazebo.launch.py

============================================================
