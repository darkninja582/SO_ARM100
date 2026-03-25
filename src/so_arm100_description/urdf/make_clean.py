import xml.etree.ElementTree as ET

# 1. Load your complex Onshape URDF
tree = ET.parse('phosphobot_so_100.urdf')
root = tree.getroot()

# 2. The exact "straight line" path from the root to the gripper
valid_joints = [
    "fixed_node_to_root_joint_0", "Fastened 3_1", "Fastened 2_1", "shoulder_pan_joint",
    "Fastened 7_0", "Fastened 3_4", "shoulder_lift_joint",
    "Fastened 10_0", "Fastened 3_6", "elbow_flex_joint",
    "Fastened 12_0", "Fastened 3_0", "wrist_flex_joint",
    "Fastened 15_0", "Fastened 3_5", "wrist_roll_joint",
    "Fastened 17_0", "Fastened 3_3", "gripper_joint"
]

# 3. Snip off any dead-end branches (screws, passive horns)
for joint in root.findall('joint'):
    if joint.attrib['name'] not in valid_joints:
        root.remove(joint)

# 4. Save the new, mathematically perfect serial chain
tree.write('so_arm100_clean.urdf')
print("so_arm100_clean.urdf successfully created! ✂️🌳")
