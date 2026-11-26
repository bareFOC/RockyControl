//! Additional sample URDF models exercising standard ROS Z-Up orientation,
//! top-level `<material>` definitions, cylinders, spheres, and prismatic joints.

/// Standard ROS Z-Up 8-DoF compact fighting biped URDF.
/// Uses top-level `<material>` tags, cylinders aligned on local Z, spheres, and boxes.
pub const STRIKER_BIPED_ZUP_URDF: &str =
    include_str!("../../../assets/robots/striker_biped.urdf");

/// 6-DoF ROS Z-Up robotic arm with revolute joints and a prismatic linear gripper slide.
pub const COMBAT_ARM_6DOF_URDF: &str = r#"<?xml version="1.0"?>
<robot name="combat_arm_6dof">
  <material name="steel_base"><color rgba="0.20 0.24 0.30 1.0"/></material>
  <material name="arm_orange"><color rgba="0.86 0.36 0.16 1.0"/></material>
  <material name="joint_dark"><color rgba="0.14 0.16 0.20 1.0"/></material>
  <material name="claw_teal"><color rgba="0.15 0.72 0.78 1.0"/></material>

  <link name="pedestal">
    <visual>
      <origin xyz="0 0 0.06" rpy="0 0 0"/>
      <geometry><cylinder radius="0.14" length="0.12"/></geometry>
      <material name="steel_base"/>
    </visual>
  </link>

  <link name="turret">
    <visual>
      <origin xyz="0 0 0.07" rpy="0 0 0"/>
      <geometry><cylinder radius="0.10" length="0.14"/></geometry>
      <material name="arm_orange"/>
    </visual>
  </link>
  <joint name="base_yaw" type="revolute">
    <parent link="pedestal"/>
    <child link="turret"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="30.0" velocity="5.0"/>
  </joint>

  <link name="upper_boom">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="0.055" length="0.11"/></geometry>
      <material name="joint_dark"/>
    </visual>
    <visual>
      <origin xyz="0 0 0.19" rpy="0 0 0"/>
      <geometry><box size="0.08 0.08 0.36"/></geometry>
      <material name="arm_orange"/>
    </visual>
  </link>
  <joint name="shoulder_pitch" type="revolute">
    <parent link="turret"/>
    <child link="upper_boom"/>
    <origin xyz="0 0 0.14" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.65" upper="1.65" effort="28.0" velocity="5.0"/>
  </joint>

  <link name="forearm_boom">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="0.048" length="0.10"/></geometry>
      <material name="joint_dark"/>
    </visual>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry><box size="0.068 0.068 0.28"/></geometry>
      <material name="steel_base"/>
    </visual>
  </link>
  <joint name="elbow_pitch" type="revolute">
    <parent link="upper_boom"/>
    <child link="forearm_boom"/>
    <origin xyz="0 0 0.37" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.10" upper="2.10" effort="22.0" velocity="5.0"/>
  </joint>

  <link name="wrist_pitch_link">
    <visual>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
      <geometry><sphere radius="0.045"/></geometry>
      <material name="joint_dark"/>
    </visual>
  </link>
  <joint name="wrist_pitch" type="revolute">
    <parent link="forearm_boom"/>
    <child link="wrist_pitch_link"/>
    <origin xyz="0 0 0.30" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.80" upper="1.80" effort="14.0" velocity="6.0"/>
  </joint>

  <link name="wrist_roll_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry><cylinder radius="0.04" length="0.09"/></geometry>
      <material name="arm_orange"/>
    </visual>
  </link>
  <joint name="wrist_roll" type="continuous">
    <parent link="wrist_pitch_link"/>
    <child link="wrist_roll_link"/>
    <origin xyz="0 0 0.07" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10.0" velocity="8.0"/>
  </joint>

  <link name="gripper_jaw">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry><box size="0.04 0.14 0.08"/></geometry>
      <material name="claw_teal"/>
    </visual>
  </link>
  <joint name="gripper_slide" type="prismatic">
    <parent link="wrist_roll_link"/>
    <child link="gripper_jaw"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0.0" upper="0.10" effort="12.0" velocity="0.5"/>
  </joint>
</robot>
"#;
