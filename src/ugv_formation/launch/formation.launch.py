import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

TARGET_RADIUS = 4.0
NUM_ROBOTS = 3
ROBOT_IDS = [1, 2, 3]
ROBOT_COLOURS = {
    1: (0.85, 0.15, 0.15),
    2: (0.15, 0.75, 0.15),
    3: (0.15, 0.45, 0.95),
}


def formation_pose(robot_id: int):
    phase = (robot_id - 1) * (2.0 * math.pi / NUM_ROBOTS)
    x = TARGET_RADIUS * math.cos(phase)
    y = TARGET_RADIUS * math.sin(phase)
    yaw = phase + math.pi / 2.0
    return x, y, yaw


URDF_TEMPLATE = """\
<?xml version="1.0"?>
<robot name="ugv02">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.047" rpy="0 0 0"/>
      <geometry><box size="0.252 0.230 0.094"/></geometry>
      <material name="mat_{ns}">
        <color rgba="{r} {g} {b} 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.047" rpy="0 0 0"/>
      <geometry><box size="0.252 0.230 0.094"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.047" rpy="0 0 0"/>
      <mass value="3.402"/>
      <inertia ixx="0.01750" ixy="0.0" ixz="0.0" iyy="0.02050" iyz="0.0" izz="0.03299"/>
    </inertial>
  </link>
  <gazebo reference="base_link">
    <material>Gazebo/DarkGrey</material>
    <mu1>0.9</mu1><mu2>0.9</mu2>
    <kp>500000.0</kp><kd>50.0</kd>
    <minDepth>0.001</minDepth><maxVel>1.0</maxVel>
  </gazebo>
  <gazebo>
    <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/{ns}</namespace>
      </ros>
      <update_rate>50</update_rate>
      <publish_rate>50</publish_rate>
      <robot_base_frame>base_link</robot_base_frame>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <publish_odometry_tf>false</publish_odometry_tf>
    </plugin>
  </gazebo>
</robot>
"""


def make_urdf(robot_id: int) -> str:
    ns = f"ugv{robot_id}"
    r, g, b = ROBOT_COLOURS[robot_id]
    return URDF_TEMPLATE.format(ns=ns, r=r, g=g, b=b)


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": "", "verbose": "false", "pause": "false"}.items(),
    )

    actions: list = [gazebo]
    urdf_files: dict[int, str] = {}

    for rid in ROBOT_IDS:
        tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=f"_ugv{rid}.urdf", delete=False
        )
        tmp.write(make_urdf(rid))
        tmp.flush()
        tmp.close()
        urdf_files[rid] = tmp.name

    for rid in ROBOT_IDS:
        x, y, yaw = formation_pose(rid)
        ns = f"ugv{rid}"
        urdf_path = urdf_files[rid]

        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"rsp_{ns}",
            namespace=ns,
            output="screen",
            parameters=[{"robot_description": make_urdf(rid)}],
        )

        spawn = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name=f"spawn_{ns}",
            output="screen",
            arguments=[
                "-entity",
                ns,
                "-robot_namespace",
                ns,
                "-file",
                urdf_path,
                "-x",
                str(round(x, 5)),
                "-y",
                str(round(y, 5)),
                "-z",
                "0.05",
                "-Y",
                str(round(yaw, 5)),
            ],
        )

        controller = Node(
            package="ugv_formation",
            executable="consensus_node",
            name=f"consensus_{ns}",
            output="screen",
            parameters=[
                {
                    "robot_id": rid,
                    "target_radius": TARGET_RADIUS,
                    "v_base": 0.5,
                    "k_radial": 0.8,
                    "k_spacing": 0.6,
                    "k_heading": 1.2,
                    "max_v": 1.0,
                    "max_w": 1.5,
                }
            ],
        )

        actions.append(TimerAction(period=2.0 + (rid - 1) * 1.5, actions=[rsp, spawn]))
        actions.append(TimerAction(period=9.0 + (rid - 1) * 0.5, actions=[controller]))

    return LaunchDescription(actions)
