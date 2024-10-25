import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'dualarm'
    urdf_name = "dualarm.urdf"

    # 获取包共享路径
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)

    # 读取URDF文件内容
    with open(urdf_model_path, 'r') as infp:
        robot_desc = infp.read()

    # 设置robot_state_publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 设置joint_state_publisher_gui节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # # 启动自定义节点
    # joint_state_publisher_node = Node(
    #     package='robot_control',
    #     executable='joint_pub_node',
    #     name='joint_pub_node',
    #     output='screen'
    # )

    # 设置RVIZ2节点
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", get_package_share_directory(package_name) + "/rviz/rviz_config.rviz"]
    )

    # 创建LaunchDescription并添加节点
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld

