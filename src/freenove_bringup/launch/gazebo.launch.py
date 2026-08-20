import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_freenove_description = get_package_share_directory('freenove_description')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_freenove_description, '..')]
    )
    
    urdf_file = os.path.join(pkg_freenove_description, 'urdf', 'robot_gazebo.urdf')

    rviz_config_file = os.path.join(pkg_freenove_description, 'rviz', 'view_robot.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read().replace(
            '$(find freenove_description)', 
            pkg_freenove_description
        )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'hexapod',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',       
            '-R', '1.5708',
            '-P', '0.0',      
            '-Y', '0.0'        
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/ultrasonic/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_controller"],
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ultrasonic_simulation = Node(
        package='freenove_simulation',
        executable='ultrasonic_simulation',
        name='ultrasonic_simulation',
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        joint_state_broadcaster_spawner,
        leg_controller_spawner,
        ultrasonic_simulation,
        rviz_node
    ])