import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('robotic_planning')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    empty_box_cmd = Node(
        package='robotic_planning',
        executable='empty_box',
        name='empty_box',
        namespace=namespace,
        output='screen',
        parameters=[])

    fill_box_cmd = Node(
        package='robotic_planning',
        executable='fill_box',
        name='fill_box',
        namespace=namespace,
        output='screen',
        parameters=[])

    load_carrier_cmd = Node(
        package='robotic_planning',
        executable='load_carrier',
        name='load_carrier',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_with_carrier_cmd = Node(
        package='robotic_planning',
        executable='move_with_carrier',
        name='move_with_carrier',
        namespace=namespace,
        output='screen',
        parameters=[])
   
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(empty_box_cmd)
    ld.add_action(fill_box_cmd)
    ld.add_action(load_carrier_cmd)
    ld.add_action(move_with_carrier_cmd)

    return ld