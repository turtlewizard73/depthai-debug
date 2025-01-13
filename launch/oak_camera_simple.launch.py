from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch.launch_context import LaunchContext
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile, ComposableNode

# from .../depthai-ros/depthai_ros_driver/launch/camera.launch.py

def launch_setup(context: LaunchContext, *args, **kwargs):
    CONFIG_DIR = PathJoinSubstitution(
        [FindPackageShare('service_robot_bringup'), 'config/hardware'])

    depthai_debug = LaunchConfiguration('depthai_debug')

    use_gdb = LaunchConfiguration('use_gdb', default='false')
    use_valgrind = LaunchConfiguration('use_valgrind', default='false')
    use_perf = LaunchConfiguration('use_perf', default='false')

    log_level = 'debug' if depthai_debug.perform(context) == '1' else 'info'

    launch_prefix = ''
    if (use_gdb.perform(context) == 'true'):
        launch_prefix += "gdb -ex run --args"
    if (use_valgrind.perform(context) == 'true'):
        launch_prefix += "valgrind --tool=callgrind"
    if (use_perf.perform(context) == 'true'):
        launch_prefix += "perf record -g --call-graph dwarf --output=perf.out.node_name.data --"

    container_name = 'marker_detection_container'
    camera_name = 'oak_top'

    oak_params = ParameterFile(PathJoinSubstitution(
        [CONFIG_DIR, 'oak_cameras.yaml']), allow_substs=True)

    return [
        Node(
            package='rclcpp_components',
            executable='component_container_mt',
            name=container_name,
            output='both',
            arguments=['--ros-args', '--log-level', log_level],
            prefix=[launch_prefix],
        ),

        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=camera_name,
                    namespace='camera',
                    parameters=[oak_params],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'simulation', default_value='false'),
        DeclareLaunchArgument(
            'depthai_debug', default_value=EnvironmentVariable('DEPTHAI_DEBUG')),
        DeclareLaunchArgument(
            'use_gdb', default_value='false'),
        DeclareLaunchArgument(
            'use_valgrind', default_value='false'),
        DeclareLaunchArgument(
            'use_perf', default_value='false')
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
