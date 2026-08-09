"""Demo entry point: run xgo_localization live on the robot, or replay a bag.

    ros2 launch /demo/launch/demo.launch.py mode:=live
    ros2 launch /demo/launch/demo.launch.py mode:=live record:=true
    ros2 launch /demo/launch/demo.launch.py mode:=replay bag:=/bag

`live` starts the hardware drivers; `replay` plays a bag instead. Either way the
localisation stack and the Foxglove bridge come up, so Lichtblick sees the same
topics in both modes.

Recording captures raw sensor inputs only, never the backend's output — that is
what makes a bag replayable through either SLAM backend afterwards.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEMO_DIR = '/demo'

#: Raw inputs only. /joint_states and /cmd_vel are nearly free and leave the
#: door open for proprioceptive odometry later.
RECORD_TOPICS = [
    '/scan',
    '/imu/data',
    '/xgo/applied_vel',
    '/tf',
    '/tf_static',
    '/joint_states',
    '/cmd_vel',
]
CAMERA_TOPICS = ['/camera/image_raw/compressed', '/camera/camera_info']


def generate_launch_description():
    localization_share = get_package_share_directory('xgo_localization')
    localization_launch = os.path.join(
        localization_share, 'launch', 'localization.launch.py')

    mode = LaunchConfiguration('mode')
    backend = LaunchConfiguration('backend')
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    record = LaunchConfiguration('record')
    record_camera = LaunchConfiguration('record_camera')
    camera = LaunchConfiguration('camera')
    bag_dir = LaunchConfiguration('bag_dir')

    is_live = IfCondition(PythonExpression(["'", mode, "' == 'live'"]))
    is_replay = IfCondition(PythonExpression(["'", mode, "' == 'replay'"]))
    live_camera = IfCondition(PythonExpression(
        ["'", mode, "' == 'live' and '", camera, "'.lower() in ('true', '1')"]))
    do_record = IfCondition(PythonExpression(
        ["'", mode, "' == 'live' and '", record, "'.lower() in ('true', '1')"]))

    # A bag is only replayable through a different backend later if it holds raw
    # inputs. The camera is excluded by default because it dominates the size by
    # an order of magnitude while contributing nothing to localisation.
    record_cmd = ['ros2', 'bag', 'record', '-o', [bag_dir, '/run'],
                  '--qos-profile-overrides-path',
                  os.path.join(DEMO_DIR, 'config', 'qos_override.yaml')]
    record_cmd += RECORD_TOPICS

    record_cmd_with_camera = list(record_cmd) + CAMERA_TOPICS

    with_camera = PythonExpression(
        ["'", record_camera, "'.lower() in ('true', '1')"])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='live',
                              choices=['live', 'replay'],
                              description='Drive the hardware, or replay a bag.'),
        DeclareLaunchArgument('backend', default_value='cartographer',
                              choices=['cartographer', 'rtabmap'],
                              description='SLAM backend.'),
        DeclareLaunchArgument('bag', default_value='/bag',
                              description='Bag to replay. mode:=replay only.'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Replay speed multiplier.'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Record raw inputs while running live.'),
        DeclareLaunchArgument('record_camera', default_value='false',
                              description='Include the camera in the recording. '
                                          'Roughly ten times the bag size.'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Run the camera so Lichtblick can show '
                                          'it. Independent of record_camera.'),
        DeclareLaunchArgument('bag_dir', default_value='/bags',
                              description='Where recordings are written.'),

        LogInfo(msg=['[demo] mode=', mode, '  backend=', backend]),

        # --- localisation, in both modes -----------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'backend': backend,
                # Replay runs on the bag clock; live runs on the wall clock.
                # This has to agree across every node or timestamps silently
                # disagree and the pose goes wrong without erroring.
                'use_sim_time': PythonExpression(
                    ["'true' if '", mode, "' == 'replay' else 'false'"]),
            }.items(),
        ),

        # --- viewer bridge, in both modes ----------------------------------
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'use_sim_time': PythonExpression(
                    ["'true' if '", mode, "' == 'replay' else 'false'"]),
            }],
        ),

        # --- live hardware --------------------------------------------------
        GroupAction([
            Node(
                package='xgo2_ros',
                executable='xgo2_ros_node',
                name='xgo2_ros_node',
                output='screen',
                parameters=[{
                    'config_path': os.path.join(DEMO_DIR, 'config', 'xgo_config.json'),
                }],
            ),
            # Resolved lazily. get_package_share_directory() here would run at
            # parse time and blow up in the core image, which has no LiDAR
            # driver — a condition gates execution, not evaluation.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('ldlidar_stl_ros2'),
                    'launch', 'ld19.launch.py',
                ])),
            ),
        ], condition=is_live),

        Node(
            condition=live_camera,
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'width': 640,
                'height': 480,
                'format': 'RGB888',
                'FrameDurationLimits': [66666, 66666],   # ~15 fps
            }],
        ),

        # --- recording, live only -------------------------------------------
        # The QoS override is not optional. /xgo/applied_vel is latched and
        # published only when a command changes; a default subscription misses
        # it, and the recording then looks fine but replays as a robot that
        # never moved.
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                ["'", mode, "' == 'live' and '", record,
                 "'.lower() in ('true','1') and not '", record_camera,
                 "'.lower() in ('true','1')"])),
            cmd=record_cmd, output='screen',
        ),
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                ["'", mode, "' == 'live' and '", record,
                 "'.lower() in ('true','1') and '", record_camera,
                 "'.lower() in ('true','1')"])),
            cmd=record_cmd_with_camera, output='screen',
        ),
        LogInfo(condition=do_record,
                msg=['[demo] recording raw inputs to ', bag_dir,
                     '/run  (camera included: ', record_camera, ')']),

        # --- replay ----------------------------------------------------------
        ExecuteProcess(
            condition=is_replay,
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
            output='screen',
        ),
    ])
