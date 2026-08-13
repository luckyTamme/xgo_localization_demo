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
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEMO_DIR = '/demo'

#: Raw inputs only. /joint_states and /cmd_vel are nearly free and leave the
#: door open for proprioceptive odometry later.
#:
#: /tf is deliberately absent. It is not an input: during a live run it carries
#: the backend's own ``map -> odom`` alongside our ``odom -> base_link``, so
#: recording it bakes one backend's answer into the bag. Replaying that put two
#: publishers on ``map -> odom`` — the recording and the live backend — which is
#: precisely the split-brain the node's design avoids, and it silently ruins the
#: one thing this bag format exists for: pushing the same inputs through a
#: different backend and comparing. Nothing is lost by dropping it, because the
#: stack regenerates both edges on replay: this node republishes
#: ``odom -> base_link`` from /imu/data and /xgo/applied_vel, and the backend
#: recomputes ``map -> odom`` from /scan.
#:
#: /tf_static stays: it is the sensor mount, a genuine input, and identical
#: whoever publishes it.
RECORD_TOPICS = [
    '/scan',
    '/imu/data',
    '/xgo/applied_vel',
    '/tf_static',
    '/joint_states',
    '/cmd_vel',
]
CAMERA_TOPICS = ['/camera/image_raw/compressed', '/camera/camera_info']


def _check_bag_destination(context, *args, **kwargs):
    """Refuse to start a recording that cannot be written.

    ``ros2 bag record -o`` will not write into a directory that already exists.
    It exits immediately, but only that one process dies: the drivers and the
    localisation stack come up regardless and the container sits there looking
    healthy, having recorded nothing. The failure surfaces as an empty bag
    directory much later, typically once the robot is back on the bench.

    Better to refuse the launch outright, while the person who asked for a
    recording is still watching.
    """
    if LaunchConfiguration('mode').perform(context) != 'live':
        return []
    if LaunchConfiguration('record').perform(context).lower() not in ('true', '1'):
        return []

    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    bag_name = LaunchConfiguration('bag_name').perform(context)
    destination = os.path.join(bag_dir, bag_name)
    if not os.path.exists(destination):
        return []

    raise RuntimeError(
        f'refusing to record: {destination} already exists and rosbag2 will not '
        f'write into it.\nMove or rename the previous run, or pass a different '
        f'name:\n    RECORD=true BAG_NAME=run2 docker compose --profile live up')


def generate_launch_description():
    localization_share = get_package_share_directory('xgo_localization')
    localization_launch = os.path.join(
        localization_share, 'launch', 'localization.launch.py')

    mode = LaunchConfiguration('mode')
    backend = LaunchConfiguration('backend')
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    loop = LaunchConfiguration('loop')
    record = LaunchConfiguration('record')
    record_camera = LaunchConfiguration('record_camera')
    camera = LaunchConfiguration('camera')
    bag_dir = LaunchConfiguration('bag_dir')
    bag_name = LaunchConfiguration('bag_name')

    is_live = IfCondition(PythonExpression(["'", mode, "' == 'live'"]))
    is_replay = IfCondition(PythonExpression(["'", mode, "' == 'replay'"]))
    live_camera = IfCondition(PythonExpression(
        ["'", mode, "' == 'live' and '", camera, "'.lower() in ('true', '1')"]))
    do_record = IfCondition(PythonExpression(
        ["'", mode, "' == 'live' and '", record, "'.lower() in ('true', '1')"]))

    # A bag is only replayable through a different backend later if it holds raw
    # inputs. The camera is excluded by default because it dominates the size by
    # an order of magnitude while contributing nothing to localisation.
    record_cmd = ['ros2', 'bag', 'record', '-o', [bag_dir, '/', bag_name],
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
        DeclareLaunchArgument('loop', default_value='false',
                              description='Restart the bag when it ends. Handy '
                                          'for demos; the odometry detects the '
                                          'backward time jump and resets.'),
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
        DeclareLaunchArgument('bag_name', default_value='run',
                              description='Name of the recording directory '
                                          'under bag_dir. Change it rather than '
                                          'overwriting a previous run.'),

        LogInfo(msg=['[demo] mode=', mode, '  backend=', backend]),

        # Before anything starts, so a doomed recording fails while someone is
        # still watching rather than after a session that cannot be repeated.
        OpaqueFunction(function=_check_bag_destination),

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
                msg=['[demo] recording raw inputs to ', bag_dir, '/', bag_name,
                     '  (camera included: ', record_camera, ')']),

        # --- replay ----------------------------------------------------------
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                ["'", mode, "' == 'replay' and not '", loop,
                 "'.lower() in ('true','1')"])),
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
            output='screen',
        ),

        # Looping deliberately does NOT use `ros2 bag play --loop`. That rewinds
        # the clock under a SLAM backend that has no concept of going backwards:
        # cartographer only ever advances its trajectory time, so after the first
        # rewind it drops every scan as already-seen and never emits another
        # `map -> odom`. The display then shows a frozen map and permanently
        # degraded diagnostics — worse, it looks like a localisation bug.
        #
        # Instead each pass is a whole fresh run: play once, shut the launch
        # down, and let the container's restart policy start the next one with an
        # empty backend. Slower to come round (a few seconds of restart between
        # passes) and completely honest — every pass localises from scratch
        # exactly as a single pass does, so an unattended display keeps showing
        # a healthy stack rather than a wedged one.
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                ["'", mode, "' == 'replay' and '", loop,
                 "'.lower() in ('true','1')"])),
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
            output='screen',
            on_exit=[
                LogInfo(msg='[demo] bag finished; restarting for the next pass'),
                Shutdown(reason='replay loop: restarting for a clean pass'),
            ],
        ),
    ])
