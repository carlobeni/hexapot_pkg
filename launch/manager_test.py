import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node



def generate_launch_description():

    package_name='hexapot_pkg' 

    # =====================================================
    # LIMPIEZA PREVIA
    # =====================================================
    cleanup = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "pkill -f sensors_fast_listener_node.py || true; "
            "pkill -f sensors_reliable_listener_node.py || true; "
            "pkill -f command_talker_node.py || true; "
            "pkill -f monitor_node_pc.py || true; "
            "pkill -f teleop_twist_keyboard || true; "
            "pkill -f cmd_vel_relay_node.py || true"
        ],
        output="screen"
    )
    # =====================================================
    # SIMULACIÓN
    # =====================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "rsp.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_ros2_control": "true"
        }.items(),
    )

    # Define world
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'green_space.world'
        )    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'hexapot_pkg',
                                   '-z', '0.1'],
                        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Launch ros_gz_bridge (ROS <-> Gazebo)
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Launch ros_gz_image_bridge (ROS2 <-> Gazebo virtual camera)
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    # =====================================================
    # LAUNCH DE JOYSTICK
    # =====================================================

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "joystick.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # (Se mantiene tu twist_mux, aunque ya no es crítico para mover)
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "twist_mux.yaml"
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        output="screen",
    )

    # =====================================================
    # TELEOP (UNA SOLA TERMINAL)
    # Publica en /cmd_vel (default)
    # =====================================================
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop",
        output="screen",
        prefix=["xterm -hold -e"],
        # sin remaps: teleop publica /cmd_vel
    )

    # =====================================================
    # RELAY: /cmd_vel  ->  /cmd_vel_robot  y  /diff_cont/cmd_vel_unstamped
    # =====================================================
    cmd_vel_relay = Node(
        package="hexapot_pkg",
        executable="cmd_vel_relay_node.py",
        name="cmd_vel_relay",
        output="screen",
        parameters=[{
            "input_topic": "/cmd_vel",
            #"out_robot_topic": "/cmd_vel_robot",
            "out_sim_topic": "/diff_cont/cmd_vel_unstamped",
        }],
    )

    # =====================================================
    # EVENTO HANDLER: LANZAR CONTROLES CUANDO EL ROBOT YA EXISTE
    # =====================================================
    start_comm_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                teleop,
                cmd_vel_relay,  
            ],
        )
    )



    # Launch them all!
    return LaunchDescription([

        rsp,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,

        start_comm_after_spawn,
    ])