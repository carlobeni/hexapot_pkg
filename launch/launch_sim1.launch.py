import os

from ament_index_python.packages import get_package_share_directory
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='hexapod_pkg' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )



    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacles3.world'
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
                                   '-name', 'hexapod_pkg',
                                   '-z', '0.02'],
                        output='screen')


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )




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

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )


    heading_estimator = Node(
    package='hexapod_pkg',
    executable='noise_compute_heading.py',
    name='heading_estimator',
    output='screen',
    )
    # OBS: Calcula el heading

    gps_to_local_xy = Node(
    package='hexapod_pkg',
    executable='noise_gps_to_local_xy.py',
    name='gps_to_local_xy',
    output='screen',
    )
    # OBS: Coordenadas metricas respecto al LAR donde la X+ es el polo magnetico con +Y


    corrected_stimate_xy = Node(
    package='hexapod_pkg',
    executable='corrected_stimate_xy.py',
    name='corrected_stimate_xy',
    output='screen',
    )
    # OBS: Partiendo de una posicion inicial, calcula la posicion actual en (x,y)

    # cinematica inversa
    hexapod_low_level_control = Node(
    package='hexapod_pkg',
    executable='hexapod_low_level_control.py',
    name='hexapod_low_level_control',
    output='screen'
    )
    # OBS: El robot se mueve publicando en hl/cmd tipic de control

    ultrasonic_range = Node(
    package='hexapod_pkg',
    executable='ultrasonic_range.py',
    name='ultrasonic_range',
    output='screen'
    )
    # OBS: Transorma del RANGE a longitudes del ultrasonico en m, si no detecta nada manda -1

    ir_emulator = Node(
    package='hexapod_pkg',
    executable='ir_gazebo_ros_emulator.py',
    name='ir_emulator',
    output='screen'
    )
    # OBS: Emula los sensores IR y publica sus estados



    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,
        heading_estimator,
        gps_to_local_xy,
        corrected_stimate_xy,
        ultrasonic_range,
        ir_emulator,
        hexapod_low_level_control
    ])