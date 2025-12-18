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
    
    # =====================================================
    # NODOS Y PRELAUNCHERS DE GAZEBO
    # =====================================================

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

    # Emula y gestiona el sensor IR de la simulacion
    gz_ir_emulator = Node(
    package='hexapod_pkg',
    executable='gz_ir_emulator.py',
    name='gz_ir_emulator',
    output='screen'
    )

    # Gestiona la cinematica inversa del hexapodo de la simulacion
    # subsbriver topics:
    # pubbrizer topics:
    gz_hexapod_inverse_kinematics = Node(
    package='hexapod_pkg',
    executable='gz_hexapod_inv_kinematics.py',
    name='gz_hexapod_inv_kinematics',
    output='screen'
    )

    # =====================================================
    # NODOS DE GESTION DE LAS SENSORES
    # =====================================================
    # Gestiona el sensor ultrasonico con
    # -1: no detecta nada
    # valor positivo: distancia a la que se le asigna el rango
    # subsbriver topics:
    # pubbrizer topics:
    sensor_ultrasonic = Node(
    package='hexapod_pkg',
    executable='sensor_ultrasonic.py',
    name='sensor_ultrasonic',
    output='screen'
    )

    # =====================================================
    # NODOS DE COMPUTO
    # =====================================================

    # Calcula la posicion del robot respecto a un punto definido
    #empleando unicamente el GPS
    # subsbriver topics:
    # pubbrizer topics:
    compute_gps_to_local_xy = Node(
    package='hexapod_pkg',
    executable='compute_gps_to_local_xy.py',
    name='compute_gps_to_local_xy',
    output='screen',
    )

    # Calcula la posicion de un robot respecto a un punto definido
    #empleando el GPS de la posicion inical, el heading y una 
    #velocidad constante
    # subsbriver topics:
    # pubbrizer topics:
    compute_stimate_xy = Node(
    package='hexapod_pkg',
    executable='compute_stimate_xy.py',
    name='compute_stimate_xy',
    output='screen',
    )

    # Calcula el heading aplicando filtro Kalman
    # subsbriver topics:
    # pubbrizer topics:
    compute_heading = Node(
    package='hexapod_pkg',
    executable='compute_heading.py',
    name='compute_heading',
    output='screen',
    )

    # =====================================================
    # NODOS MANAGER
    # =====================================================
    # In developments

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
        gz_ir_emulator,
        gz_hexapod_inverse_kinematics,

        sensor_ultrasonic,

        compute_heading,
        compute_gps_to_local_xy,
        compute_stimate_xy,
    ])