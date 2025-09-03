from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch argümanları
    fcu_url = DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:57600')
    gcs_url = DeclareLaunchArgument('gcs_url', default_value='')
    tgt_system = DeclareLaunchArgument('tgt_system', default_value='1')
    tgt_component = DeclareLaunchArgument('tgt_component', default_value='1')
    pluginlists_yaml = DeclareLaunchArgument(
        'pluginlists_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('mavros'),
            'launch',
            'px4_pluginlists.yaml'
        ])
    )
    config_yaml = DeclareLaunchArgument(
        'config_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('mavros'),
            'launch',
            'px4_config.yaml'
        ])
    )
    log_output = DeclareLaunchArgument('log_output', default_value='screen')
    fcu_protocol = DeclareLaunchArgument('fcu_protocol', default_value='v2.0')
    respawn_mavros = DeclareLaunchArgument('respawn_mavros', default_value='false')
    namespace = DeclareLaunchArgument('namespace', default_value='mavros')

    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=LaunchConfiguration('namespace'),
        output=LaunchConfiguration('log_output'),
        parameters=[
            {'fcu_url': LaunchConfiguration('fcu_url')},
            {'gcs_url': LaunchConfiguration('gcs_url')},
            {'target_system_id': LaunchConfiguration('tgt_system')},
            {'target_component_id': LaunchConfiguration('tgt_component')},
            {'fcu_protocol': LaunchConfiguration('fcu_protocol')},
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4_pluginlists.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4_config.yaml'
            ])
        ]
    )

    return LaunchDescription([
        fcu_url,
        gcs_url,
        tgt_system,
        tgt_component,
        pluginlists_yaml,
        config_yaml,
        log_output,
        fcu_protocol,
        respawn_mavros,
        namespace,
        mavros_node
    ])




""" # IMU
/mavros/imu/data                (sensor_msgs/Imu)      # fused orientation
/mavros/imu/data_raw            (sensor_msgs/Imu)      # raw accel/gyro
/mavros/imu/temperature
/mavros/imu/mag

# GPS / Global position
/mavros/global_position/global  (sensor_msgs/NavSatFix)
/mavros/global_position/raw/fix (sensor_msgs/NavSatFix)
/mavros/global_position/rel_alt (std_msgs/Float64)
/mavros/home_position/home

# Local position / attitude
/mavros/local_position/pose     (geometry_msgs/PoseStamped)
/mavros/local_position/velocity_local (TwistStamped)
/mavros/attitude/roll_pitch_yaw
/mavros/attitude/target

# State / Battery / RC
/mavros/state                   (mavros_msgs/State)
/mavros/battery                 (sensor_msgs/BatteryState)
/mavros/rc/in                   (mavros_msgs/RCIn)
/mavros/rc/out                  (mavros_msgs/RCOut)

# Velocity / Odometry (varsa)
/mavros/odometry/in             (nav_msgs/Odometry)   # offboard için
/mavros/odometry/out            (nav_msgs/Odometry)

# TF (bazı kurulumlarda)
/tf, /tf_static  (robot_state_publisher ile birlikte) """