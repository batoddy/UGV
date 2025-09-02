from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fcu = DeclareLaunchArgument(
        'fcu_url', 
        default_value='serial:///dev/ttyACM0:115200'
    )
    gcs = DeclareLaunchArgument(
        'gcs_url',  
        default_value=''
    )
    tgt = DeclareLaunchArgument(
        'tgt_system', 
        default_value='1'
    )
    comp = DeclareLaunchArgument(
        'tgt_component', 
        default_value='1'
    )

    return LaunchDescription([
        fcu, 
        gcs, 
        tgt, 
        comp,
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'system_id': 1,
                'component_id': 1,
                'target_system_id': LaunchConfiguration('tgt_system'),
                'target_component_id': LaunchConfiguration('tgt_component'),
                'plugin_blacklist': [''],
            }]
        )
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