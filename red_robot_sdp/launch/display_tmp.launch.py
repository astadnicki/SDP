import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='red_robot_sdp').find('red_robot_sdp')
    default_model_path = os.path.join(pkg_share, 'src/description/red_robot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    robot_localization_file_path = os.path.join(pkg_share, 'params/ekf_with_gps.yaml') 
    use_sim_time = LaunchConfiguration('use_sim_time')
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory(
        'nav2_gps_waypoint_follower_demo')
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, 'config')
    nav2_params = os.path.join(params_dir, 'nav2_params.yaml')
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )
    #nmea_config_file = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config","nmea_serial_driver.yaml") #for navsat driver node
    nmea_dir = get_package_share_directory('nmea_navsat_driver')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    #Waypoint follower run file
    waypoint_follower_node = launch_ros.actions.Node(
        package='nav2_gps_waypoint_follower_demo',
        executable='logged_waypoint_follower',
        name='waypoint_follower'
    )
    #Navsat node launch
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )
    #nav2 node launch
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )
    #nmea navsat node
    nmea_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nmea_dir, 'launch', 'navigation_launch.py')
        ),
    )
    #rplidar_node launch
    rplidar_ros_node = launch_ros.actions.Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                #'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Boost',
            }]
     )
     
    laser_odom_node = launch_ros.actions.Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}]
     )
     
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')



    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Use simulation/Gazebo clock'),
        launch.actions.DeclareLaunchArgument(name='slam_params_file', default_value=os.path.join(get_package_share_directory("slam_toolbox"),'config', 'mapper_params_online_async.yaml'),  description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
                                       
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        #navigation2_cmd,
        #robot_localization_cmd,
        waypoint_follower_node,
        #navsat_node,
        
        #start_async_slam_toolbox_node,
        
        #start_navsat_transform_node,
        #start_robot_localization_local_node,
        #start_robot_localization_global_node,
        #start_ekf_filter_node_odom,
        #start_ekf_filter_node_map, 
        #start_robot_localization_node, 
        
        rplidar_ros_node,
        laser_odom_node,
       
    ])
    ld.addaction(nmea_cmd)
    ld.addaction(robot_localization_cmd)
    ld.addaction(navigation2_cmd)   
    ld.addaction(robot_localization_cmd)

    return ld
