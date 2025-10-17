from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    obj_num_arg = DeclareLaunchArgument(
        'obj_num',
        default_value='0', # 内存优化：减小地图大小，减少内存使用 10->0，关闭移动对象检测和轨迹处理
        description='Number of moving objects'
    )
    
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='Drone ID'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.15',  # 内存优化：减小地图大小，因为地图减小，分辨率可以适当增大，基于原先的0.4分辨来说：0.25导致资源消耗增加了一些，0.3的资源消耗减少了
        description='高精度分辨率，用于真机实验'
    )

    map_size_x_arg = DeclareLaunchArgument(
        'map_size_x',
        default_value='20.0', # 内存优化：减小地图大小，减少内存使用 100->20
        description='4 size in X direction'
    )

    map_size_y_arg = DeclareLaunchArgument(
        'map_size_y',
        default_value='10.0', # 内存优化：减小地图大小，减少内存使用 50->10
        description='Map size in Y direction'
    )

    map_size_z_arg = DeclareLaunchArgument(
        'map_size_z',
        default_value='1.5', # 内存优化：减小地图大小，减少内存使用 3.0->1.5
        description='Map size in Z direction'
    )

    planning_horizon_arg = DeclareLaunchArgument(
        'planning_horizon',
        default_value='6.0',
        description='Planning horizon'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/vins_estimator/imu_propagate',
        description='Odometry topic'
    )
    
    # Get argument values
    obj_num = LaunchConfiguration('obj_num')
    drone_id = LaunchConfiguration('drone_id')
    resolution = LaunchConfiguration('resolution')
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    planning_horizon = LaunchConfiguration('planning_horizon')
    odom_topic = LaunchConfiguration('odom_topic')

    # 可选：仅影响显示，将Z轴翻转（默认关闭，适配倒装相机时可设为true）
    flip_z_arg = DeclareLaunchArgument(
        'flip_z',
        default_value='false',
        description='Flip Z axis only for visualization (does not affect algorithms)'
    )

    # Include advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ego_planner'),
                'launch',
                'advanced_param_exp.launch.py'
            )
        ),
        launch_arguments={
            'obj_num_set': obj_num,
            'drone_id': drone_id,
            'resolution_': resolution,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'planning_horizon_': planning_horizon,
            'odometry_topic': odom_topic,
            # camera pose: transform of camera frame in the world frame
            # depth topic: depth image, 640x480 by default
            # don't set cloud_topic if you already set these ones!
            'camera_pose_topic': 'nouse1',
            'depth_topic': '/camera/camera/depth/image_rect_raw',  # 直接使用实际topic名称
            # topic of point cloud measurement, such as from LIDAR
            # don't set camera pose and depth, if you already set this one!
            'cloud_topic': 'nouse2',
            # intrinsic params of the depth camera
            'cx': '323.3316345214844',
            'cy': '234.95498657226562',
            'fx': '384.39654541015625',
            'fy': '384.39654541015625',
            # maximum velocity and acceleration the drone will reach
            'max_vel': '0.5',
            'max_acc': '6.0',
            # always set to 1.5 times greater than sensing horizon
            'use_distinctive_trajs': 'false',
            # 1: use 2D Nav Goal to select goal
            # 2: use global waypoints below
            'flight_type': '1',
            # global waypoints
            # It generates a piecewise min-snap traj passing all waypoints
            'point_num': '1',
            'point0_x': '15.0',
            'point0_y': '0.0',
            'point0_z': '1.0',
            'point1_x': '0.0',
            'point1_y': '0.0',
            'point1_z': '1.0',
            'point2_x': '15.0',
            'point2_y': '0.0',
            'point2_z': '1.0',
            'point3_x': '0.0',
            'point3_y': '0.0',
            'point3_z': '1.0',
            'point4_x': '15.0',
            'point4_y': '0.0',
            'point4_z': '1.0'
        }.items()
        # 移除remappings，因为我们直接使用实际topic名称
    )

    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    # 无人机模型可视化（在 /drone_<id>_vis/robot 发布 Marker）
    # 说明：默认 RViz 配置使用该话题显示模型，而非 URDF RobotModel
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
        parameters=[
            {
                'mesh_resource': 'package://odom_visualization/meshes/meshes/hummingbird.mesh',
                'color/r': 0.0,
                'color/g': 0.0,
                'color/b': 1.0,
                'color/a': 1.0,
                'robot_scale': 1.0,
                'frame_id': 'world',
                'drone_id': drone_id,
                # 仅影响显示，将Z轴翻转（默认关闭，实机D435倒装时可设为true）
                'flip_z': LaunchConfiguration('flip_z'),
            }
        ],
        remappings=[
            ('odom', odom_topic),
            ('robot', ['drone_', drone_id, '_vis/robot']),
        ]
    )

    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(obj_num_arg)
    ld.add_action(drone_id_arg)
    ld.add_action(resolution_arg)
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(planning_horizon_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(flip_z_arg)
    
    # Add included launch files
    ld.add_action(advanced_param_include)
    
    # Add nodes
    ld.add_action(traj_server_node)
    ld.add_action(odom_visualization_node)
    
    return ld