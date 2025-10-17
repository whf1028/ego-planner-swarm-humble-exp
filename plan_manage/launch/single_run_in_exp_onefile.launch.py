#!/usr/bin/env python3

"""
ROS2版本的EGO-Planner实验启动脚本
基于ROS1的single_run_in_exp.launch改造
用于真实实验环境，接收VINS数据和PX4数据
Author: AI Assistant
Date: 2025-01-24
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # 声明启动参数
    obj_num_arg = DeclareLaunchArgument(
        'obj_num',
        default_value='10',
        description='Number of moving objects'
    )
    
    drone_id_arg = DeclareLaunchArgument(
        'drone_id', 
        default_value='0',
        description='Drone ID'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.25',  # 内存优化：减小地图大小，因为地图减小，分辨率可以适当增大，基于原先的0.4分辨来说：0.25导致资源消耗增加了一些，0.3的资源消耗减少了
        description='高精度分辨率，用于真机实验'
    )

    map_size_x_arg = DeclareLaunchArgument(
        'map_size_x',
        default_value='4.0',  # 内存优化：减小地图大小，减少内存使用，6->4
        description='Map size in X direction'
    )

    map_size_y_arg = DeclareLaunchArgument(
        'map_size_y',
        default_value='4.0',  # 内存优化：减小地图大小，减少内存使用，6->4
        description='Map size in Y direction'
    )

    map_size_z_arg = DeclareLaunchArgument(
        'map_size_z',
        default_value='1.5',  # 内存优化：减小高度，减少内存使用，2->1.5
        description='Map size in Z direction'
    )

    # A*节点池动态配置参数
    astar_pool_x_arg = DeclareLaunchArgument(
        'astar_pool_x',
        default_value='50',  # 室内默认配置
        description='A* node pool size in X direction'
    )

    astar_pool_y_arg = DeclareLaunchArgument(
        'astar_pool_y',
        default_value='50',  # 室内默认配置
        description='A* node pool size in Y direction'
    )

    astar_pool_z_arg = DeclareLaunchArgument(
        'astar_pool_z',
        default_value='20',  # 室内默认配置
        description='A* node pool size in Z direction'
    )
    
    # VINS里程计话题 - 修正为VINS实际发布的话题
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/vins_estimator/odometry',  # VINS实际发布的里程计话题
        description='Odometry topic from VINS'
    )

    # 深度相机话题 - 使用RealSense D435i
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/depth/image_rect_raw',  # RealSense实际发布的深度图话题（修正）
        description='Depth camera topic'
    )
    
    # 相机内参 - RealSense D435i参数
    cx_arg = DeclareLaunchArgument('cx', default_value='320.0')
    cy_arg = DeclareLaunchArgument('cy', default_value='240.0') 
    fx_arg = DeclareLaunchArgument('fx', default_value='387.229248046875')
    fy_arg = DeclareLaunchArgument('fy', default_value='387.229248046875')
    
    # 飞行参数
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value='2.0')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value='3.0')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value='3.5') # 规划视野，需要和地图大小相匹配，7.5->3.5
    
    # 飞行类型：1=手动目标点，2=预设航点
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value='1')
    
    # 航点参数 - 修复：确保所有waypoint参数都有默认值
    point_num_arg = DeclareLaunchArgument('point_num', default_value='1')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value='3.0')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value='0.0')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value='1.0')

    # 额外的waypoint参数（防止访问越界）
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value='0.0')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value='0.0')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value='0.8')
    
    # 获取参数值
    drone_id = LaunchConfiguration('drone_id')
    odom_topic = LaunchConfiguration('odom_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    
    # 可选：仅影响显示，将Z轴翻转（默认关闭，适配倒装相机时可设为true）
    flip_z_arg = DeclareLaunchArgument(
        'flip_z',
        default_value='false',
        description='Flip Z axis only for visualization (does not affect algorithms)'
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '\n=== EGO-Planner 真机实验模式 ===\n',
            'Drone ID: ', drone_id, '\n',
            'VINS Odometry: ', odom_topic, '\n',
            'Depth Camera: ', depth_topic, '\n',
            'Flight Type: ', LaunchConfiguration('flight_type'), '\n',
            '话题映射说明:\n',
            '  VINS -> EGO-Planner: /vins_estimator/odometry -> odom_world\n',
            '  RealSense -> EGO-Planner: /camera/camera/depth/image_rect_raw -> grid_map/depth\n',
            '  EGO-Planner -> PX4Ctrl: position_cmd -> /cmd\n',
            '  PX4Ctrl -> PX4: /fmu/in/vehicle_attitude_setpoint\n',
            '使用真实传感器数据进行路径规划\n',
            '================================\n'
        ]
    )
    
    # EGO-Planner主节点
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        parameters=[{
            # FSM参数
            'fsm/flight_type': LaunchConfiguration('flight_type'),
            'fsm/thresh_replan_time': 1.0,
            'fsm/thresh_no_replan_meter': 1.0,
            'fsm/planning_horizon': LaunchConfiguration('planning_horizon'),
            'fsm/planning_horizen_time': 3.0,
            'fsm/emergency_time': 1.0,
            'fsm/realworld_experiment': True,  # 真实实验模式
            'fsm/fail_safe': True,
            
            # 航点参数
            'fsm/waypoint_num': LaunchConfiguration('point_num'),
            'fsm/waypoint0_x': LaunchConfiguration('point0_x'),
            'fsm/waypoint0_y': LaunchConfiguration('point0_y'),
            'fsm/waypoint0_z': LaunchConfiguration('point0_z'),
            'fsm/waypoint1_x': LaunchConfiguration('point1_x'),
            'fsm/waypoint1_y': LaunchConfiguration('point1_y'),
            'fsm/waypoint1_z': LaunchConfiguration('point1_z'),
            
            # 地图参数 - 内存优化配置
            'grid_map/resolution': LaunchConfiguration('resolution'), 
            'grid_map/map_size_x': LaunchConfiguration('map_size_x'),
            'grid_map/map_size_y': LaunchConfiguration('map_size_y'),
            'grid_map/map_size_z': LaunchConfiguration('map_size_z'),
            'grid_map/local_update_range_x': 3.0,  # 内存优化：减小局部更新范围
            'grid_map/local_update_range_y': 3.0,  # 内存优化：减小局部更新范围
            'grid_map/local_update_range_z': 2.0,  # 内存优化：减小局部更新范围
            'grid_map/obstacles_inflation': 0.1,   # 调低膨胀，降低误判碰撞概率
            'grid_map/local_map_margin': 5,        # 内存优化：减小局部地图边界
            'grid_map/ground_height': -0.01,
            
            # 相机参数
            'grid_map/cx': LaunchConfiguration('cx'),
            'grid_map/cy': LaunchConfiguration('cy'),
            'grid_map/fx': LaunchConfiguration('fx'),
            'grid_map/fy': LaunchConfiguration('fy'),
            
            # 深度滤波参数 - 内存优化
            'grid_map/use_depth_filter': True,
            'grid_map/depth_filter_tolerance': 0.2,
            'grid_map/depth_filter_maxdist': 3.0,  # 内存优化：减小最大距离
            'grid_map/depth_filter_mindist': 0.3,
            'grid_map/depth_filter_margin': 1,     # 内存优化：减小边界
            'grid_map/k_depth_scaling_factor': 1000.0,
            'grid_map/skip_pixel': 4,              # 内存优化：增加跳过像素，减少处理量
            
            # 局部融合参数 - 优化真机环境
            'grid_map/p_hit': 0.65,
            'grid_map/p_miss': 0.35,
            'grid_map/p_min': 0.12,
            'grid_map/p_max': 0.90,
            'grid_map/p_occ': 0.80,
            'grid_map/min_ray_length': 0.3,
            'grid_map/max_ray_length': 5.0,
            'grid_map/visualization_truncate_height': 1.8,
            'grid_map/show_occ_time': False,
            'grid_map/pose_type': 2,
            'grid_map/frame_id': 'world',
            'grid_map/colorize': True,
            
            # 规划器管理参数
            'manager/max_vel': LaunchConfiguration('max_vel'),
            'manager/max_acc': LaunchConfiguration('max_acc'),
            'manager/max_jerk': 4.0,
            'manager/control_points_distance': 0.4,
            'manager/feasibility_tolerance': 0.05,
            'manager/planning_horizon': LaunchConfiguration('planning_horizon'),
            'manager/use_distinctive_trajs': False,
            'manager/drone_id': LaunchConfiguration('drone_id'),

            # A*节点池动态配置
            'manager/astar_pool_x': LaunchConfiguration('astar_pool_x'),
            'manager/astar_pool_y': LaunchConfiguration('astar_pool_y'),
            'manager/astar_pool_z': LaunchConfiguration('astar_pool_z'),
            
            # 轨迹优化参数
            'optimization/lambda_smooth': 1.0,
            'optimization/lambda_collision': 0.5,
            'optimization/lambda_feasibility': 0.1,
            'optimization/lambda_fitness': 1.5,   # 提高可行性拟合权重，利于找到可行轨迹
            'optimization/dist0': 0.5,
            'optimization/swarm_clearance': 0.5,
            'optimization/max_vel': LaunchConfiguration('max_vel'),
            'optimization/max_acc': LaunchConfiguration('max_acc'),
            'optimization/order': 3,  # 添加缺失的order参数
            
            # B样条参数 - 添加缺失的参数
            'bspline/limit_vel': LaunchConfiguration('max_vel'),
            'bspline/limit_acc': LaunchConfiguration('max_acc'),
            'bspline/limit_ratio': 1.1,
            
            # 物体预测参数
            'prediction/obj_num': LaunchConfiguration('obj_num'),
            'prediction/lambda': 1.0,
            'prediction/predict_rate': 1.0,
        }],
        remappings=[
            # 重映射话题到实际的传感器数据
            ('odom_world', odom_topic),  # VINS里程计: /vins_estimator/odometry
            ('grid_map/odom', odom_topic),  # 地图用里程计: /vins_estimator/odometry
            ('grid_map/depth', depth_topic),  # 深度相机: /camera/camera/depth/image_raw
            ('grid_map/cloud', 'nouse2'),  # 不使用点云
            ('grid_map/pose', 'nouse1'),  # 不使用相机位姿
            # 规划输出话题
            ('planning/bspline', ['/drone_', drone_id, '_planning/bspline']),
            ('planning/data_display', ['/drone_', drone_id, '_planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
             # 添加可视化话题的重映射
            ('goal_point', ['/drone_', drone_id, '_ego_planner_node/goal_point']),
            ('global_list', ['/drone_', drone_id, '_ego_planner_node/global_list']),
            ('init_list', ['/drone_', drone_id, '_ego_planner_node/init_list']),
            ('optimal_list', ['/drone_', drone_id, '_ego_planner_node/optimal_list']),
            ('a_star_list', ['/drone_', drone_id, '_ego_planner_node/a_star_list']),
            ('grid_map/occupancy', ['/drone_', drone_id, '_ego_planner_node/grid_map/occupancy']),
            ('grid_map/occupancy_inflate', ['/drone_', drone_id, '_ego_planner_node/grid_map/occupancy_inflate']),
        ]
    )
    
    # 轨迹服务器节点
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server', 
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        parameters=[{
            'traj_server/time_forward': 1.0,
        }],
        remappings=[
            ('planning/bspline', ['/drone_', drone_id, '_planning/bspline']),
            ('position_cmd', '/cmd'),  # 输出给px4ctrl: /cmd (px4ctrl订阅的话题)
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
    
    return LaunchDescription([
        # 参数声明
        obj_num_arg,
        drone_id_arg,
        resolution_arg,
        map_size_x_arg,
        map_size_y_arg,
        map_size_z_arg,
        odom_topic_arg,
        depth_topic_arg,
        cx_arg, 
        cy_arg, 
        fx_arg, 
        fy_arg,
        max_vel_arg, max_acc_arg,
        planning_horizon_arg,
        flight_type_arg,
        point_num_arg,
        point0_x_arg, 
        point0_y_arg, 
        point0_z_arg,
        point1_x_arg, 
        point1_y_arg, 
        point1_z_arg,
        astar_pool_x_arg, 
        astar_pool_y_arg, 
        astar_pool_z_arg,
        flip_z_arg,
        
        # 启动信息
        launch_info,
        
        # 节点
        ego_planner_node,
        traj_server_node,
        odom_visualization_node,
    ])
