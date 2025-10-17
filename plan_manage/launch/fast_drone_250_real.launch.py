#!/usr/bin/env python3

"""
Fast-Drone-250 ego planner 真机启动脚本
专门针对RealSense D435 + VINS-Fusion真机环境优化
Author: AI Assistant
Date: 2025-01-25
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 日志信息
    log_info = LogInfo(
        msg=TextSubstitution(text="启动Fast-Drone-250真机环境 - RealSense D435 + VINS-Fusion")
    )
    
    # ego planner节点
    ego_planner_node = Node(
        package='plan_manage',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                # 话题配置
                'odom_topic': '/vins_estimator/odometry',  # VINS里程计
                'depth_topic': '/camera/camera/depth/image_raw',  # RealSense深度图
                
                # 相机内参 - RealSense D435参数
                'camera/cx': 320.0,
                'camera/cy': 240.0,
                'camera/fx': 387.229248046875,
                'camera/fy': 387.229248046875,
                
                # 其他参数 - 调整为更保守的值
                'max_vel': 0.8,           # 降低最大速度 (原值: 1.0)
                'max_acc': 1.2,           # 降低最大加速度 (原值: 2.0)
                'max_jerk': 2.0,          # 添加最大加加速度限制
                'planning_horizon': 7.5,
                'use_distinctive_trajs': True,
                
                # 目标配置 - 提高精度要求
                'goal_tolerance': 0.3,    # 降低目标容差 (原值: 0.5)
                'waypoint_num': 10,
                'waypoint_distance': 0.4, # 减少航点距离 (原值: 0.5)
                
                # 真机环境参数调整
                'real_world_mode': True,
                'obstacle_filter_threshold': 0.6,  # 增加障碍物过滤阈值
                'dynamic_obstacle_avoidance': True,
                
                # 新增控制参数
                'position_control_gain_p': 0.8,    # 位置控制比例增益
                'position_control_gain_i': 0.05,   # 位置控制积分增益
                'position_control_gain_d': 0.02,   # 位置控制微分增益
            }
        ]
    )
    
    return LaunchDescription([
        log_info,
        ego_planner_node
    ])