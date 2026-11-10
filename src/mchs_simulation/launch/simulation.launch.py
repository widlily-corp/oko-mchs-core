#!/usr/bin/env python3
"""
Главный launch-файл симуляции БПЛА 'Око МЧС'.
Запускает Gazebo 11 с лесным миром и очагом возгорания.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_path = os.path.join(pkg_dir, 'worlds', 'mchs_fire_zone.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        )
    ])
