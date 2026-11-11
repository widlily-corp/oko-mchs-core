#!/usr/bin/env python3
"""
Главный launch-файл симуляции БПЛА 'Око МЧС'.
Запускает Gazebo 11 с кастомным миром арены ЧС (mchs_emergency_arena.world)
и регистрирует путь к кастомной 3D-модели дрона mchs_drone.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_path = os.path.join(pkg_dir, 'worlds', 'mchs_emergency_arena.world')
    models_path = os.path.join(pkg_dir, 'models')

    # Добавляем каталог моделей проекта в GAZEBO_MODEL_PATH
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if current_model_path:
        new_model_path = f"{models_path}:{current_model_path}"
    else:
        new_model_path = f"{models_path}:/usr/share/gazebo-11/models"

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=new_model_path
        ),
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])
