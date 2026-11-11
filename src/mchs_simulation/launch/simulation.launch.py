#!/usr/bin/env python3
"""
Главный launch-файл симуляции БПЛА 'Око МЧС'.
Запускает Gazebo 11 с кастомным миром арены ЧС (mchs_emergency_arena.world)
и регистрирует путь к кастомной 3D-модели дрона mchs_drone.

Поддерживает запуск:
  1) ros2 launch src/mchs_simulation/launch/simulation.launch.py
  2) python3 src/mchs_simulation/launch/simulation.launch.py
"""
import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_path = os.path.join(pkg_dir, 'worlds', 'mchs_emergency_arena.world')
    models_path = os.path.join(pkg_dir, 'models')

    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    standard_paths = "/usr/share/gazebo-11/models:/usr/share/gazebo/models"
    if current_model_path:
        new_model_path = f"{models_path}:{current_model_path}:{standard_paths}"
    else:
        new_model_path = f"{models_path}:{standard_paths}"

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

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
