#!/usr/bin/env python3
"""
Главный launch-файл симуляции БПЛА 'Око МЧС'.
Запускает Gazebo 11:
  - gzserver (физический мир, арена ЧС, сенсоры, плагины ROS 2)
  - gzclient (графическое 3D-окно симулятора для визуализации полета БПЛА)
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

    # Настройки WSLg для надежного вывода 3D-графики на экран Windows
    display_env = os.environ.get('DISPLAY', ':0')
    wayland_env = os.environ.get('WAYLAND_DISPLAY', 'wayland-0')
    runtime_dir = os.environ.get('XDG_RUNTIME_DIR', '/mnt/wslg/runtime-dir')

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=new_model_path),
        SetEnvironmentVariable(name='DISPLAY', value=display_env),
        SetEnvironmentVariable(name='WAYLAND_DISPLAY', value=wayland_env),
        SetEnvironmentVariable(name='XDG_RUNTIME_DIR', value=runtime_dir),
        SetEnvironmentVariable(name='LIBGL_ALWAYS_INDIRECT', value='0'),
        
        # 1. Сервер физики и плагинов ROS 2
        ExecuteProcess(
            cmd=[
                'gzserver',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),
        
        # 2. Графический клиент Gazebo (3D визуал полета дрона над ареной)
        ExecuteProcess(
            cmd=['gzclient', '--verbose'],
            output='screen'
        )
    ])

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
