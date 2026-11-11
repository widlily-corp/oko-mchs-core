# Large Rescue Drone (STL by Studio Mechanik)

Модель крупного спасательного квадрокоптера для Gazebo 11.

## Файлы

```text
largedrone/
├── model.config
├── model.sdf
├── preview.png
└── meshes/
    └── largedrone.stl   # исходный STL, 988 062 треугольника
```

## Параметры

| Параметр | Значение |
|---|---|
| Исходный меш | `largedrone.by.studiomechanik.stl` |
| Треугольников | 988 062 |
| Габариты STL | 100 × 80.19 × 35.37 единиц |
| Масштаб в Gazebo | `0.01` (1 единица STL = 1 см) |
| Габариты в Gazebo | ≈ 1.00 × 0.80 × 0.354 м |
| Ориентация | нос (подвес камеры) развёрнут по **+X** |
| Масса | 4.8 кг |
| Камера | 1280×720, HFOV 80°, наклон вниз 45° |
| Топики | `/mchs/camera/image_raw`, `/mchs/camera/camera_info` |
| Движение | `libgazebo_ros_planar_move.so`: `/mchs/cmd_vel` → `/mchs/odom` |

## Как подключено

В `mchs_emergency_arena.world`:

```xml
<include>
  <uri>model://largedrone</uri>
</include>
```

`model://largedrone` находится через `GAZEBO_MODEL_PATH`, который
`simulation.launch.py` добавляет автоматически.

## Ориентация меша

Исходный STL смотрит носом в **−Y** (подвес камеры — в отрицательной Y).
В `model.sdf` visual повёрнут на `yaw = +90°`, поэтому нос смотрит в **+X** —
в сторону полёта в ROS/Gazebo. Камера закреплена в точке `x = 0.42 м`,
`z = 0.12 м` и наклонена вниз на 45°.

## Ограничение симуляции

`libgazebo_ros_planar_move.so` двигает модель только в плоскости X–Y и
игнорирует `linear.z`. Поэтому команда взлёта автопилота не поднимает
модель: дрон остаётся на высоте ~0.05 м. Для полноценного полёта нужен
плагин высоты или замена `planar_move` на модель с `linear.z`.
