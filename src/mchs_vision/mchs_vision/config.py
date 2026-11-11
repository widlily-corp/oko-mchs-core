"""Конфигурация бортового модуля компьютерного зрения «Око МЧС».

Модуль не зависит от ROS: те же настройки используются ROS-нодой,
офлайн-демонстратором и юнит-тестами. Все пороги вынесены в dataclass'ы,
чтобы их можно было переопределить параметрами ноды или аргументами CLI.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

# Цвет в формате OpenCV (BGR), а не RGB.
BGR = Tuple[int, int, int]
HSVRange = Tuple[Tuple[int, int, int], Tuple[int, int, int]]


@dataclass(frozen=True)
class HazardClassSpec:
    """Описание одного класса ЧС.

    Атрибуты:
        name: машиночитаемый код класса (уходит в JSON и ROS-топик).
        label_ru: человекочитаемое название для отчётов и презентации.
        color_bgr: цвет рамки/HUD в палитре OpenCV (BGR).
        hsv_ranges: список диапазонов HSV. Пиксель попадает в маску класса,
            если он попал хотя бы в один диапазон.
        min_area_px: минимальная площадь пятна класса в пикселях (число
            пикселей маски, а не площадь внешнего контура). Отсекает шум.
        open_kernel: размер ядра MORPH_OPEN (убирает одиночные пиксели).
        close_kernel: размер ядра MORPH_CLOSE (склеивает разрывы пятна).
        max_texture_std: максимальное СКО лапласиана внутри контура.
            Для гладких пятен (нефть, вода, дым) помогает отсечь текстуру
            травы/листвы, которая лишь кажется тёмной или серой.
        max_fill_ratio: максимальная доля пикселей класса в bbox. Плоские
            прямоугольные крыши и разметка дают почти 1.0; пламя и дым —
            неровные пятна заметно меньше.
        min_solidity_ratio: доля пикселей класса внутри внешнего контура.
            Кольца и полые объекты (например, оранжевая разметка вертодрома)
            дают < 0.5 и отбрасываются, хотя внешний контур выглядит
            сплошным диском.
        min_hull_ratio: площадь контура / площадь выпуклой оболочки. Тонкие
            серповидные и кольцевые структуры имеют низкое значение.
        max_rectangularity: площадь контура / площадь минимального
            повёрнутого прямоугольника. Почти 1.0 — это искусственная
            прямоугольная крыша/плита, а не очаг ЧС.
        min_hue_std: минимальное СКО оттенка внутри маски. Однотонная
            оранжевая поверхность (разметка, крыша, конус) имеет СКО < 1,
            а пламя всегда содержит несколько оттенков.
        min_mean_saturation: минимальная средняя насыщенность. Нейтральные
            серые тени зданий (S < 20) не являются нефтяным пятном, у нефти
            обычно есть цветной оттенок или радужная плёнка.
        max_mean_value: максимальная средняя яркость. Бурые лужи и мокрый
            грунт светлее тёмного нефтяного пятна и отсекаются по V.
        max_border_touches: сколько краёв кадра может касаться контур.
            Для наземных классов 0: срезанный объект даёт неверную площадь.
            Для воды допускается 2, потому что разлив может уходить за кадр,
            но кадр, полностью залитый синим (небо/фон), — не затопление.
        min_area_m2: минимальная площадь очага в метрах для публикации.
            Мелкие блики и фрагменты построек не должны создавать инцидент.
        description: пояснение для документации.
    """

    name: str
    label_ru: str
    color_bgr: BGR
    hsv_ranges: Tuple[HSVRange, ...]
    min_area_px: int
    open_kernel: int = 3
    close_kernel: int = 9
    max_texture_std: float = 999.0
    max_fill_ratio: float = 0.98
    min_solidity_ratio: float = 0.55
    min_hull_ratio: float = 0.35
    max_rectangularity: float = 0.97
    min_hue_std: float = 0.0
    min_mean_saturation: float = 0.0
    max_mean_value: float = 255.0
    max_border_touches: int = 0
    min_area_m2: float = 0.0
    description: str = ""


# Порядок классов в кортеже = приоритет при подавлении перекрытий.
# Огонь важнее дыма: дым рядом с пламенем не должен «перебивать» очаг.
FIRE = HazardClassSpec(
    name="FIRE",
    label_ru="Пожар",
    color_bgr=(0, 80, 255),  # оранжево-красный
    hsv_ranges=(((5, 120, 180), (25, 255, 255)),),
    min_area_px=250,
    open_kernel=3,
    close_kernel=9,
    max_texture_std=999.0,
    max_fill_ratio=0.92,
    min_solidity_ratio=0.55,
    min_hull_ratio=0.40,
    max_rectangularity=0.93,
    min_hue_std=1.2,
    min_area_m2=2.0,
    description="Яркие жёлто-оранжевые тона пламени с высокой насыщенностью.",
)

SMOKE = HazardClassSpec(
    name="SMOKE",
    label_ru="Задымление",
    color_bgr=(200, 200, 200),  # светло-серый
    hsv_ranges=(((0, 0, 160), (180, 45, 230)),),
    min_area_px=700,
    open_kernel=7,
    close_kernel=15,
    max_texture_std=26.0,
    max_fill_ratio=0.90,
    min_solidity_ratio=0.55,
    min_hull_ratio=0.40,
    max_rectangularity=0.90,
    min_area_m2=25.0,
    description="Светло-серые рассеянные тона с низкой насыщенностью.",
)

OIL_SPILL = HazardClassSpec(
    name="OIL_SPILL",
    label_ru="Разлив нефтепродуктов",
    color_bgr=(180, 60, 200),  # фиолетовый, хорошо виден на зелёном
    hsv_ranges=(((0, 0, 15), (180, 75, 60)),),
    min_area_px=400,
    open_kernel=5,
    close_kernel=11,
    max_texture_std=17.0,
    max_fill_ratio=0.90,
    min_solidity_ratio=0.55,
    min_hull_ratio=0.35,
    max_rectangularity=0.92,
    min_mean_saturation=25.0,
    max_mean_value=48.0,
    min_area_m2=5.0,
    description="Тёмные малосатурационные пятна на почве/траве.",
)

FLOOD = HazardClassSpec(
    name="FLOOD",
    label_ru="Затопление",
    color_bgr=(255, 170, 0),  # сине-голубой
    hsv_ranges=(((95, 80, 190), (125, 255, 255)),),
    min_area_px=600,
    open_kernel=7,
    close_kernel=15,
    max_texture_std=22.0,
    max_fill_ratio=0.88,
    min_solidity_ratio=0.55,
    min_hull_ratio=0.35,
    max_rectangularity=0.86,
    max_border_touches=2,
    min_area_m2=30.0,
    description="Сине-голубая водная поверхность с высокой насыщенностью.",
)

DEFAULT_CLASSES: Tuple[HazardClassSpec, ...] = (FIRE, OIL_SPILL, FLOOD, SMOKE)
DEFAULT_ENABLED: Tuple[str, ...] = ("FIRE", "SMOKE", "OIL_SPILL", "FLOOD")


@dataclass
class DetectorConfig:
    """Параметры детектора, не привязанные к конкретному классу."""

    classes: Tuple[HazardClassSpec, ...] = DEFAULT_CLASSES
    enabled: Tuple[str, ...] = DEFAULT_ENABLED

    # Геометрические фильтры контуров.
    max_area_ratio: float = 0.50        # >50% кадра — это фон/небо, не очаг
    min_fill_ratio: float = 0.18        # пиксели класса / площадь bbox
    max_aspect_ratio: float = 10.0      # отсекаем длинные тонкие линии (дороги)
    border_touch_limit: int = 0         # объект, срезанный краем кадра, не считаем
    merge_gap_px: int = 12              # склейка близких пятен одного класса
    max_detections_per_class: int = 3

    # Пороги min_area_px заданы для кадра 640×480. На 1280×720 площадь
    # кадра в 3 раза больше, поэтому порог умножается на 3 — иначе
    # чувствительность выросла бы и в кадр полезли бы мелкие блики.
    reference_frame_pixels: int = 640 * 480
    scale_min_area_with_resolution: bool = True

    # Текстурный фильтр.
    texture_blur_ksize: int = 5

    # Трекинг и подтверждение детекций.
    track_min_hits: int = 2             # сколько кадров подряд нужно для события
    track_max_missed: int = 5           # сколько кадров трек живёт без детекции
    track_match_iou: float = 0.25
    track_match_distance_px: float = 45.0
    smoothing_alpha: float = 0.45       # EMA для координат/площади

    # Уровни опасности по площади (из ТЗ хакатона).
    threat_warning_m2: float = 15.0
    threat_critical_m2: float = 50.0

    min_confidence: float = 0.30


@dataclass(frozen=True)
class CameraConfig:
    """Модель камеры БПЛА.

    pitch_down_deg — угол наклона оптической оси вниз:
        0°   — камера смотрит в горизонт (курсовая);
        45°  — текущая камера в largedrone/model.sdf;
        90°  — строго надирная камера.

    Пиксели считаются квадратными, поэтому fy = fx.
    """

    width: int = 1280
    height: int = 720
    hfov_deg: float = 80.0
    pitch_down_deg: float = 45.0

    # Смещение камеры относительно центра масс дрона (метры, СК дрона FLU:
    # X — вперёд, Y — влево, Z — вверх). Соответствует
    # src/mchs_simulation/models/largedrone/model.sdf: pose 0.42 0 0.12.
    mount_forward_m: float = 0.42
    mount_right_m: float = 0.0
    mount_up_m: float = 0.12

    min_range_m: float = 0.3
    max_range_m: float = 300.0

    @property
    def fx(self) -> float:
        return self.width / (2.0 * math.tan(math.radians(self.hfov_deg) / 2.0))

    @property
    def fy(self) -> float:
        # Квадратные пиксели: вертикальный FOV выводится из соотношения сторон.
        return self.fx

    @property
    def cx(self) -> float:
        return (self.width - 1) / 2.0

    @property
    def cy(self) -> float:
        return (self.height - 1) / 2.0

    @property
    def vfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan((self.height / 2.0) / self.fy))

    @property
    def gsd_at_nadir_m(self) -> float:
        """Разрешение на местности для надирной съёмки на единичной высоте."""
        return (2.0 * math.tan(math.radians(self.hfov_deg) / 2.0)) / self.width


@dataclass(frozen=True)
class GeoOrigin:
    """Точка привязки локальной системы координат к WGS-84."""

    lat: float = 53.4129
    lon: float = 58.9814


DEFAULT_CAMERA = CameraConfig()
DEFAULT_ORIGIN = GeoOrigin()


def threat_level(area_m2: float, warning_m2: float = 15.0,
                 critical_m2: float = 50.0) -> str:
    """Классификация уровня опасности по площади очага (ТЗ хакатона).

    < 15 м² — ADVISORY, 15..50 м² — WARNING, > 50 м² — CRITICAL.
    Границы включены в нижний уровень, чтобы ровно 15 м² не «прыгало».
    """
    if area_m2 < warning_m2:
        return "ADVISORY"
    if area_m2 <= critical_m2:
        return "WARNING"
    return "CRITICAL"


def class_spec(name: str, classes: Tuple[HazardClassSpec, ...] = DEFAULT_CLASSES
               ) -> HazardClassSpec:
    """Найти описание класса по имени (с понятной ошибкой)."""
    for spec in classes:
        if spec.name == name:
            return spec
    raise KeyError(f"Неизвестный класс ЧС: {name}")


def classes_by_name(classes: Tuple[HazardClassSpec, ...] = DEFAULT_CLASSES
                    ) -> Dict[str, HazardClassSpec]:
    return {spec.name: spec for spec in classes}
