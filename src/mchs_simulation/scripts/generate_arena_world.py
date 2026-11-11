#!/usr/bin/env python3
"""
Генератор реалистичного полигона ЧС «Око МЧС» для Gazebo 11.

Создаёт карту ~200×200 м:
  • аэродром/вертодром базы МЧС (вертолетная площадка, ангар, вышка, контейнеры);
  • дорожная сеть с разметкой и мостом через реку;
  • река, пойма, зона затопления;
  • лесной массив с очагом пожара;
  • нефтебаза с резервуарами, обваловкой и трубопроводами;
  • полигон ТБО / несанкционированная свалка;
  • посёлок, склады, поля, холмы, кустарник.

Координаты 4 очагов ЧС берутся из config/hazards.json (fallback — демо-значения).
Гравитация мира отключена: кинематический дрон держит высоту 22 м.

Запуск:  python3 generate_arena_world.py
Результат: worlds/mchs_emergency_arena.world
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD_PATH = os.path.join(HERE, "..", "worlds", "mchs_emergency_arena.world")
CONFIG_PATH = os.path.join(HERE, "..", "..", "..", "config", "hazards.json")

# Координаты очагов по умолчанию (совпадают с config/hazards.json).
DEFAULT_HAZARD_POSES = {
    "FIRE": (60.0, 28.0, 0.2),
    "OIL": (-58.0, 30.0, 0.02),
    "FLOOD": (45.0, -56.0, 0.02),
    "WASTE": (68.0, -72.0, 0.4),
    "HUMAN": (15.0, -22.0, 0.0),
}


def load_hazard_poses():
    """Читает координаты очагов из config/hazards.json с fallback-значениями."""
    poses = dict(DEFAULT_HAZARD_POSES)
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return poses
    if not isinstance(data, dict):
        return poses
    for key in poses:
        item = data.get(key)
        if not isinstance(item, dict):
            continue
        try:
            x = float(item["x"])
            y = float(item["y"])
            z = float(item.get("z", poses[key][2]))
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            poses[key] = (x, y, z)
    return poses

# --------------------------------------------------------------------- утилиты

def c(color, alpha=1.0):
    return f"{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} {alpha:.2f}"


def fmt_pose(pose):
    return " ".join(f"{v:.4f}" if isinstance(v, float) else str(v) for v in pose)


def material(color, emissive=None, specular=(0.08, 0.08, 0.08), alpha=1.0):
    xml = (
        "        <material>\n"
        f"          <ambient>{c(color, alpha)}</ambient>\n"
        f"          <diffuse>{c(color, alpha)}</diffuse>\n"
        f"          <specular>{c(specular)}</specular>\n"
    )
    if emissive is not None:
        xml += f"          <emissive>{c(emissive)}</emissive>\n"
    xml += "        </material>\n"
    return xml


def box(sx, sy, sz):
    return f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"


def cylinder(radius, length):
    return f"<cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder>"


def sphere(radius):
    return f"<sphere><radius>{radius:.3f}</radius></sphere>"


def plane(size):
    return f"<plane><normal>0 0 1</normal><size>{size:.1f} {size:.1f}</size></plane>"


class Model:
    """Статическая декоративная модель: один link, много visual/collision."""

    def __init__(self, name, pose=(0, 0, 0, 0, 0, 0), static=True):
        self.name = name
        self.pose = pose
        self.static = static
        self.visuals = []
        self.collisions = []

    def visual(self, geom, pose=(0, 0, 0, 0, 0, 0), color=(0.5, 0.5, 0.5),
               emissive=None, specular=(0.08, 0.08, 0.08), alpha=1.0, name=None):
        vid = name or f"visual_{len(self.visuals)}"
        self.visuals.append(
            f"      <visual name=\"{vid}\">\n"
            f"        <pose>{fmt_pose(pose)}</pose>\n"
            f"        <geometry>{geom}</geometry>\n"
            f"{material(color, emissive, specular, alpha)}"
            f"      </visual>\n"
        )

    def collision(self, geom, pose=(0, 0, 0, 0, 0, 0)):
        cid = f"collision_{len(self.collisions)}"
        self.collisions.append(
            f"      <collision name=\"{cid}\">\n"
            f"        <pose>{fmt_pose(pose)}</pose>\n"
            f"        <geometry>{geom}</geometry>\n"
            f"      </collision>\n"
        )

    def xml(self):
        out = [f"    <model name=\"{self.name}\">\n"]
        if self.static:
            out.append("      <static>true</static>\n")
        out.append(f"      <pose>{fmt_pose(self.pose)}</pose>\n")
        out.append("      <link name=\"link\">\n")
        out.extend(self.visuals)
        out.extend(self.collisions)
        out.append("      </link>\n")
        out.append("    </model>\n")
        return "".join(out)


# ------------------------------------------------------------ элементы сцены

def add_tree(model, x, y, scale=1.0, crown=(0.10, 0.35, 0.10), trunk=(0.30, 0.18, 0.08)):
    """Дерево: ствол + 2-3 кроны (без коллизий для производительности)."""
    h = 5.0 * scale
    model.visual(cylinder(0.25 * scale, h), (x, y, h / 2.0), trunk,
                 name=f"trunk_{len(model.visuals)}")
    model.visual(sphere(2.4 * scale), (x, y, h), crown, name=f"crown_{len(model.visuals)}")
    model.visual(sphere(1.8 * scale), (x + 0.8 * scale, y - 0.6 * scale, h + 1.6 * scale),
                 tuple(min(1.0, v * 1.15) for v in crown), name=f"crown_{len(model.visuals)}")
    model.visual(sphere(1.7 * scale), (x - 0.9 * scale, y + 0.7 * scale, h + 1.2 * scale),
                 tuple(v * 0.9 for v in crown), name=f"crown_{len(model.visuals)}")


def add_house(model, x, y, w, d, h, rot, wall=(0.62, 0.58, 0.48), roof=(0.32, 0.12, 0.09)):
    model.visual(box(w, d, h), (x, y, h / 2.0, 0, 0, rot), wall,
                 name=f"house_{len(model.visuals)}")
    model.visual(box(w * 1.12, d * 1.12, h * 0.32), (x, y, h + h * 0.14, 0, 0, rot), roof,
                 name=f"roof_{len(model.visuals)}")
    model.visual(box(w * 0.18, d * 0.04, h * 0.45),
                 (x, y, h * 0.26, 0, 0, rot), (0.20, 0.22, 0.26),
                 name=f"door_{len(model.visuals)}")
    for k, ox in enumerate((-w * 0.28, w * 0.28)):
        model.visual(box(w * 0.14, d * 0.03, h * 0.18),
                     (x + ox, y, h * 0.62, 0, 0, rot), (0.55, 0.75, 0.85),
                     emissive=(0.25, 0.35, 0.45), name=f"window_{len(model.visuals)}")


def add_tank(model, x, y, radius=6.0, height=7.0, color=(0.72, 0.73, 0.70)):
    model.visual(cylinder(radius, height), (x, y, height / 2.0), color,
                 specular=(0.35, 0.35, 0.35), name=f"tank_{len(model.visuals)}")
    model.visual(sphere(radius), (x, y, height), tuple(v * 0.95 for v in color),
                 name=f"dome_{len(model.visuals)}")
    for ox, oy in ((-radius, 0), (radius, 0), (0, -radius), (0, radius)):
        model.visual(box(radius * 1.9, 0.35, 0.35),
                     (x + ox * 0.2, y + oy * 0.2, 0.6, 0, 0, math.atan2(oy, ox)),
                     (0.45, 0.45, 0.45), name=f"pipe_{len(model.visuals)}")


def add_berm(model, cx, cy, size=30.0, wall=1.1):
    """Прямоугольная обваловка (4 стены)."""
    half = size / 2.0
    model.visual(box(size, 0.6, wall), (cx, cy - half, wall / 2.0), (0.52, 0.47, 0.38))
    model.visual(box(size, 0.6, wall), (cx, cy + half, wall / 2.0), (0.52, 0.47, 0.38))
    model.visual(box(0.6, size, wall), (cx - half, cy, wall / 2.0), (0.52, 0.47, 0.38))
    model.visual(box(0.6, size, wall), (cx + half, cy, wall / 2.0), (0.52, 0.47, 0.38))


def add_fence(model, x1, y1, x2, y2, height=1.6, color=(0.55, 0.55, 0.52)):
    length = math.hypot(x2 - x1, y2 - y1)
    rot = math.atan2(y2 - y1, x2 - x1)
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    model.visual(box(length, 0.08, height), (cx, cy, height / 2.0, 0, 0, rot),
                 color, name=f"fence_{len(model.visuals)}")
    posts = max(2, int(length / 6.0))
    for i in range(posts + 1):
        t = i / max(1, posts)
        px = x1 + (x2 - x1) * t
        py = y1 + (y2 - y1) * t
        model.visual(box(0.16, 0.16, height * 1.1), (px, py, height * 0.55),
                     (0.35, 0.33, 0.30), name=f"post_{len(model.visuals)}")


def add_barrel(model, x, y, color, h=1.1, r=0.45):
    model.visual(cylinder(r, h), (x, y, h / 2.0), color,
                 specular=(0.3, 0.3, 0.3), name=f"barrel_{len(model.visuals)}")


def add_road(model, x, y, length, width, rot, color=(0.34, 0.34, 0.37)):
    model.visual(box(length, width, 0.04), (x, y, 0.02, 0, 0, rot), color,
                 specular=(0.05, 0.05, 0.05), name=f"road_{len(model.visuals)}")


def add_dashes(model, x, y, length, width, rot, count=12):
    for i in range(count):
        t = (i + 0.5) / count - 0.5
        dx = math.cos(rot) * t * length
        dy = math.sin(rot) * t * length
        model.visual(box(length / count * 0.45, 0.35, 0.03), (x + dx, y + dy, 0.045, 0, 0, rot),
                     (0.85, 0.85, 0.80), name=f"dash_{len(model.visuals)}")


# --------------------------------------------------------------------- сцена

def build_world():
    parts = []
    add = parts.append
    hazard_poses = load_hazard_poses()

    add('<?xml version="1.0" ?>\n')
    add('<sdf version="1.6">\n')
    add('  <world name="mchs_emergency_arena">\n')

    # Освещение и небо
    add('''    <include>
      <uri>model://sun</uri>
    </include>
    <scene>
      <ambient>0.55 0.58 0.62 1.0</ambient>
      <background>0.53 0.68 0.86 1.0</background>
      <shadows>true</shadows>
    </scene>
''')

    # Физика
    add('''    <physics type="ode">
      <!-- Высоту 22 м держит <gravity>false</gravity> в модели дрона;
           глобальную гравитацию арены не трогаем (Gazebo 11.10 не умеет
           применять <gravity>0 0 0</gravity> и пишет ошибку any_cast). -->
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <!-- ROS 2 World State Plugin: /gazebo/set_entity_state, /gazebo/get_entity_state -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>50.0</update_rate>
    </plugin>
''')

    # --- Земля 240x240 м ---------------------------------------------------
    ground = Model("terrain_ground")
    ground.collision(plane(240.0), (0, 0, 0))
    ground.visual(plane(240.0), (0, 0, 0), (0.20, 0.32, 0.14), specular=(0.02, 0.02, 0.02),
                  name="grass_plane")
    add(ground.xml())

    # --- Поля/луговые пятна (patchwork) -----------------------------------
    fields = Model("fields")
    field_specs = [
        (-15, 70, 60, 34, (0.42, 0.40, 0.18)),
        (-78, 85, 55, 28, (0.30, 0.34, 0.14)),
        (30, 82, 65, 26, (0.46, 0.43, 0.20)),
        (-88, -20, 24, 46, (0.26, 0.36, 0.15)),
        (86, 15, 22, 60, (0.34, 0.38, 0.16)),
        (5, -20, 38, 20, (0.36, 0.33, 0.15)),
        (-40, -60, 30, 18, (0.44, 0.38, 0.19)),
        (55, 5, 26, 22, (0.48, 0.45, 0.21)),
    ]
    for i, (x, y, w, d, col) in enumerate(field_specs):
        fields.visual(box(w, d, 0.02), (x, y, 0.012), col, name=f"field_{i}")
    add(fields.xml())

    # --- Дорожная сеть ------------------------------------------------------
    roads = Model("roads")
    add_road(roads, 0, -30, 205, 8.0, 0.0)                    # главная E-W
    add_road(roads, 25, 0, 205, 7.0, math.pi / 2)             # N-S через мост
    add_road(roads, -58, 0, 200, 6.0, math.pi / 2)            # к нефтебазе
    add_road(roads, 25.0, -30.0, 8.0, 8.0, 0.0)               # перекрёсток
    add_road(roads, -58.0, -30.0, 6.0, 8.0, 0.0)
    # серпантин к лесному очагу
    add_road(roads, 40, 4, 46, 4.5, math.atan2(20, 40))
    add_road(roads, 66, 22, 18, 4.5, math.atan2(4, 10))
    # стоянка у базы
    roads.visual(box(26, 14, 0.04), (30, -12, 0.025), (0.34, 0.34, 0.37),
                 name="parking")
    add_dashes(roads, 0, -30, 190, 8.0, 0.0, count=16)
    add_dashes(roads, 25, 0, 190, 7.0, math.pi / 2, count=16)
    add(roads.xml())

    # --- Река и мост --------------------------------------------------------
    river = Model("river")
    river.visual(box(240, 16, 0.06), (0, -62, 0.018, 0, 0, 0.06), (0.10, 0.38, 0.34),
                 specular=(0.6, 0.65, 0.7), name="water_surface")
    river.visual(box(240, 3.0, 0.04), (0, -54.2, 0.02, 0, 0, 0.06), (0.42, 0.38, 0.22),
                 name="sand_bank_north")
    river.visual(box(240, 3.0, 0.04), (0, -69.8, 0.02, 0, 0, 0.06), (0.42, 0.38, 0.22),
                 name="sand_bank_south")
    add(river.xml())

    bridge = Model("bridge")
    bridge.visual(box(11, 26, 0.7), (25, -62, 0.8), (0.42, 0.43, 0.45),
                  name="bridge_deck")
    bridge.visual(box(0.4, 26, 1.2), (19.7, -62, 1.7), (0.62, 0.62, 0.60),
                  name="rail_left")
    bridge.visual(box(0.4, 26, 1.2), (30.3, -62, 1.7), (0.62, 0.62, 0.60),
                  name="rail_right")
    for px in (19.0, 31.0):
        for py in (-74, -62, -50):
            bridge.visual(cylinder(0.8, 6.0), (px, py, -1.0), (0.45, 0.45, 0.45),
                          name=f"pier_{len(bridge.visuals)}")
    add(bridge.xml())

    # --- Вертодром базы (модель mchs_base_helipad) -------------------------
    pad = Model("mchs_base_helipad")
    pad.visual(cylinder(8.0, 0.10), (0, 0, 0.05), (0.55, 0.56, 0.58), name="concrete_pad")
    pad.visual(cylinder(7.2, 0.12), (0, 0, 0.07), (0.85, 0.42, 0.05), name="orange_ring")
    pad.visual(cylinder(6.6, 0.13), (0, 0, 0.08), (0.18, 0.18, 0.20), name="dark_ring")
    pad.visual(cylinder(5.9, 0.14), (0, 0, 0.09), (0.30, 0.31, 0.33), name="pad_inner")
    # буква H
    pad.visual(box(1.1, 4.2, 0.03), (-2.2, 0, 0.105), (0.88, 0.88, 0.85), name="h_left")
    pad.visual(box(1.1, 4.2, 0.03), (2.2, 0, 0.105), (0.88, 0.88, 0.85), name="h_right")
    pad.visual(box(5.5, 1.0, 0.03), (0, 0, 0.105), (0.88, 0.88, 0.85), name="h_bar")
    # огни площадки
    for i in range(8):
        ang = i * math.pi / 4.0
        pad.visual(sphere(0.22), (7.4 * math.cos(ang), 7.4 * math.sin(ang), 0.2),
                   (0.95, 0.85, 0.3), emissive=(0.9, 0.7, 0.2),
                   name=f"pad_light_{i}")
    # ангар
    pad.visual(box(17, 11, 6.0), (17, -9, 3.0), (0.55, 0.57, 0.60), name="hangar")
    pad.visual(box(17.6, 11.6, 0.8), (17, -9, 6.2), (0.38, 0.40, 0.44), name="hangar_roof")
    pad.visual(box(6.0, 0.3, 4.0), (17, -3.55, 2.0), (0.22, 0.24, 0.28), name="hangar_gate")
    pad.visual(box(16.8, 3.2, 0.06), (17, -3.4, 4.6), (0.30, 0.32, 0.36), name="awning")
    # вышка связи
    pad.visual(cylinder(0.22, 12.0), (-9, -9, 6.0), (0.62, 0.62, 0.60), name="mast")
    pad.visual(sphere(0.35), (-9, -9, 12.2), (0.95, 0.15, 0.10),
               emissive=(1.0, 0.1, 0.05), name="beacon")
    pad.visual(cylinder(0.10, 3.0), (-9, -9, 13.6), (0.7, 0.7, 0.7), name="antenna")
    # контейнеры
    for i, (cx, cy, col) in enumerate([
        (8.0, -18.0, (0.85, 0.40, 0.08)),
        (14.0, -18.0, (0.12, 0.32, 0.65)),
        (20.0, -18.0, (0.55, 0.55, 0.52)),
    ]):
        pad.visual(box(6.0, 2.5, 2.6), (cx, cy, 1.3), col, name=f"container_{i}")
    # мешки/бочки у ангара
    for i in range(4):
        add_barrel(pad, 8.5 + i * 1.1, -13.0, (0.35, 0.33, 0.30), h=1.0, r=0.4)
    add(pad.xml())

    # --- Лесной массив ------------------------------------------------------
    forest = Model("forest")
    tree_spots = [
        (52, 22, 1.0), (56, 30, 1.2), (63, 24, 0.9), (68, 32, 1.1), (60, 36, 1.0),
        (72, 24, 0.8), (48, 32, 0.95), (57, 41, 1.15), (66, 45, 0.9), (75, 38, 1.0),
        (44, 38, 0.85), (53, 47, 1.05), (61, 50, 0.95), (70, 52, 1.1), (79, 30, 0.9),
        (46, 20, 1.0), (74, 44, 0.85), (80, 48, 1.0), (36, 44, 0.8), (42, 52, 1.0),
        (90, 40, 0.9), (85, 55, 1.05), (30, 60, 0.85), (24, 52, 0.9), (18, 66, 1.0),
        (10, 55, 0.8), (0, 70, 0.95), (-8, 62, 0.9), (36, 70, 1.0), (50, 62, 0.9),
        (-30, 60, 0.85), (-45, 72, 1.0), (-18, 78, 0.9), (-62, 62, 0.8),
    ]
    for i, (x, y, s) in enumerate(tree_spots):
        variant = (0.09 + 0.03 * (i % 3), 0.30 + 0.04 * (i % 4), 0.09)
        add_tree(forest, x, y, scale=s, crown=variant)
    # кустарник вдоль дорог/реки
    for i in range(22):
        x = -92 + (i * 8.7) % 184
        y = -30 + 16 * math.sin(i * 1.7)
        forest.visual(sphere(0.9 + 0.3 * (i % 3)), (x, y, 0.8),
                      (0.12, 0.30 + 0.05 * (i % 3), 0.10), name=f"bush_{i}")
    add(forest.xml())

    # --- Посёлок и склады ---------------------------------------------------
    village = Model("village")
    houses = [
        (-32, -16, 9, 7, 3.2, 0.0), (-20, -18, 8, 6, 3.0, 0.1),
        (-42, -18, 10, 7, 3.4, -0.05), (-30, -24, 7, 6, 2.8, 0.15),
        (-16, -14, 8, 6, 3.1, -0.1), (-46, -13, 9, 6, 3.0, 0.0),
    ]
    for i, (x, y, w, d, h, r) in enumerate(houses):
        add_house(village, x, y, w, d, h, r)
        village.visual(box(w * 1.4, d * 1.4, 0.12), (x, y, 0.06), (0.35, 0.33, 0.28),
                       name=f"yard_{i}")
    # склады
    village.visual(box(24, 14, 7.0), (-4, 16, 3.5), (0.52, 0.54, 0.57), name="warehouse_1")
    village.visual(box(24.6, 14.6, 0.9), (-4, 16, 7.3), (0.35, 0.37, 0.41),
                   name="warehouse_roof_1")
    village.visual(box(18, 11, 6.0), (26, 16, 3.0), (0.58, 0.55, 0.48), name="warehouse_2")
    village.visual(box(18.6, 11.6, 0.8), (26, 16, 6.2), (0.36, 0.34, 0.30),
                   name="warehouse_roof_2")
    village.visual(box(9, 6, 0.5), (-34, -24, 0.25), (0.40, 0.40, 0.40), name="small_yard")
    add(village.xml())

    # --- Нефтебаза ----------------------------------------------------------
    depot = Model("oil_depot")
    for i, (x, y) in enumerate([(-58, 52), (-44, 52), (-58, 66), (-44, 66)]):
        add_tank(depot, x, y, radius=5.4, height=6.5)
    add_berm(depot, -51, 59, size=26.0)
    depot.visual(cylinder(0.35, 22), (-51, 52, 1.2, 0, math.pi / 2, 0),
                 (0.55, 0.55, 0.55), name="pipe_main")
    depot.visual(box(7, 5, 3.4), (-66, 44, 1.7), (0.60, 0.58, 0.52), name="pump_house")
    depot.visual(box(7.6, 5.6, 0.5), (-66, 44, 3.6), (0.34, 0.32, 0.28),
                 name="pump_roof")
    # сливная эстакада
    depot.visual(box(14, 2.2, 0.4), (-51, 44, 1.4), (0.42, 0.42, 0.42), name="loading_rack")
    for i in range(6):
        depot.visual(cylinder(0.2, 3.6), (-57 + i * 2.4, 44, 0.9), (0.4, 0.4, 0.4),
                     name=f"rack_leg_{i}")
    add(depot.xml())

    # --- Полигон ТБО / свалка ----------------------------------------------
    landfill = Model("landfill_site")
    landfill.visual(sphere(9.0), (68, -74, -5.0), (0.30, 0.24, 0.14),
                    name="waste_mound")
    landfill.visual(cylinder(11.0, 0.3), (68, -74, 0.15), (0.28, 0.24, 0.16),
                    name="waste_base")
    barrel_colors = [
        (0.85, 0.72, 0.05), (0.80, 0.30, 0.05), (0.15, 0.45, 0.18),
        (0.10, 0.30, 0.60), (0.55, 0.55, 0.55), (0.85, 0.72, 0.05),
        (0.80, 0.30, 0.05), (0.15, 0.45, 0.18), (0.10, 0.30, 0.60), (0.60, 0.35, 0.10),
    ]
    for i, col in enumerate(barrel_colors):
        ang = i * 0.63
        r = 4.0 + (i % 3) * 1.6
        add_barrel(landfill, 68 + r * math.cos(ang), -74 + r * math.sin(ang), col)
    add_fence(landfill, 56, -88, 80, -88)
    add_fence(landfill, 56, -88, 56, -60)
    add_fence(landfill, 80, -88, 80, -60)
    landfill.visual(box(4, 3, 2.2), (60, -66, 1.1), (0.42, 0.38, 0.30),
                    name="debris_1")
    landfill.visual(box(3, 4, 1.8), (75, -64, 0.9), (0.36, 0.34, 0.28),
                    name="debris_2")
    add(landfill.xml())

    # --- Холмы на краю карты ------------------------------------------------
    hills = Model("hills")
    for i, (x, y, r) in enumerate([(-85, -85, 34), (88, 75, 30), (-80, 88, 26)]):
        hills.visual(sphere(r), (x, y, 2.0 - r), (0.18 + 0.02 * i, 0.28, 0.12),
                     name=f"hill_{i}")
    add(hills.xml())

    # --- Спецтехника на базе ------------------------------------------------
    vehicles = Model("vehicles")
    # условные пожарные машины (корпус + кабина + мигалки)
    for i, (x, y, col) in enumerate([
        (27, -6, (0.80, 0.10, 0.08)), (31, -6, (0.80, 0.10, 0.08)),
        (27, -1.0, (0.90, 0.90, 0.88)),
    ]):
        vehicles.visual(box(6.0, 2.4, 2.2), (x, y, 1.2), col, name=f"truck_body_{i}")
        vehicles.visual(box(2.2, 2.4, 1.6), (x + 2.1, y, 1.5), (0.35, 0.38, 0.42),
                        name=f"truck_cab_{i}")
        for wx in (-1.8, 1.8):
            vehicles.visual(cylinder(0.55, 0.35), (x + wx, y + 1.1, 0.55, math.pi / 2, 0, 0),
                            (0.08, 0.08, 0.08), name=f"wheel_{len(vehicles.visuals)}")
            vehicles.visual(cylinder(0.55, 0.35), (x + wx, y - 1.1, 0.55, math.pi / 2, 0, 0),
                            (0.08, 0.08, 0.08), name=f"wheel_{len(vehicles.visuals)}")
        vehicles.visual(sphere(0.18), (x, y, 2.45), (0.2, 0.4, 1.0),
                        emissive=(0.2, 0.5, 1.0), name=f"beacon_{i}")
    add(vehicles.xml())

    # =======================================================================
    #  ОБЪЕКТЫ ЧС (модели с именами, которые двигает arena_controller.py)
    # =======================================================================

    # 1. Пожар в лесном массиве
    fire_x, fire_y, fire_z = hazard_poses["FIRE"]
    fire = Model("fire_hazard", pose=(fire_x, fire_y, fire_z, 0, 0, 0))
    fire.collision(cylinder(6.0, 0.6), (0, 0, 0))
    fire.visual(cylinder(8.5, 0.06), (0, 0, -0.17), (0.16, 0.10, 0.06), name="ash_outer")
    fire.visual(cylinder(6.5, 0.08), (0, 0, -0.15), (0.10, 0.06, 0.04), name="charred_ground")
    fire.visual(cylinder(4.6, 2.2), (0, 0, 0.9), (0.95, 0.35, 0.02), alpha=0.9,
                emissive=(1.0, 0.35, 0.0), name="flame_base")
    fire.visual(cylinder(3.0, 3.2), (0.6, -0.4, 1.5), (1.0, 0.62, 0.02), alpha=0.85,
                emissive=(1.0, 0.7, 0.0), name="flame_mid")
    fire.visual(cylinder(1.7, 3.6), (-0.5, 0.5, 2.0), (1.0, 0.85, 0.10), alpha=0.9,
                emissive=(1.0, 0.9, 0.2), name="flame_core")
    for i in range(7):
        ang = i * 0.9
        fire.visual(sphere(0.35), (4.2 * math.cos(ang), 4.2 * math.sin(ang), 0.25),
                    (1.0, 0.55, 0.05), emissive=(1.0, 0.4, 0.0),
                    name=f"ember_{i}")
    for i, (z, r, a) in enumerate([(6.0, 3.2, 0.30), (8.5, 4.2, 0.22), (11.0, 5.0, 0.15)]):
        fire.visual(sphere(r), (i * 0.8, -i * 0.4, z), (0.35, 0.33, 0.32), alpha=a,
                    name=f"smoke_{i}")
    add(fire.xml())

    # 2. Разлив нефтепродуктов
    oil_x, oil_y, oil_z = hazard_poses["OIL"]
    oil = Model("oil_spill_hazard", pose=(oil_x, oil_y, oil_z, 0, 0, 0))
    oil.collision(cylinder(9.0, 0.08), (0, 0, 0))
    oil.visual(cylinder(10.0, 0.05), (0, 0, -0.005), (0.10, 0.09, 0.08), name="spill_edge")
    oil.visual(cylinder(8.2, 0.06), (0, 0, 0.005), (0.03, 0.03, 0.04), name="spill_core")
    oil.visual(cylinder(6.0, 0.065), (0.6, -0.4, 0.01), (0.05, 0.05, 0.07),
               specular=(0.8, 0.8, 0.9), name="spill_sheen")
    oil.visual(cylinder(3.4, 0.07), (-0.8, 0.6, 0.012), (0.10, 0.09, 0.14),
               specular=(0.9, 0.9, 1.0), name="spill_gloss")
    add(oil.xml())

    # 3. Зона затопления / паводок
    flood_x, flood_y, flood_z = hazard_poses["FLOOD"]
    flood = Model("flood_hazard", pose=(flood_x, flood_y, flood_z, 0, 0, 0))
    flood.collision(cylinder(10.5, 0.10), (0, 0, 0))
    flood.visual(cylinder(11.5, 0.06), (0, 0, -0.002), (0.10, 0.32, 0.55), alpha=0.85,
                 specular=(0.7, 0.75, 0.8), name="flood_edge")
    flood.visual(cylinder(9.5, 0.07), (0, 0, 0.008), (0.08, 0.42, 0.70), alpha=0.8,
                 specular=(0.8, 0.85, 0.9), name="flood_water")
    flood.visual(cylinder(4.0, 0.08), (1.5, -1.0, 0.014), (0.20, 0.55, 0.80), alpha=0.75,
                 name="flood_ripple")
    flood.visual(box(3.0, 1.8, 0.8), (-6.0, 4.0, 0.4), (0.40, 0.38, 0.30),
                 name="flood_car_wreck")
    add(flood.xml())

    # 4. Несанкционированная свалка / токсичные отходы
    waste_x, waste_y, waste_z = hazard_poses["WASTE"]
    waste = Model("waste_hazard", pose=(waste_x, waste_y, waste_z, 0, 0, 0))
    waste.collision(box(9.0, 9.0, 1.4), (0, 0, 0.3))
    waste.visual(sphere(7.0), (0, 0, -3.0), (0.28, 0.22, 0.13), name="waste_hill")
    waste.visual(cylinder(9.0, 0.20), (0, 0, -0.25), (0.32, 0.26, 0.15), name="waste_pad")
    for i, col in enumerate([
        (0.85, 0.72, 0.05), (0.80, 0.30, 0.05), (0.15, 0.45, 0.18),
        (0.10, 0.30, 0.60), (0.55, 0.55, 0.55), (0.70, 0.12, 0.08),
    ]):
        ang = i * 1.05
        add_barrel(waste, 3.8 * math.cos(ang), 3.8 * math.sin(ang), col)
    waste.visual(box(2.6, 2.0, 1.4), (-2.5, -2.0, 0.3), (0.36, 0.33, 0.26),
                 name="waste_debris_1")
    add(waste.xml())

    # 5. Пострадавший / человек в сигнальном жилете
    human_pose = hazard_poses.get("HUMAN") or hazard_poses.get("VICTIM", (15.0, -22.0, 0.0))
    hx, hy, hz = human_pose[0], human_pose[1], human_pose[2]
    add(f'''    <!-- 5. Пострадавший (человек) в сигнальном жилете -->
    <include>
      <name>person_standing</name>
      <uri>model://person_standing</uri>
      <pose>{hx:.4f} {hy:.4f} {hz:.4f} 0 0 0</pose>
    </include>
''')

    # --- Дрон МЧС -----------------------------------------------------------
    add('''    <!-- Крупный дрон МЧС largedrone (STL by Studio Mechanik), высота 22 м -->
    <include>
      <uri>model://largedrone</uri>
      <pose>0 0 22.0 0 0 0</pose>
    </include>
''')

    add('  </world>\n</sdf>\n')
    return "".join(parts)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Генератор мира ЧС Gazebo")
    parser.add_argument("--output", "-o", default=None, help="Путь для сохранения .world файла")
    args = parser.parse_args()

    world_xml = build_world()
    world_path = os.path.abspath(args.output if args.output else WORLD_PATH)
    tmp_path = world_path + ".tmp"
    try:
        os.makedirs(os.path.dirname(world_path), exist_ok=True)
        with open(tmp_path, "w", encoding="utf-8") as f:
            f.write(world_xml)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, world_path)  # атомарная публикация: битый world не останется
    except OSError as exc:
        try:
            os.remove(tmp_path)
        except OSError:
            pass
        print(f"[ОШИБКА] Не удалось записать мир {world_path}: {exc}")
        return 1
    print(f"[OK] Мир сгенерирован: {world_path}")
    print(f"     Размер: {len(world_xml) / 1024.0:.1f} КБ, моделей: {world_xml.count('<model ')}")
    print("     Очаги из config/hazards.json:")
    for key, (hx, hy, hz) in load_hazard_poses().items():
        print(f"       {key:<5} -> X={hx:.1f} Y={hy:.1f} Z={hz:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
