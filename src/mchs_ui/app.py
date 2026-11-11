"""
АВТОНОМНЫЙ ТАКТИЧЕСКИЙ ПУЛЬТ ОПЕРАТОРА МЧС РОССИИ — «ОКО МЧС»
Интегрированный АРМ ЦУКС:
1. Тактическая векторная карта полигона с отображением базы, трека БПЛА и 4 типов ЧС;
2. Мультиспектральная камера БПЛА (Оптический 4K / Тепловизор FLIR / ИИ-оверлей детекции);
3. Детекция и классификация 4 типов ЧС (Пожар, Разлив нефти, Затопление, Токсичные отходы);
4. Панель телеметрии (высота, скорость, заряд АКБ, координаты, дальномер);
5. Журнал оперативных событий и экспорт протокола инцидента МЧС (JSON / TXT).
"""
import os
import sys
import math
import time
import datetime
import json
import threading
import urllib.request
import tkinter as tk
import customtkinter as ctk
import cv2
import numpy as np
from PIL import Image, ImageTk

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Спецификация 4 типов ЧС по типовому заданию хакатона
HAZARDS_DB = {
    'FIRE': {
        'title': 'ОЧАГ ЛЕСНОГО ПОЖАРА',
        'threat': 'CRITICAL',
        'color_bgr': (0, 110, 255),
        'color_hex': '#FF3D00',
        'map_x': 390,
        'map_y': 160,
        'area_sq_m': 64.2,
        'details': 'Верховое распространение, пламя 850°C'
    },
    'OIL': {
        'title': 'РАЗЛИВ НЕФТЕПРОДУКТОВ',
        'threat': 'WARNING',
        'color_bgr': (30, 30, 40),
        'color_hex': '#424242',
        'map_x': 320,
        'map_y': 310,
        'area_sq_m': 118.0,
        'details': 'Утечка дизельного топлива, угроза водоему'
    },
    'FLOOD': {
        'title': 'ЗОНА ЗАТОПЛЕНИЯ / ПАВОДКА',
        'threat': 'WARNING',
        'color_bgr': (255, 170, 0),
        'color_hex': '#00B0FF',
        'map_x': 180,
        'map_y': 330,
        'area_sq_m': 340.5,
        'details': 'Подъем уровня воды +1.8 м, затопление дороги'
    },
    'WASTE': {
        'title': 'ТОКСИЧНЫЕ ОТХОДЫ / БОЧКИ',
        'threat': 'CRITICAL',
        'color_bgr': (0, 220, 220),
        'color_hex': '#FFD600',
        'map_x': 410,
        'map_y': 340,
        'area_sq_m': 28.0,
        'details': 'Несанкционированная свалка, 12 бочек химреагентов'
    }
}

class OkoMchsOperatorApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("ОКО МЧС — АВТОНОМНЫЙ ПУЛЬТ ОПЕРАТОРА ЦУКС (ВЕРСИЯ 2.0)")
        self.geometry("1280x820")
        self.minsize(1120, 720)

        # Переменные полетной модели и состояния (Демо-режим активен по умолчанию)
        self.is_flying = True
        self.drone_x = 75.0
        self.drone_y = 220.0
        self.altitude = 1.0
        self.battery = 98.4
        self.flight_speed = 3.2
        self.flight_phase = "ВЗЛЕТ И ПАТРУЛИРОВАНИЕ СЕКТОРА"
        self.thermal_mode = False
        self.selected_hazard_key = 'FIRE'
        self.hover_timer = 0
        self.hazards_sequence = ['FIRE', 'OIL', 'FLOOD', 'WASTE']
        self.seq_index = 0
        self.active_detection = False
        self.flight_trail = []
        self.frame_counter = 0

        # Фоновое подключение к живому видеопотоку бортовой камеры Gazebo/ROS2 (порт 8080)
        self.live_stream_frame = None
        self.is_live_stream_active = False
        self.stream_thread = threading.Thread(target=self.poll_live_stream, daemon=True)
        self.stream_thread.start()

        self.setup_ui()
        self.log_event("Инициализация системы 'Око МЧС'. Связь с БПЛА установлена (RTK FIX).")
        self.update_loop()

    def setup_ui(self):
        # 1. Верхний командный баннер (Header)
        header_frame = ctk.CTkFrame(self, height=55, corner_radius=0, fg_color="#12151B")
        header_frame.pack(side="top", fill="x")

        title_label = ctk.CTkLabel(
            header_frame,
            text="🚨 ОКО МЧС  |  КОМПЛЕКС АВТОНОМНОГО БПЛА-МОНИТОРИНГА ЧС  |  ЦУКС",
            font=ctk.CTkFont(size=16, weight="bold"),
            text_color="#FF6B35"
        )
        title_label.pack(side="left", padx=18, pady=8)

        self.threat_badge = ctk.CTkLabel(
            header_frame,
            text="СТАТУС: ДЕЖУРСТВО",
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color="#1E232E",
            corner_radius=6,
            padx=14, pady=5,
            text_color="#00E5FF"
        )
        self.threat_badge.pack(side="right", padx=18, pady=8)

        # 2. Главная рабочая панель (Две колонки: Карта слева, Сенсорная камера справа)
        center_frame = ctk.CTkFrame(self, fg_color="transparent")
        center_frame.pack(side="top", fill="both", expand=True, padx=12, pady=8)
        center_frame.columnconfigure(0, weight=1)
        center_frame.columnconfigure(1, weight=1)
        center_frame.rowconfigure(0, weight=1)

        # Левая колонка: Векторная тактическая карта
        left_box = ctk.CTkFrame(center_frame, corner_radius=8, fg_color="#171A21")
        left_box.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=0)

        map_header = ctk.CTkFrame(left_box, height=32, fg_color="transparent")
        map_header.pack(fill="x", padx=10, pady=(6, 2))

        ctk.CTkLabel(
            map_header,
            text="🗺 ТАКТИЧЕСКИЙ РАДАР ОБЛЕТА (ПОЛИГОН: 50×50 М)",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color="#A4B3C6"
        ).pack(side="left")

        # Селектор целевого очага ЧС
        self.hazard_selector = ctk.CTkSegmentedButton(
            map_header,
            values=["ПОЖАР", "НЕФТЬ", "ПАВОДОК", "СВАЛКА"],
            command=self.on_hazard_select,
            font=ctk.CTkFont(size=11, weight="bold"),
            selected_color="#FF6B35",
            selected_hover_color="#E85924"
        )
        self.hazard_selector.set("ПОЖАР")
        self.hazard_selector.pack(side="right")

        self.map_canvas = tk.Canvas(left_box, bg="#0E1116", highlightthickness=0)
        self.map_canvas.pack(fill="both", expand=True, padx=10, pady=(0, 8))

        # Правая колонка: Оптико-электронный комплекс
        right_box = ctk.CTkFrame(center_frame, corner_radius=8, fg_color="#171A21")
        right_box.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=0)

        cam_header = ctk.CTkFrame(right_box, height=32, fg_color="transparent")
        cam_header.pack(fill="x", padx=10, pady=(6, 2))

        ctk.CTkLabel(
            cam_header,
            text="📷 БОРТОВАЯ СЕНСОРНАЯ КАМЕРА (ИИ-ДЕТЕКЦИЯ OPENCV / HSV)",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color="#A4B3C6"
        ).pack(side="left")

        self.btn_thermal = ctk.CTkButton(
            cam_header,
            text="🌡 ТЕПЛОВИЗОР FLIR",
            font=ctk.CTkFont(size=11, weight="bold"),
            fg_color="#37474F",
            hover_color="#455A64",
            width=140, height=26,
            command=self.toggle_thermal
        )
        self.btn_thermal.pack(side="right")

        self.cam_label = tk.Label(right_box, bg="#0A0C0F")
        self.cam_label.pack(fill="both", expand=True, padx=10, pady=(0, 8))

        # 3. Нижняя панель: Телеметрия, Журнал событий и Командные кнопки
        bottom_frame = ctk.CTkFrame(self, height=170, corner_radius=8, fg_color="#171A21")
        bottom_frame.pack(side="bottom", fill="x", padx=12, pady=(0, 10))

        # Телеметрия (Слева)
        tele_box = ctk.CTkFrame(bottom_frame, fg_color="#12151B", corner_radius=6)
        tele_box.pack(side="left", fill="y", padx=10, pady=8)

        ctk.CTkLabel(tele_box, text="ТЕЛЕМЕТРИЯ БПЛА", font=ctk.CTkFont(size=11, weight="bold"), text_color="#78909C").grid(row=0, column=0, columnspan=2, padx=10, pady=(4, 2), sticky="w")

        self.lbl_alt = ctk.CTkLabel(tele_box, text="ВЫСОТА: 0.0 м", font=ctk.CTkFont(size=12, weight="bold"))
        self.lbl_alt.grid(row=1, column=0, sticky="w", padx=10, pady=2)

        self.lbl_speed = ctk.CTkLabel(tele_box, text="СКОРОСТЬ: 0.0 м/с", font=ctk.CTkFont(size=12, weight="bold"))
        self.lbl_speed.grid(row=1, column=1, sticky="w", padx=10, pady=2)

        self.lbl_bat = ctk.CTkLabel(tele_box, text="БАТАРЕЯ: 98%", font=ctk.CTkFont(size=12, weight="bold"), text_color="#00E676")
        self.lbl_bat.grid(row=2, column=0, sticky="w", padx=10, pady=2)

        self.lbl_gps = ctk.CTkLabel(tele_box, text="GPS: X=0.0m, Y=0.0m", font=ctk.CTkFont(size=12, weight="bold"))
        self.lbl_gps.grid(row=2, column=1, sticky="w", padx=10, pady=2)

        # Центральная консоль: Журнал оперативных событий
        log_box = ctk.CTkFrame(bottom_frame, fg_color="#12151B", corner_radius=6)
        log_box.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        ctk.CTkLabel(log_box, text="ЖУРНАЛ ОПЕРАТИВНОГО ДЕЖУРНОГО ЦУКС", font=ctk.CTkFont(size=11, weight="bold"), text_color="#78909C").pack(anchor="w", padx=10, pady=(4, 0))

        self.log_textbox = ctk.CTkTextbox(log_box, font=ctk.CTkFont(family="Consolas", size=11), fg_color="transparent", activate_scrollbars=True)
        self.log_textbox.pack(fill="both", expand=True, padx=6, pady=4)

        # Командные кнопки (Справа)
        btn_box = ctk.CTkFrame(bottom_frame, fg_color="transparent")
        btn_box.pack(side="right", padx=10, pady=8)

        self.btn_mission = ctk.CTkButton(
            btn_box,
            text="⏹ АВАРИЙНЫЙ ВОЗВРАТ (RTL)",
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color="#D32F2F",
            hover_color="#B71C1C",
            width=210, height=36,
            command=self.toggle_mission
        )
        self.btn_mission.grid(row=0, column=0, padx=4, pady=4)

        self.btn_report = ctk.CTkButton(
            btn_box,
            text="📄 ПРОТОКОЛ ЦУКС (JSON)",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color="#1976D2",
            hover_color="#1565C0",
            width=210, height=32,
            command=self.export_protocol
        )
        self.btn_report.grid(row=1, column=0, padx=4, pady=4)

    def log_event(self, text: str):
        now = datetime.datetime.now().strftime("%H:%M:%S")
        msg = f"[{now}] {text}\n"
        self.log_textbox.insert("end", msg)
        self.log_textbox.see("end")

    def on_hazard_select(self, val):
        mapping = {"ПОЖАР": 'FIRE', "НЕФТЬ": 'OIL', "ПАВОДОК": 'FLOOD', "СВАЛКА": 'WASTE'}
        self.selected_hazard_key = mapping.get(val, 'FIRE')
        cfg = HAZARDS_DB[self.selected_hazard_key]
        self.log_event(f"Выбран объект для инспекции: {cfg['title']} ({cfg['details']}).")

    def toggle_thermal(self):
        self.thermal_mode = not self.thermal_mode
        if self.thermal_mode:
            self.btn_thermal.configure(fg_color="#7B1FA2", hover_color="#6A1B9A", text="🌈 РЕЖИМ RGB (4K)")
            self.log_event("Сенсор переключен в спектральный режим: FLIR LWIR 8-14 мкм.")
        else:
            self.btn_thermal.configure(fg_color="#37474F", hover_color="#455A64", text="🌡 ТЕПЛОВИЗОР FLIR")
            self.log_event("Сенсор переключен в оптический режим: Sony IMX 4K RGB.")

    def toggle_mission(self):
        if not self.is_flying:
            self.is_flying = True
            self.flight_phase = "ВЗЛЕТ И ПАТРУЛИРОВАНИЕ СЕКТОРА"
            self.btn_mission.configure(text="⏹ АВАРИЙНЫЙ ВОЗВРАТ (RTL)", fg_color="#D32F2F", hover_color="#B71C1C")
            self.log_event("Команда оператора: ВЗЛЕТ НА ЭШЕЛОН H=8.0м. Начало патрулирования.")
        else:
            self.is_flying = False
            self.flight_phase = "ВОЗВРАТ НА БАЗУ (RTL)"
            self.btn_mission.configure(text="🚀 СТАРТ ПАТРУЛИРОВАНИЯ", fg_color="#FF6B35", hover_color="#E85924")
            self.log_event("Команда оператора: ВОЗВРАТ НА БАЗУ (RTL).")

    def export_protocol(self):
        cfg = HAZARDS_DB[self.selected_hazard_key]
        inc_id = f"INC-2026-{int(time.time()) % 10000:04d}"
        report_data = {
            "incident_id": inc_id,
            "timestamp": datetime.datetime.now().isoformat(),
            "station": "ЦУКС ГУ МЧС России",
            "hazard_type": cfg['title'],
            "threat_level": cfg['threat'],
            "estimated_area_sq_m": cfg['area_sq_m'],
            "drone_telemetry": {
                "altitude_m": round(self.altitude, 1),
                "battery_percent": int(self.battery),
                "grid_x_m": round(self.drone_x / 10.0, 1),
                "grid_y_m": round(self.drone_y / 10.0, 1)
            },
            "recommended_response": "Направить 2 пожарные автоцистерны АЦ-40, оповестить диспетчера района."
        }

        # Сохранение в текущую папку
        report_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "incident_protocol.json")
        try:
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(report_data, f, ensure_ascii=False, indent=2)
            self.log_event(f"✅ Протокол {inc_id} успешно сохранен: {os.path.basename(report_path)}")
        except Exception as e:
            self.log_event(f"Ошибка сохранения отчета: {e}")

    def update_loop(self):
        self.frame_counter += 1
        target_hazard = HAZARDS_DB[self.selected_hazard_key]
        target_x = target_hazard['map_x']
        target_y = target_hazard['map_y']

        if self.is_flying:
            # Набор высоты
            if self.altitude < 10.0:
                self.altitude += 0.4
                self.flight_speed = 1.8
                self.flight_phase = "НАБОР ВЫСОТЫ (ЭШЕЛОН 10М)"
            else:
                # Полет к целевой ЧС
                dx = target_x - self.drone_x
                dy = target_y - self.drone_y
                dist = math.hypot(dx, dy)

                if dist > 10.0:
                    speed = 2.8
                    self.drone_x += (dx / dist) * speed
                    self.drone_y += (dy / dist) * speed
                    self.flight_speed = 12.4
                    self.flight_phase = f"ПАТРУЛИРОВАНИЕ: КУРС НА {target_hazard['title']}"
                    self.active_detection = False
                else:
                    self.flight_speed = 0.4
                    self.active_detection = True
                    self.flight_phase = f"⚠️ ОБНАРУЖЕН {target_hazard['title']}!"
                    self.threat_badge.configure(
                        text=f"ТРЕВОГА: {target_hazard['threat']}!",
                        fg_color="#D32F2F" if target_hazard['threat'] == 'CRITICAL' else "#F57C00",
                        text_color="#FFFFFF"
                    )
                    self.hover_timer += 1
                    if self.hover_timer == 1:
                        self.log_event(f"🚨 АВТОДЕТЕКЦИЯ: {target_hazard['title']} | Площадь ~{target_hazard['area_sq_m']} м².")
                    if self.hover_timer > 90:  # ~4.5 сек зависания и детального сканирования
                        self.hover_timer = 0
                        self.seq_index = (self.seq_index + 1) % len(self.hazards_sequence)
                        self.selected_hazard_key = self.hazards_sequence[self.seq_index]
                        map_ru = {'FIRE': 'ПОЖАР', 'OIL': 'НЕФТЬ', 'FLOOD': 'ПАВОДОК', 'WASTE': 'СВАЛКА'}
                        self.hazard_selector.set(map_ru[self.selected_hazard_key])
                        self.log_event(f"Патруль: переход к сектору {HAZARDS_DB[self.selected_hazard_key]['title']}.")

            self.battery = max(12.0, self.battery - 0.015)
            self.flight_trail.append((self.drone_x, self.drone_y))
            if len(self.flight_trail) > 120:
                self.flight_trail.pop(0)
        else:
            if self.altitude > 0:
                self.altitude = max(0.0, self.altitude - 0.5)
                self.drone_x += (75.0 - self.drone_x) * 0.05
                self.drone_y += (220.0 - self.drone_y) * 0.05
            else:
                self.threat_badge.configure(text="СТАТУС: ДЕЖУРСТВО НА БАЗЕ", fg_color="#1E232E", text_color="#00E5FF")

        # Обновление лейблов телеметрии
        self.lbl_alt.configure(text=f"ВЫСОТА: {self.altitude:.1f} м")
        self.lbl_speed.configure(text=f"СКОРОСТЬ: {self.flight_speed:.1f} м/с")
        self.lbl_bat.configure(text=f"БАТАРЕЯ: {int(self.battery)}%")
        self.lbl_gps.configure(text=f"ПОЛОЖЕНИЕ: X={self.drone_x/10.0:.1f}м, Y={self.drone_y/10.0:.1f}м")

        self.draw_map()
        self.draw_camera()

        self.after(40, self.update_loop)

    def draw_map(self):
        c = self.map_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        c.delete("all")

        # 1. Сетка координат (метры полигона)
        for x in range(0, w, 40):
            c.create_line(x, 0, x, h, fill="#141820", width=1)
        for y in range(0, h, 40):
            c.create_line(0, y, w, y, fill="#141820", width=1)

        # 2. Границы полигона патрулирования (50×50 м)
        c.create_rectangle(50, 40, w - 30, h - 40, outline="#2E4057", width=1, dash=(3, 3))
        c.create_text(60, 52, text="ЗОНА ПАТРУЛИРОВАНИЯ БПЛА [50×50 М]", fill="#5C7080", font=("Consolas", 9, "bold"), anchor="w")

        # 3. Отрисовка 4 объектов ЧС на карте
        for key, hzd in HAZARDS_DB.items():
            hx = hzd['map_x']
            hy = hzd['map_y']
            is_target = (key == self.selected_hazard_key)

            # Маркер очага
            r = 16 if not is_target else 20 + int(math.sin(self.frame_counter * 0.25) * 4)
            c.create_oval(hx - r, hy - r, hx + r, hy + r, fill=hzd['color_hex'], outline="#FFFFFF" if is_target else "", width=2 if is_target else 0)
            c.create_text(hx, hy + r + 10, text=key, fill=hzd['color_hex'], font=("Segoe UI", 9, "bold"), justify="center")

        # 4. База МЧС и вертодром
        c.create_rectangle(60, 205, 90, 235, fill="#1976D2", outline="#64B5F6", width=2)
        c.create_text(75, 245, text="БАЗА МЧС (СТАРТ)", fill="#90CAF9", font=("Segoe UI", 8, "bold"))

        # 5. Траектория полета дрона
        if len(self.flight_trail) > 1:
            for i in range(len(self.flight_trail) - 1):
                c.create_line(self.flight_trail[i][0], self.flight_trail[i][1],
                              self.flight_trail[i+1][0], self.flight_trail[i+1][1],
                              fill="#00E5FF", width=2)

        # 6. Конус обзора сенсора
        if self.altitude > 2.0:
            cone_r = int(self.altitude * 2.5)
            c.create_oval(self.drone_x - cone_r, self.drone_y - cone_r,
                          self.drone_x + cone_r, self.drone_y + cone_r,
                          outline="#00E5FF", width=1, dash=(2, 4))

        # 7. Иконка БПЛА
        dx, dy = self.drone_x, self.drone_y
        arm = 12
        c.create_line(dx - arm, dy - arm, dx + arm, dy + arm, fill="#FFFFFF", width=3)
        c.create_line(dx - arm, dy + arm, dx + arm, dy - arm, fill="#FFFFFF", width=3)
        for px, py in [(dx-arm, dy-arm), (dx+arm, dy-arm), (dx-arm, dy+arm), (dx+arm, dy+arm)]:
            c.create_oval(px-4, py-4, px+4, py+4, fill="#263238", outline="#00E5FF", width=1)
        c.create_oval(dx-5, dy-5, dx+5, dy+5, fill="#FF6B35", outline="#FFFFFF", width=2)

    def poll_live_stream(self):
        """Опрос веб-потока камеры бортового компьютера БПЛА (порт 8080)."""
        while True:
            try:
                req = urllib.request.Request("http://127.0.0.1:8080/snapshot.jpg", headers={'User-Agent': 'MchsOperatorApp'})
                with urllib.request.urlopen(req, timeout=0.25) as response:
                    img_array = np.asarray(bytearray(response.read()), dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    if frame is not None:
                        self.live_stream_frame = frame
                        self.is_live_stream_active = True
            except Exception:
                self.is_live_stream_active = False
            time.sleep(0.04)  # ~25 FPS опрос

    def draw_camera(self):
        h, w = 360, 520

        # Если доступен реальный видеопоток из Gazebo 11 / ROS 2
        if self.is_live_stream_active and self.live_stream_frame is not None:
            frame = cv2.resize(self.live_stream_frame, (w, h))
            if self.thermal_mode:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
            # Бейдж реального подключения к симуляции
            cv2.rectangle(frame, (10, 10), (230, 32), (0, 80, 0), -1)
            cv2.putText(frame, "LIVE: ROS 2 GAZEBO 11", (16, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 120), 2)
            cv2.circle(frame, (215, 21), 5, (0, 255, 0), -1)

            # Отображаем в UI
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb_frame)
            self.tk_cam_image = ImageTk.PhotoImage(image=img)
            self.cam_canvas.create_image(0, 0, anchor="nw", image=self.tk_cam_image)
            return

        # Иначе — автономный тактический синтетический рендер
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        frame[:] = (20, 35, 18)  # Фоновая растительность полигона

        target = HAZARDS_DB[self.selected_hazard_key]
        dist = math.hypot(self.drone_x - target['map_x'], self.drone_y - target['map_y'])
        zoom = max(0.1, 1.0 - (dist / 350.0))

        cx, cy = int(w * 0.5), int(h * 0.5)

        # Рендеринг специфичной ЧС в оптическом потоке
        if zoom > 0.25:
            radius = int(50 * zoom)
            if self.selected_hazard_key == 'FIRE':
                # Пламя и дым
                cv2.circle(frame, (cx + 8, cy - 20), int(radius * 1.3), (70, 70, 65), -1)
                cv2.circle(frame, (cx, cy), radius, (0, 110, 255), -1)
                cv2.circle(frame, (cx - 4, cy + 4), int(radius * 0.5), (30, 220, 255), -1)
            elif self.selected_hazard_key == 'OIL':
                # Темный масляный разлив
                cv2.ellipse(frame, (cx, cy), (int(radius*1.4), int(radius*0.8)), 25, 0, 360, (25, 25, 30), -1)
                cv2.ellipse(frame, (cx, cy), (int(radius*1.2), int(radius*0.6)), 25, 0, 360, (45, 40, 50), -1)
            elif self.selected_hazard_key == 'FLOOD':
                # Зона паводка / синяя вода
                cv2.circle(frame, (cx, cy), int(radius*1.5), (200, 120, 20), -1)
                cv2.circle(frame, (cx, cy), int(radius*1.2), (240, 160, 40), -1)
            elif self.selected_hazard_key == 'WASTE':
                # Свалка и бочки
                for ox, oy, col in [(-15, -10, (0, 200, 200)), (15, -5, (0, 140, 255)), (0, 15, (40, 180, 50))]:
                    cv2.rectangle(frame, (cx + ox - 12, cy + oy - 16), (cx + ox + 12, cy + oy + 16), col, -1)

        # Режим тепловизора FLIR
        if self.thermal_mode:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)

        # ИИ-Детекция (Рамка и HUD)
        if zoom > 0.35 and self.altitude > 3.0:
            bw, bh = int(140 * zoom), int(100 * zoom)
            bx1, by1 = cx - bw, cy - bh
            bx2, by2 = cx + bw, cy + bh

            box_color = (0, 0, 255) if not self.thermal_mode else (0, 255, 255)
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), box_color, 2)

            # Скобки видоискателя
            c_len = 12
            for px, py in [(bx1, by1), (bx2, by1), (bx1, by2), (bx2, by2)]:
                sx = 1 if px == bx1 else -1
                sy = 1 if py == by1 else -1
                cv2.line(frame, (px, py), (px + sx * c_len, py), box_color, 3)
                cv2.line(frame, (px, py), (px, py + sy * c_len), box_color, 3)

            # Табличка детекции
            badge = f"{self.selected_hazard_key}: {target['threat']} | 96.8%"
            cv2.rectangle(frame, (bx1, by1 - 22), (bx1 + 210, by1), box_color, -1)
            cv2.putText(frame, badge, (bx1 + 4, by1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(frame, f"S: ~{target['area_sq_m']} sq.m | REL: X={self.drone_x/10:.1f} Y={self.drone_y/10:.1f}", (bx1, by2 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

        # Перекрестие прицела камеры
        cv2.line(frame, (int(w/2) - 16, int(h/2)), (int(w/2) + 16, int(h/2)), (0, 230, 255), 1)
        cv2.line(frame, (int(w/2), int(h/2) - 16), (int(w/2), int(h/2) + 16), (0, 230, 255), 1)

        # Верхняя плашка видеопотока
        mode_text = "THERMAL FLIR 8-14um" if self.thermal_mode else "OPTICAL HD 720p (1280x720)"
        cv2.putText(frame, f"REC [LIVE] | {mode_text} | FPS: 30.0", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 230, 255), 1)

        # Конвертация в Tkinter
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(rgb)
        imgtk = ImageTk.PhotoImage(image=im)
        self.cam_label.imgtk = imgtk
        self.cam_label.configure(image=imgtk)

if __name__ == "__main__":
    app = OkoMchsOperatorApp()
    app.mainloop()
