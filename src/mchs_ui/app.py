"""
Комплекс мониторинга и раннего обнаружения ЧС 'Око МЧС'.
Автономный тактический пульт оператора с картой патрулирования, тепловизором и ИИ-детекцией.
"""
import tkinter as tk
import customtkinter as ctk
import cv2
import numpy as np
from PIL import Image, ImageTk
import math
import time

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class OkoMchsApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("АВТОНОМНЫЙ КОМПЛЕКС МОНИТОРИНГА 'ОКО МЧС' — ЦУКС")
        self.geometry("1240x780")
        self.minsize(1100, 700)
        
        # Выводим окно поверх всех окон
        self.attributes('-topmost', True)
        self.lift()
        self.focus_force()
        self.after(1000, lambda: self.attributes('-topmost', False))

        # Переменные состояния
        self.is_flying = False
        self.drone_x = 80
        self.drone_y = 350
        self.target_x = 380
        self.target_y = 180
        self.fire_x = 380
        self.fire_y = 180
        self.altitude = 0.0
        self.battery = 98.0
        self.flight_phase = "ДЕЖУРСТВО НА БАЗЕ"
        self.thermal_mode = False
        self.fire_detected = False
        self.flight_trail = []
        self.frame_counter = 0

        self.setup_ui()
        self.update_loop()

    def setup_ui(self):
        # 1. Верхняя панель (Заголовок и статус)
        header_frame = ctk.CTkFrame(self, height=60, corner_radius=0, fg_color="#181B20")
        header_frame.pack(side="top", fill="x")

        title_label = ctk.CTkLabel(
            header_frame, 
            text="🚨 ОКО МЧС | ЕДИНАЯ СИСТЕМА ДРОН-МОНИТОРИНГА И ЛИКВИДАЦИИ ЧС",
            font=ctk.CTkFont(size=18, weight="bold"),
            text_color="#FF6B35"
        )
        title_label.pack(side="left", padx=20, pady=10)

        self.phase_badge = ctk.CTkLabel(
            header_frame,
            text=f"СТАТУС: {self.flight_phase}",
            font=ctk.CTkFont(size=14, weight="bold"),
            fg_color="#2A2E39",
            corner_radius=6,
            padx=15, pady=6,
            text_color="#00E5FF"
        )
        self.phase_badge.pack(side="right", padx=20, pady=10)

        # 2. Центральная рабочая область (2 колонки)
        main_body = ctk.CTkFrame(self, fg_color="transparent")
        main_body.pack(side="top", fill="both", expand=True, padx=15, pady=10)
        main_body.columnconfigure(0, weight=1)
        main_body.columnconfigure(1, weight=1)
        main_body.rowconfigure(0, weight=1)

        # Левая колонка: Тактическая карта
        left_box = ctk.CTkFrame(main_body, corner_radius=10, fg_color="#1E222B")
        left_box.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=0)

        map_title = ctk.CTkLabel(
            left_box, 
            text="🗺 ТАКТИЧЕСКАЯ КАРТА ОБЛЕТА (РАЙОН: МАГНИТОГОРСКИЙ ЛЕСХОЗ)",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color="#A0AAB5"
        )
        map_title.pack(anchor="w", padx=15, pady=(10, 5))

        self.map_canvas = tk.Canvas(left_box, bg="#12151B", highlightthickness=0)
        self.map_canvas.pack(fill="both", expand=True, padx=12, pady=(0, 10))

        # Правая колонка: Камера БПЛА + Нейросеть
        right_box = ctk.CTkFrame(main_body, corner_radius=10, fg_color="#1E222B")
        right_box.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=0)

        cam_title = ctk.CTkLabel(
            right_box,
            text="📷 ОПТИКО-ЭЛЕКТРОННАЯ СИСТЕМА БПЛА (ИИ-ДЕКТЕКТОР YOLOv8)",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color="#A0AAB5"
        )
        cam_title.pack(anchor="w", padx=15, pady=(10, 5))

        self.cam_label = tk.Label(right_box, bg="#0E1015")
        self.cam_label.pack(fill="both", expand=True, padx=12, pady=(0, 10))

        # 3. Нижняя панель (Телеметрия + Кнопки)
        bottom_frame = ctk.CTkFrame(self, height=130, corner_radius=10, fg_color="#1E222B")
        bottom_frame.pack(side="bottom", fill="x", padx=15, pady=(0, 15))

        # Телеметрия (Слева)
        tele_box = ctk.CTkFrame(bottom_frame, fg_color="transparent")
        tele_box.pack(side="left", fill="both", expand=True, padx=20, pady=10)

        self.lbl_alt = ctk.CTkLabel(tele_box, text="ВЫСОТА: 0.0 м", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_alt.grid(row=0, column=0, sticky="w", padx=15, pady=4)

        self.lbl_speed = ctk.CTkLabel(tele_box, text="СКОРОСТЬ: 0.0 м/с", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_speed.grid(row=0, column=1, sticky="w", padx=15, pady=4)

        self.lbl_bat = ctk.CTkLabel(tele_box, text="БАТАРЕЯ: 98%", font=ctk.CTkFont(size=14, weight="bold"), text_color="#00E676")
        self.lbl_bat.grid(row=1, column=0, sticky="w", padx=15, pady=4)

        self.lbl_gps = ctk.CTkLabel(tele_box, text="GPS: 53.4129° N, 58.9814° E (RTK FIX)", font=ctk.CTkFont(size=14, weight="bold"))
        self.lbl_gps.grid(row=1, column=1, sticky="w", padx=15, pady=4)

        # Кнопки управления (Справа)
        btn_box = ctk.CTkFrame(bottom_frame, fg_color="transparent")
        btn_box.pack(side="right", padx=20, pady=10)

        self.btn_launch = ctk.CTkButton(
            btn_box, 
            text="🚀 СТАРТ ПАТРУЛИРОВАНИЯ", 
            font=ctk.CTkFont(size=14, weight="bold"),
            fg_color="#FF6B35", 
            hover_color="#E85A20",
            width=220, height=42,
            command=self.toggle_mission
        )
        self.btn_launch.grid(row=0, column=0, padx=8, pady=5)

        self.btn_thermal = ctk.CTkButton(
            btn_box, 
            text="🌡 РЕЖИМ ТЕПЛОВИЗОРА (FLIR)", 
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color="#37474F", 
            hover_color="#455A64",
            width=220, height=36,
            command=self.toggle_thermal
        )
        self.btn_thermal.grid(row=1, column=0, padx=8, pady=5)

    def toggle_mission(self):
        if not self.is_flying:
            self.is_flying = True
            self.flight_phase = "ВЗЛЕТ И ПАТРУЛИРОВАНИЕ СЕКТОРА"
            self.btn_launch.configure(text="⏹ АВАРИЙНЫЙ ВОЗВРАТ (RTL)", fg_color="#D32F2F", hover_color="#B71C1C")
        else:
            self.flight_phase = "ВОЗВРАТ НА БАЗУ (RTL)"
            self.btn_launch.configure(text="🚀 СТАРТ ПАТРУЛИРОВАНИЯ", fg_color="#FF6B35", hover_color="#E85A20")
            self.is_flying = False

    def toggle_thermal(self):
        self.thermal_mode = not self.thermal_mode
        if self.thermal_mode:
            self.btn_thermal.configure(fg_color="#7B1FA2", hover_color="#6A1B9A", text="🌈 РЕЖИМ RGB (КАМЕРА)")
        else:
            self.btn_thermal.configure(fg_color="#37474F", hover_color="#455A64", text="🌡 РЕЖИМ ТЕПЛОВИЗОРА (FLIR)")

    def update_loop(self):
        self.frame_counter += 1

        # Логика полета
        if self.is_flying:
            if self.altitude < 45.0:
                self.altitude += 1.5
            else:
                # Движение к очагу
                dx = self.target_x - self.drone_x
                dy = self.target_y - self.drone_y
                dist = math.hypot(dx, dy)
                if dist > 8:
                    speed = 3.2
                    self.drone_x += (dx / dist) * speed
                    self.drone_y += (dy / dist) * speed
                    self.flight_phase = "АВТОНОМНЫЙ ПОЛЕТ К КВАДРАТУ №4"
                    self.lbl_speed.configure(text="СКОРОСТЬ: 14.2 м/с")
                else:
                    self.fire_detected = True
                    self.flight_phase = "🔥 ОБНАРУЖЕН ОЧАГ ЧС! ПЕРЕДАЧА В ЦУКС"
                    self.phase_badge.configure(fg_color="#D32F2F", text_color="#FFFFFF")
                    self.lbl_speed.configure(text="СКОРОСТЬ: 1.5 м/с (ЗАВИСАНИЕ)")

            self.battery = max(10.0, self.battery - 0.02)
            self.flight_trail.append((self.drone_x, self.drone_y))
            if len(self.flight_trail) > 150:
                self.flight_trail.pop(0)
        else:
            if self.altitude > 0:
                self.altitude = max(0.0, self.altitude - 2.0)
                # Возврат к базе
                self.drone_x += (80 - self.drone_x) * 0.05
                self.drone_y += (350 - self.drone_y) * 0.05

        # Обновление надписей
        self.phase_badge.configure(text=f"СТАТУС: {self.flight_phase}")
        self.lbl_alt.configure(text=f"ВЫСОТА: {self.altitude:.1f} м")
        self.lbl_bat.configure(text=f"БАТАРЕЯ: {int(self.battery)}%")

        # Отрисовка карты
        self.draw_map()

        # Отрисовка камеры
        self.draw_camera()

        self.after(50, self.update_loop)

    def draw_map(self):
        c = self.map_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        c.delete("all")

        # Сетка координат
        for x in range(0, w, 50):
            c.create_line(x, 0, x, h, fill="#1B202B", width=1)
        for y in range(0, h, 50):
            c.create_line(0, y, w, y, fill="#1B202B", width=1)

        # Река / водоем
        river_pts = [0, h*0.8, w*0.3, h*0.65, w*0.6, h*0.75, w, h*0.6, w, h, 0, h]
        c.create_polygon(river_pts, fill="#0F2438", smooth=True)

        # Лесной массив (квадрат патрулирования)
        c.create_rectangle(150, 60, w-40, h-80, outline="#2E5A44", width=2, dash=(4, 4))
        c.create_text(160, 75, text="СЕКТОР ПАТРУЛИРОВАНИЯ #4", fill="#588B71", font=("Segoe UI", 9, "bold"), anchor="w")

        # Очаг ЧС (Огонь)
        fire_r = 18 + int(math.sin(self.frame_counter * 0.2) * 4)
        c.create_oval(self.fire_x - fire_r*1.8, self.fire_y - fire_r*1.8, self.fire_x + fire_r*1.8, self.fire_y + fire_r*1.8, fill="#3D1A16", outline="")
        c.create_oval(self.fire_x - fire_r, self.fire_y - fire_r, self.fire_x + fire_r, self.fire_y + fire_r, fill="#D84315", outline="#FF6E40", width=2)
        c.create_text(self.fire_x, self.fire_y + 30, text="ОЧАГ ВОЗГОРАНИЯ\n(53.4129° N, 58.9814° E)", fill="#FF8A65", font=("Segoe UI", 9, "bold"), justify="center")

        # База МЧС
        c.create_rectangle(65, 335, 95, 365, fill="#1565C0", outline="#64B5F6", width=2)
        c.create_text(80, 380, text="БАЗА МЧС (СТАРТ)", fill="#90CAF9", font=("Segoe UI", 9, "bold"))

        # След полета
        if len(self.flight_trail) > 1:
            for i in range(len(self.flight_trail) - 1):
                c.create_line(self.flight_trail[i][0], self.flight_trail[i][1],
                              self.flight_trail[i+1][0], self.flight_trail[i+1][1],
                              fill="#00E5FF", width=2)

        # Конус обзора камеры дрона
        if self.altitude > 5:
            cone_r = int(self.altitude * 1.2)
            c.create_oval(self.drone_x - cone_r, self.drone_y - cone_r,
                          self.drone_x + cone_r, self.drone_y + cone_r,
                          outline="#00E5FF", width=1, dash=(2, 4))

        # Иконка дрона (Квадрокоптер)
        dx, dy = self.drone_x, self.drone_y
        arm_len = 14
        c.create_line(dx - arm_len, dy - arm_len, dx + arm_len, dy + arm_len, fill="#ECEFF1", width=3)
        c.create_line(dx - arm_len, dy + arm_len, dx + arm_len, dy - arm_len, fill="#ECEFF1", width=3)
        # Пропеллеры
        for px, py in [(dx-arm_len, dy-arm_len), (dx+arm_len, dy-arm_len), (dx-arm_len, dy+arm_len), (dx+arm_len, dy+arm_len)]:
            c.create_oval(px-5, py-5, px+5, py-5, fill="#263238", outline="#00E5FF", width=1)
        c.create_oval(dx-6, dy-6, dx+6, dy+6, fill="#FF6B35", outline="#FFFFFF", width=2)

    def draw_camera(self):
        # Создаем виртуальный кадр камеры БПЛА через OpenCV
        h, w = 380, 540
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        # Фоновый лес
        frame[:] = (25, 45, 20)  # Лесная крона (BGR)

        # Рендерим очаг пожара на кадре
        # Если дрон приближается к цели
        dist = math.hypot(self.drone_x - self.fire_x, self.drone_y - self.fire_y)
        zoom = max(0.1, 1.0 - (dist / 400.0))

        if zoom > 0.2:
            cx, cy = int(w * 0.52), int(h * 0.48)
            fire_radius = int(55 * zoom) + int(math.sin(self.frame_counter * 0.4) * 5)
            
            # Дым
            cv2.circle(frame, (cx + 10, cy - 25), int(fire_radius * 1.5), (75, 75, 70), -1)
            # Внешнее пламя
            cv2.circle(frame, (cx, cy), fire_radius, (0, 110, 255), -1)
            # Горячее ядро
            cv2.circle(frame, (cx - 5, cy + 5), int(fire_radius * 0.5), (50, 230, 255), -1)

        # Режим тепловизора
        if self.thermal_mode:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)

        # ИИ-Детекция (YOLO / OpenCV)
        if zoom > 0.35 and self.altitude > 10:
            bx1, by1 = int(w * 0.36), int(h * 0.28)
            bx2, by2 = int(w * 0.68), int(h * 0.72)

            color = (0, 0, 255) if not self.thermal_mode else (0, 255, 255)
            # Рамка детектора
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), color, 2)
            
            # Угловые скобки видоискателя
            corner_len = 15
            for px, py in [(bx1, by1), (bx2, by1), (bx1, by2), (bx2, by2)]:
                sx = 1 if px == bx1 else -1
                sy = 1 if py == by1 else -1
                cv2.line(frame, (px, py), (px + sx * corner_len, py), color, 3)
                cv2.line(frame, (px, py), (px, py + sy * corner_len), color, 3)

            # Плашка с метаданными
            label = "FIRE DETECTED | 97.4%"
            cv2.rectangle(frame, (bx1, by1 - 25), (bx1 + 220, by1), color, -1)
            cv2.putText(frame, label, (bx1 + 5, by1 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            # Телеметрия инцидента прямо на камере
            cv2.putText(frame, "LAT: 53.4129 N  LON: 58.9814 E", (bx1, by2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.putText(frame, "SQUARE: ~1.4 Ha  TEMP: +480 C", (bx1, by2 + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # HUD камеры (Перекрестие)
        cv2.line(frame, (int(w/2) - 20, int(h/2)), (int(w/2) + 20, int(h/2)), (0, 255, 200), 1)
        cv2.line(frame, (int(w/2), int(h/2) - 20), (int(w/2), int(h/2) + 20), (0, 255, 200), 1)
        mode_str = "FLIR THERMAL 8-14um" if self.thermal_mode else "RGB OPTICAL 4K"
        cv2.putText(frame, f"MODE: {mode_str} | FPS: 59.8", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1)

        # Конвертация в Image для Tkinter
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(rgb)
        imgtk = ImageTk.PhotoImage(image=im)
        self.cam_label.imgtk = imgtk
        self.cam_label.configure(image=imgtk)

if __name__ == "__main__":
    app = OkoMchsApp()
    app.mainloop()
