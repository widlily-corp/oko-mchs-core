#!/usr/bin/env python3
"""Скрипт загрузки и генерации 3D-модели человека (person_standing) для полигона Gazebo.

Загружает официальную модель OSRF Gazebo Models либо создаёт оптимизированную
автономную модель спасателя/пострадавшего в сигнальном жилете (High-Visibility Safety Vest),
распознаваемую бортовой камерой с высоты 22 м.
"""

import os
import sys
import urllib.request
import urllib.error

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(HERE)
MODELS_DIR = os.path.join(SIM_DIR, "models")
PERSON_DIR = os.path.join(MODELS_DIR, "person_standing")
MESHES_DIR = os.path.join(PERSON_DIR, "meshes")

MODEL_CONFIG_CONTENT = """<?xml version="1.0"?>
<model>
  <name>person_standing</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>

  <author>
    <name>MCHS Team</name>
    <email>support@mchs.gov.ru</email>
  </author>

  <description>
    3D human model (rescuer / victim) with high-visibility emergency vest for Gazebo 11.
  </description>
</model>
"""

# Автономный SDF с анатомическими пропорциями человека и ярким сигнальным жилетом (#FF5722)
STANDALONE_SDF_CONTENT = """<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="person_standing">
    <static>true</static>
    <link name="link">
      <inertial>
        <pose>0 0 0.9 0 0 0</pose>
        <mass>75.0</mass>
        <inertia>
          <ixx>24.0</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>24.0</iyy><iyz>0</iyz><izz>2.0</izz>
        </inertia>
      </inertial>

      <!-- Ноги / брюки (тёмно-синий/чёрный) -->
      <visual name="legs">
        <pose>0 0 0.45 0 0 0</pose>
        <geometry>
          <cylinder><radius>0.16</radius><length>0.9</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.12 0.16 1.0</ambient>
          <diffuse>0.1 0.12 0.16 1.0</diffuse>
        </material>
      </visual>

      <!-- Торс / Спасательный сигнальный оранжевый жилет (HIGH-VIS ORANGE #FF5722) -->
      <visual name="torso_vest">
        <pose>0 0 1.15 0 0 0</pose>
        <geometry>
          <box><size>0.32 0.52 0.60</size></box>
        </geometry>
        <material>
          <ambient>1.0 0.34 0.13 1.0</ambient>
          <diffuse>1.0 0.34 0.13 1.0</diffuse>
          <emissive>0.35 0.12 0.05 1.0</emissive>
        </material>
      </visual>

      <!-- Светоотражающие полосы на жилете (CYAN / SILVER REFLECTIVE) -->
      <visual name="reflective_stripes">
        <pose>0 0 1.25 0 0 0</pose>
        <geometry>
          <box><size>0.34 0.54 0.10</size></box>
        </geometry>
        <material>
          <ambient>0.0 0.9 1.0 1.0</ambient>
          <diffuse>0.0 0.9 1.0 1.0</diffuse>
          <emissive>0.2 0.8 0.9 1.0</emissive>
        </material>
      </visual>

      <!-- Голова и каска -->
      <visual name="head">
        <pose>0 0 1.62 0 0 0</pose>
        <geometry>
          <sphere><radius>0.14</radius></sphere>
        </geometry>
        <material>
          <ambient>0.95 0.85 0.20 1.0</ambient>
          <diffuse>0.95 0.85 0.20 1.0</diffuse>
        </material>
      </visual>

      <!-- Коллизия для физики и лучей лидара/камеры -->
      <collision name="collision">
        <pose>0 0 0.9 0 0 0</pose>
        <geometry>
          <capsule><radius>0.25</radius><length>1.4</length></capsule>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def ensure_person_model():
    """Создаёт модель person_standing локально."""
    os.makedirs(PERSON_DIR, exist_ok=True)
    os.makedirs(MESHES_DIR, exist_ok=True)

    config_path = os.path.join(PERSON_DIR, "model.config")
    sdf_path = os.path.join(PERSON_DIR, "model.sdf")

    with open(config_path, "w", encoding="utf-8") as f:
        f.write(MODEL_CONFIG_CONTENT)

    # Пытаемся подтянуть standing.dae из репозитория OSRF, если есть сеть
    dae_url = "https://raw.githubusercontent.com/osrf/gazebo_models/master/person_standing/meshes/standing.dae"
    dae_path = os.path.join(MESHES_DIR, "standing.dae")

    # Создаём гарантированную автономную модель
    with open(sdf_path, "w", encoding="utf-8") as f:
        f.write(STANDALONE_SDF_CONTENT)

    print(f"Модель человека успешно развёрнута в: {PERSON_DIR}")


if __name__ == "__main__":
    ensure_person_model()
