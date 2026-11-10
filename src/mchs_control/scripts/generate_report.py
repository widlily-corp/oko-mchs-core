#!/usr/bin/env python3
"""
Генератор оперативного протокола инцидента МЧС.
Сохраняет структурированный отчет при фиксации ЧС.
"""
import json
import time
from datetime import datetime

def generate_incident_report(
    incident_type="Лесной пожар (Низовой)",
    coordinates={"lat": 55.751244, "lon": 37.618423, "alt_m": 85.0},
    danger_level="ВЫСОКИЙ (Угроза верхового перехода)",
    estimated_area_m2=1450.0,
    drone_id="MCHS-UAV-04"
):
    now = datetime.now()
    report = {
        "mchs_incident_report": {
            "protocol_version": "1.0",
            "timestamp": now.isoformat(),
            "readable_time": now.strftime("%d.%m.%Y %H:%M:%S"),
            "dispatcher_unit": "ЦУКС Главного управления МЧС России",
            "uav_telemetry": {
                "drone_id": drone_id,
                "altitude_agl_m": coordinates["alt_m"],
                "target_coordinates": {
                    "latitude": coordinates["lat"],
                    "longitude": coordinates["lon"]
                }
            },
            "hazard_classification": {
                "incident_type": incident_type,
                "danger_level": danger_level,
                "estimated_area_sq_meters": estimated_area_m2,
                "containment_priority": "Срочное реагирование (Ранг 1-БИС)"
            },
            "recommended_actions": [
                "Направить оперативно-разведывательную группу на квадроциклах",
                "Привести в готовность автоцистерну АЦ-3,0-40",
                "Оповестить лесопожарную службу района"
            ]
        }
    }
    
    filename = f"mchs_report_{now.strftime('%Y%m%d_%H%M%S')}.json"
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
        
    print(f"[OK] Протокол МЧС успешно сформирован: {filename}")
    return filename

if __name__ == "__main__":
    generate_incident_report()
