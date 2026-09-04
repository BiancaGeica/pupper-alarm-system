# Pupper Firefighter - Alarm & Telemetry System

Acest modul acționează ca un dispecerat ROS 2 izolat pentru robotul Pupper. Ascultă mesajele (telemetrie, senzori, recunoaștere umană) generate de robot și le transmite în timp real către echipă via Telegram.

## Arhitectura Sistemului
* **`alarm_bot_node.py`**: Nodul principal care rutează alertele către Telegram.
* **`sensors_bridge.py`**: Procesează și standardizează datele de la senzorii de gaz și temperatură.
* **`thermal_map_node.py`**: Gestionează datele spațiale (hărți PGM) și vizualizarea zonelor toxice.

## Rulare (Containerizată)
Sistemul este complet containerizat prin Docker pentru a garanta izolarea față de simularea fizică (Gazebo). 
Pentru a porni nodul:
1. Deschide terminalul 1
2. Rulează: `./run.sh`
