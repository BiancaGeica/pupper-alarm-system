# Pupper Firefighter - Alarm & Telemetry System

Acest modul actioneaza ca un dispecerat ROS 2 izolat. Asculta mesajele si hartile generate de robot in reteaua locala si le transmite in timp real catre echipa, prin Telegram.

## Cum se porneste (Terminalul 1)
Sistemul este complet containerizat. Pentru a porni nodul, deschide un terminal si ruleaza scriptul automat:
```bash
./run.sh