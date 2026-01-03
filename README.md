# ros_geofence

ROS 2 пакет для перевірки знаходження робота в межах визначеної географічної зони (geofence).

## Опис

Пакет дозволяє:
- Завантажувати зону з GeoJSON файлу
- Перевіряти чи робот знаходиться в межах зони на основі GPS координат
- Публікувати межі зони для візуалізації в Mapviz
- Генерувати costmap для Nav2

## Залежності

- ROS 2 (Humble/Iron/Jazzy)
- nav2_msgs
- mapviz, mapviz_plugins
- swri_transform_util
- PROJ (бібліотека для картографічних проекцій)
- nlohmann_json

## Встановлення

```bash
cd ~/ros2_ws/src
git clone https://github.com/Fastroof/ros_geofence.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros_geofence
source install/setup.bash
```

## Налаштування MapTiler API

Для відображення супутникових тайлів потрібен API ключ від [MapTiler](https://www.maptiler.com/).

Встановіть змінну середовища:

```bash
# Додайте в ~/.bashrc для постійного використання
export MAPTILER_API_KEY="your_api_key_here"
```

Або передайте ключ як параметр при запуску:

```bash
ros2 launch ros_geofence bringup.launch.py maptiler_api_key:=your_api_key_here
```

## Структура пакету

```
ros_geofence/
├── config/
│   ├── mapviz.mvc       # Конфігурація Mapviz
│   └── zone.geojson     # Файл зони
├── include/ros_geofence/
│   ├── zone_checker.hpp # Заголовок ZoneChecker
│   └── fake_robot.hpp   # Заголовок FakeRobot
├── launch/
│   ├── bringup.launch.py       # Запуск всіх компонентів
│   ├── zone_checker.launch.py  # Запуск zone_checker
│   ├── fake_robot.launch.py    # Запуск fake_robot
│   └── mapviz.launch.py        # Запуск Mapviz
└── src/
    ├── zone_checker.cpp # Нода перевірки зони
    └── fake_robot.cpp   # Тестова нода робота
```

## Використання

### Запуск всіх компонентів

```bash
ros2 launch ros_geofence bringup.launch.py
```

### Параметри bringup.launch.py

| Параметр | За замовчуванням | Опис |
|----------|------------------|------|
| use_fake_robot | true | Запускати fake_robot |
| use_zone_checker | true | Запускати zone_checker |
| use_mapviz | true | Запускати Mapviz |
| latitude | 47.254 | Широта робота |
| longitude | 30.44 | Довгота робота |
| geojson_file | config/zone.geojson | Шлях до GeoJSON файлу |

### Приклади запуску

```bash
# Запуск з власними координатами
ros2 launch ros_geofence bringup.launch.py latitude:=47.25 longitude:=30.44

# Без Mapviz (тільки перевірка зони)
ros2 launch ros_geofence bringup.launch.py use_mapviz:=false

# Тільки zone_checker (для реального робота)
ros2 launch ros_geofence zone_checker.launch.py

# Тільки fake_robot
ros2 launch ros_geofence fake_robot.launch.py latitude:=47.25 longitude:=30.44
```

## Ноди

### zone_checker

Перевіряє чи робот знаходиться в межах зони.

**Підписки:**
- `/gps/fix` (sensor_msgs/NavSatFix) - GPS позиція робота

**Публікації:**
- `/robot_in_zone` (std_msgs/Bool) - чи робот в зоні
- `/geofence_marker` (visualization_msgs/Marker) - маркер для Mapviz
- `/geofence_zone` (geometry_msgs/PolygonStamped) - полігон зони
- `/global_costmap/costmap` (nav2_msgs/Costmap) - costmap для Nav2

**Параметри:**
- `geojson_file` (string) - шлях до GeoJSON файлу
- `costmap_resolution` (double) - роздільна здатність costmap в метрах
- `costmap_topic` (string) - топік для публікації costmap
- `geofence_zone_topic` (string) - топік для публікації полігону зони

### fake_robot

Тестова нода що імітує робота з GPS.

**Публікації:**
- `/gps/fix` (sensor_msgs/NavSatFix) - GPS позиція
- TF: map -> base_link

**Параметри:**
- `latitude` (double) - широта
- `longitude` (double) - довгота
- `altitude` (double) - висота
- `publish_rate` (double) - частота публікації в Hz

## Формат GeoJSON

Приклад файлу zone.geojson:

```json
{
  "type": "FeatureCollection",
  "features": [{
    "type": "Feature",
    "properties": {},
    "geometry": {
      "type": "Polygon",
      "coordinates": [[
        [30.4309, 47.2594],
        [30.4356, 47.2461],
        [30.4502, 47.2484],
        [30.4454, 47.2624],
        [30.4309, 47.2594]
      ]]
    }
  }]
}
```

## Ліцензія

Apache-2.0
