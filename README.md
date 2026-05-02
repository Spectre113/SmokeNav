# SmokeNav (ROS 2 Humble + Gazebo Classic)

Проект симуляции мобильного робота, который едет к цели (человек в Gazebo), избегает препятствия и "поглощает" цель при достижении.

## 1. Структура

- `src/project_sim` - мир, спавн робота, full-stack launch
- `src/project_nav` - навигация и обход препятствий
- `src/project_detection` - детекция цели в Gazebo + маркер цели
- `src/human_localization` - трекинг/адаптация цели в `/target_info`
- `src/project_smoke` - фильтрация лидара по дыму

## 2. Требования

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (`gazebo_ros`)
- `colcon`

Пример базовой установки:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  python3-colcon-common-extensions
```

## 3. Первичная подготовка

```bash
cd ~/AMR_ws/src/SmokeNav-test
bash setup.sh
source ~/ros2_venv/bin/activate
```

## 4. Сборка

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Запуск full stack

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project_sim sim_full_stack.launch.py
```

Запускается:
- Gazebo world + robot spawn
- smoke filter
- detector + marker цели
- localization + adapter
- sector analyzer + goal-aware navigation

## 6. Важное поведение

- Робот едет к цели по `/target_info`.
- При близком подходе цель скрывается для навигации (чтобы робот не крутился вокруг).
- При достижении порога `consume_distance` цель удаляется из Gazebo:
  - удаляется `human_0`
  - удаляется `goal_target_marker`
  - цель больше не появляется в текущем запуске.

## 7. Где менять позицию человека

Файл мира:

- `src/project_sim/worlds/custom-flat.world`

Ищите:

```xml
<model name="human_0">
  <pose>1.5 0.0 0.0 0 0 0</pose>
```

Формат `pose`: `x y z roll pitch yaw`.

После изменения:

```bash
cd ~/AMR_ws/src/SmokeNav-test
source /opt/ros/humble/setup.bash
colcon build --packages-select project_sim --symlink-install
source install/setup.bash
ros2 launch project_sim sim_full_stack.launch.py
```

## 8. Полезная диагностика

```bash
ros2 topic echo /target_info --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /gazebo/model_states --once
```

## 9. Если Gazebo "not responding" / `gzserver exit code 255`

Обычно причина: завис старый процесс Gazebo (порт master уже занят).

```bash
pkill -f gzclient
pkill -f gzserver
pkill -f 'spawn_entity.py'
pkill -f 'sim_full_stack.launch.py'

source /opt/ros/humble/setup.bash
ros2 daemon stop
ros2 daemon start
```

И потом запуск заново (см. раздел 5).

## 10. Текущие ключевые launch-параметры цели

В `src/project_detection/launch/detection_from_gazebo.launch.py`:

- `consume_on_reach: true`
- `consume_distance: 0.75`
- `target_entity_name: "human_0"`

Если хотите более раннее исчезновение цели, уменьшайте `consume_distance`.
