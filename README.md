# Robotica-2026-2

Repositorio para los archivos de la práctica 1 de la materia Robótica.

## Integrantes
- De Luna Santillan Jose de Jesus
- Mejia Huertas Rodrigo Alfredo
- Valdez Hernandez Brayan Julio

## Instrucciones del work space

rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding example_interfaces
source install/setup.bash
ros2 launch example_bringup rviz_bringup.launch.py

En otra terminal 
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch example_bringup controler.launch.py

rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding example_interfaces
source install/setup.bash
ros2 launch example_bringup rviz_bringup.launch.py

En otra terminal 
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch example_bringup controler.launch.py
