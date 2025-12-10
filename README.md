# Robotica-2026-2

Repositorio para los archivos de la práctica 1 de la materia Robótica.

## Integrantes
- De Luna Santillan Jose de Jesus
- Mejia Huertas Rodrigo Alfredo
- Valdez Hernandez Brayan Julio

## Instrucciones del work space

1. rm -rf build install log
2. source /opt/ros/humble/setup.bash
3. colcon build --symlink-install --allow-overriding example_interfaces
4. source install/setup.bash
5. 5.ros2 launch example_bringup rviz_bringup.launch.py

En otra terminal 
1. source /opt/ros/humble/setup.bash
2. source install/setup.bash
3. ros2 launch example_bringup controler.launch.py
   
 Segundo robot
 
1. rm -rf build install log
2. source /opt/ros/humble/setup.bash
3. colcon build --symlink-install --allow-overriding example_interfaces
4. source install/setup.bash
5. ros2 launch example_bringup rviz_bringup.launch.py

En otra terminal 
1. source /opt/ros/humble/setup.bash
2. source install/setup.bash
3. ros2 launch example_bringup controler.launch.py
