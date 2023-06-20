# РАСПОЗНАВАНИЕ ЭЛЕМЕНТОВ ДОРОЖНОЙ ИНФРАСТРУКТУРЫ В РАМКАХ ПЛАТФОРМЫ ROS 2


Для успешной работы проекта вам понадобятся следующие зависимости:

- Ubuntu 22.04
- ROS 2 Humble (установленный и настроенный)
- Turtlebot Gazebo Package
- Colcon

## Сборка рабочего пространства

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/ros/ros_tutorials.git -b humble
   rosdep install -i --from-path src --rosdistro humble -y
   colcon build
   ```
  
## Сборка проекта
   ```bash
   cd 
   cd ~/ros2_ws/src
   git clone https://github.com/MaksTue/video_recognition.git
   colcon build video_recognition
   ```
 ## Запуск симуляции
   ```bash
   cd
   gazebo ros2_ws/src/video_recognition/world/test_world.world
   ```
  ## Запуск узлов
  Запуск узла обнаружения
   ```bash
   cd
   cd ~/ros2_ws
   ros2 run video_recognition detection
   ```
   Запуск узла классификации
   ```bash
   cd
   cd ~/ros2_ws
   ros2 run video_recognition recognition
   ```
   Запуск узла управления скоростью
   ```bash
   cd
   cd ~/ros2_ws
   ros2 run video_recognition speed_control
   ```
   ## Запуск узлов
   Для визуализации видеоптока из топика можно использовать
   ```bash
   cd
   cd ~/ros2_ws
   ros2 run rqt_image_view rqt_image_view
   ```
