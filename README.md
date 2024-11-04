# RL24_HW_1_lisi
## Istruzioni 
1. Clona il repository in una cartella contente il ros2_docker_scripts:
   ```bash
   git clone https://github.com/GioGab78/rl24_hw1_Imbimbo.git
 
2. Con lo scopo di configurare e costruire tutti i pacchetti presenti nel workspace:
   ```bash
      colcon build
      source install/setup.bash
 
## Lancio dei pacchetti:
1. Per lanciare arm_description:
   ```bash
      ros2 launch arm_description display.launch.py
 
2. Per lanciare arm_gazebo senza controlli:
 
     ```bash
      ros2 launch arm_gazebo arm_world.launch.py
 
3. Per lanciare arm_control:
 
     ```bash
      ros2 launch arm_control control.launch.py

4. Per lanciare arm_gazebo con i controlli:
      ```bash
      ros2 launch arm_gazebo arm_gazebo.launch.py
 
 
5. Avviare il nodo ros_publisher:
      ```bash
      ros2 topic pub /position_controller/commands std_msgs/msg/   Float64MultiArray "{data: [0.5, -0.5, 0.3, -0.3]}"
