cmake_minimum_required(VERSION 3.0.2)
project(sensors)

# Trouver les packages nécessaires
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# Installer les scripts Python
catkin_install_python(PROGRAMS
  src/dht11_sensor_node.py  # Chemin vers votre script Python
  src/micro_sensor_node.py
  src/badge_sensor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Dépendances Python externes
find_package(PythonLibs REQUIRED)
