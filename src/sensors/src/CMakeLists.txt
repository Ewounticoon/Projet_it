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
  dht11_sensor_node.py  # Chemin vers votre script Python
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Dépendances Python externes
find_package(PythonLibs REQUIRED)
