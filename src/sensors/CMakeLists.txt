cmake_minimum_required(VERSION 3.0.2)
project(sensors)

# Trouver les packages nécessaires
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation  # Pour générer les messages personnalisés
)

# Déclarer les fichiers de message
add_message_files(
  FILES
  dht11.msg  # Pas besoin de "msg/" ici, juste le nom du fichier
)

# Générer les messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Déclarer le package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# Installer les scripts Python
catkin_install_python(PROGRAMS
  src/dht11_sensor_node.py  # Chemin vers votre script Python
  src/micro_sensor_node.py
  src/badge_sensor_node.py
  src/test_node/test_pub_badge.py
  src/test_node/test_pub_dht11.py
  src/test_node/test_pub_micro.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Dépendances Python externes (optionnel, si nécessaire pour votre projet)
find_package(PythonLibs REQUIRED)

