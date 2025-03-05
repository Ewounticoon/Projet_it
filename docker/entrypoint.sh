#!/bin/bash
set -e  # Stoppe le script en cas d'erreur


# FORCER L'INSTALLATION DE Adafruit_DHT
echo " Installation forcée de Adafruit_DHT..."
pip3 install --no-cache-dir --force-reinstall --no-build-isolation Adafruit_DHT --install-option="--force-pi"
# Garder le conteneur actif en boucle infinieecho " Adafruit_DHT installé avec succès."

chown -R 1000:1000 /root

# Source ROS Noetic
source /opt/ros/noetic/setup.bash

# Source le workspace ROS si `catkin_make` a été fait
if [ -f "/root/devel/setup.bash" ]; then
    source /root/devel/setup.bash
fi

# Lancer roscore en arrière-plan
roscore &
sleep 3  # Pause pour s'assurer que roscore démarre

# Garder le conteneur actif
tail -f /dev/null