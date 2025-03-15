# Industrial IT Project

## Description
The **Industrial IT** project is a monitoring and access control system for a company using RFID badges and environmental sensors. Its goal is to ensure employee safety by monitoring ambient conditions during their passage.

The system relies on multiple sensors:
- **RFID Badge**: Identifies employees entering and leaving.
- **Humidity Sensor**: Measures ambient humidity levels.
- **Temperature Sensor**: Monitors indoor temperature.
- **Sound Sensor**: Detects noise levels to ensure a safe environment.

An interactive web interface allows users to:
- Display real-time graphs of temperature, humidity, and noise levels.
- Manage badges (add/remove).
- Consult the log of entries and exits with timestamps.

## Technologies Used
- **Main Language**: Python under **ROS Noetic**.
- **Database**: SQLite3.
- **Web Interface**: Flask, HTML, CSS, JavaScript.

## Key Features
1. **RFID Badge Identification**:
   - Reads badges to authenticate employees.
   - Records entries and exits in the database.

2. **Environmental Condition Monitoring**:
   - Logs and displays temperature, humidity, and noise level values over time.
   
3. **Web Interface**:
   - Displays sensor data in graphical form.
   - Manages badges (add/remove).
   - Provides access to entry and exit history.

## Installation and Execution
### Prerequisites
- **Docker** 

### Installation
1. **Clone the project repository**:
   ```bash
   git clone https://github.com/Ewounticoon/Projet_it.git
   cd Projet_it
   ```
2. **Build the docker**:
   ```bash
   cd docker
   chmod +x entrypoint.sh
   docker-compose up -d --build
   docker exec -it ros_noetic_projetIT bash
   ```
   **To run or close the container**:
   ```bash
   docker start ros_noetic_projetIT
   docker exec -it ros_noetic_projetIT bash
   docker stop ros_noetic_projetIT
   ```

3. **Start the system**:
   ```bash
   cd ~/Projet_it
   catkin_make
   source devel/setup.bash
   roslaunch industrial_it system.launch
   ```

4. **Access the Web Interface**:
   Open a browser and go to:
   ```
   https://localhost:5000
   ```

## Authors
This project was developed by **GAUDET Pierre**, **BOISSENIN Enzo** and **GIRAUD-CARRIER Ewen** as part of the **Industrial IT** project.

---

# Projet Industrial IT

## Description
Le projet **Industrial IT** est un système de surveillance et de gestion des entrées dans une entreprise à l'aide de badges RFID et de capteurs environnementaux. Son objectif est d'assurer la sécurité des employés en surveillant les conditions ambiantes lors de leur passage.

Le système repose sur plusieurs capteurs :
- **Badge RFID** : Identification des employés entrant et sortant.
- **Capteur d'humidité** : Mesure du taux d'humidité ambiant.
- **Capteur de température** : Surveillance de la température intérieure.
- **Capteur sonore** : Détection du niveau sonore pour assurer un environnement sûr.

Une interface Web interactive permet :
- D'afficher les graphiques en temps réel de la température, de l'humidité et du niveau sonore.
- De gérer les badges (ajout/suppression).
- De consulter l'historique des entrées et sorties avec les horaires correspondants.

## Technologies utilisées
- **Langage principal** : Python sous **ROS Noetic**.
- **Base de données** : SQLite3.
- **Interface Web** : Flask, HTML, CSS, JavaScript.

## Fonctionnalités principales
1. **Identification par badge RFID** :
   - Lecture des badges pour authentifier un employé.
   - Enregistrement des entrées et sorties dans la base de données.

2. **Surveillance des conditions environnementales** :
   - Enregistrement et affichage des valeurs de température, humidité et niveau sonore en fonction du temps.
   
3. **Interface Web** :
   - Visualisation des mesures sous forme de graphiques.
   - Gestion des badges (ajout/suppression).
   - Consultation des historiques d'entrée et de sortie.

## Installation et exécution
### Prérequis
- **Docker** (recommandé pour éviter les problèmes de compatibilité avec ROS Noetic sur Raspberry Pi)

### Installation
1. **Cloner le dépôt du projet**:
   ```bash
   git clone https://github.com/Ewounticoon/Projet_it.git
   cd Projet_it
   ```
2. **Construire le Docker**:
   ```bash
   cd docker
   chmod +x entrypoint.sh
   docker-compose up -d --build
   docker exec -it ros_noetic_projetIT bash
   ```
   **Pour démarrer ou arrêter le conteneur**:
   ```bash
   docker start ros_noetic_projetIT
   docker exec -it ros_noetic_projetIT bash
   docker stop ros_noetic_projetIT
   ```

3. **Démarrer le système**:
   ```bash
   cd ~/Projet_it
   catkin_make
   source devel/setup.bash
   roslaunch industrial_it system.launch
   ```

4. **Accéder à l'interface web**:
   Ouvrez un navigateur et allez à l'adresse suivante :
   ```
   https://localhost:5000
   ```

## Auteurs
Ce projet a été réalisé par **GAUDET Pierre**, **BOISSENIN Enzo** et **GIRAUD-CARRIER Ewen** dans le cadre du projet **Industrial IT**.
