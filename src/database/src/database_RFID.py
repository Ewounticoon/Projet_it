#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32 
import sqlite3
import os
from datetime import datetime

# Chemins des bases de données
db_path_mesures = os.path.expanduser('~/ros_workspace/src/database/database/RFID_mesures.db')
db_path_infos = os.path.expanduser('~/ros_workspace/src/database/database/RFID_infos.db')

# Création de la base de données des mesures
def create_database_mesure():
    """Création de la table pour les mesures RFID."""
    conn = sqlite3.connect(db_path_mesures)
    cursor = conn.cursor()
    try:
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS mesures (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                date_time TEXT NOT NULL,
                numBadge INT32 NOT NULL,
                register TEXT NOT NULL
            )
        ''')
        rospy.loginfo("Création de la table pour les relevés du badge")  # DEBUG
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de la création de la table : {e}")
    finally:
        conn.commit()
        conn.close()

# Vérification si un badge est présent dans la base de données infos
def check_badge_in_infos(num_badge):
    """Vérifie si un numéro de badge est dans la base de données infos."""
    conn = sqlite3.connect(db_path_infos)
    cursor = conn.cursor()
    try:
        cursor.execute('SELECT * FROM infos WHERE numBadge = ?', (num_badge,))
        result = cursor.fetchone()
        return result is not None
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de la vérification du badge : {e}")
        return False
    finally:
        conn.close()

# Fonction de callback pour les messages du topic
def rfid_callback(msg):
    """Callback appelée à la réception d'un message sur le topic."""
    num_badge = msg.data  # Supposant que msg.data contient le numéro du badge
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    # Vérifier si le badge est connu
    is_known = check_badge_in_infos(num_badge)
    register_value = "YES" if is_known else "NO"

    # Enregistrement dans la base de données des mesures
    conn = sqlite3.connect(db_path_mesures)
    cursor = conn.cursor()
    try:
        cursor.execute('''
            INSERT INTO mesures (date_time, numBadge, register) 
            VALUES (?, ?, ?)
        ''', (timestamp, num_badge, register_value))
        rospy.loginfo(f"Badge {num_badge} lu à {timestamp}, statut d'enregistrement : {register_value} ")  # DEBUG
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de l'insertion des données : {e}")
    finally:
        conn.commit()
        conn.close()

# Initialisation du noeud ROS
def listener():
    """Initialise le noeud ROS et écoute le topic."""
    rospy.init_node('rfid_listener', anonymous=True)

    # Créer la base de données si elle n'existe pas
    create_database_mesure()

    # Souscrire au topic
    rospy.Subscriber("topic_rfid", Int32, rfid_callback)  # Remplacez Int32 si nécessaire

    # Boucle ROS
    rospy.loginfo("Noeud rfid_listener démarré et en attente de messages...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noeud rfid_listener arrêté.")
