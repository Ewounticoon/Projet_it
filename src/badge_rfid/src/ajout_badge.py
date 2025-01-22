#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32 
import sqlite3
import os
from datetime import datetime
from badge_rfid.srv import ajout_badge, ajout_badgeResponse



# Chemin de la base de données
db_path = os.path.expanduser('~/ros_workspace/src/database/src/RFID_infos.db')

# Création de la base de données
def create_database_mesure():
    """Création de la table pour les mesures RFID."""
    conn = sqlite3.connect(db_path)
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


def lecture_badge(msg):
    num_badge = msg.data
    return num_badge

def ajout_badge_base(req):
    rospy.loginfo(f"Received user info: {req.prenom} {req.nom}, Age: {req.age}, Email: {req.email}, Job: {req.poste}")
    success = True

    num_badge=rospy.Subscriber("topic_rfid", Int32, lecture_badge)  # Remplacez Int32 si nécessaire

    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    register_value = "NO"

    # Enregistrement dans la base de données
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    try:
        cursor.execute('''
            INSERT INTO mesures (date_time, numBadge, register) 
            VALUES (?, ?, ?)
        ''', (timestamp, num_badge, register_value))
        rospy.loginfo(f"Badge {num_badge} enregistré à {timestamp}")  # DEBUG
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de l'insertion des données : {e}")
    finally:
        conn.commit()
        conn.close()


    return ajout_badgeResponse(success)




# Initialisation du noeud ROS
def listener():
    """Initialise le noeud ROS et écoute le topic."""
    rospy.init_node('ajout_badge', anonymous=True)

    # Créer la base de données si elle n'existe pas
    create_database_mesure()

    # Souscrire au topic
    # rospy.Subscriber("topic_rfid", Int32, rfid_callback)  # Remplacez Int32 si nécessaire
    s = rospy.Service('ajout_badge', ajout_badge, ajout_badge_base)


    # Boucle ROS
    rospy.loginfo("Noeud rfid_listener démarré et en attente de messages...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noeud rfid_listener arrêté.")
