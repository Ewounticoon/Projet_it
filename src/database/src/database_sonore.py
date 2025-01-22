#!/usr/bin/env python3

#code permettant de lire le topic "topic_micro", récupère la valeur du volume sonore, et écrit l'info dans une BBD sous la forme :
# ID (int) | Heure (datatime) |  Valeur (float32)

import sqlite3
import os
import rospy  # ROS Python
from std_msgs.msg import Float32  # Message ROS pour le son (type Float32)
from datetime import datetime

db_path=os.path.expanduser('~/ros_workspace/src/database/src/volumeMicro.db') #chemin d'acces
# Création de la base de données
def create_database():
    
    conn = sqlite3.connect(db_path) 
    cursor = conn.cursor()
    try :
        cursor.execute('''
        	CREATE TABLE IF NOT EXISTS son (
            	id INTEGER PRIMARY KEY AUTOINCREMENT,
            	date_time TEXT NOT NULL,
                nivSon REAL NOT NULL
                )
        ''')
        rospy.loginfo("Creation de la table pour le volume sonore") #DEBUG
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de la creation : {e}")
    finally:
        conn.commit()
        conn.close()

# Insertion des mesures dans la base de données
def insert_measurement(volSon):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute('''
        INSERT INTO son (date_time, volSon)
        VALUES (?, ?)
    ''', (datetime.now().strftime('%Y-%m-%d %H:%M:%S'), volSon))
    rospy.loginfo(f"Ecriture dans la table (son) : {volSon} %") #DEBUG
    conn.commit()
    conn.close()

# Callback pour traiter les messages du topic
def sound_callback(msg):
    volSon = msg.data  # La température est stockée dans msg.data
    rospy.loginfo(f"volume sonore reçu : {volSon} %")
    insert_measurement(volSon)  # Enregistrer dans la base de données

# Nœud ROS pour écouter le topic et enregistrer les données
def sound_listener():
    # Initialisation du nœud ROS
    rospy.init_node('sound_listener', anonymous=True)
    
    # S'abonner au topic "topic_micro" pour récupérer les données du micro
    rospy.Subscriber('topic_micro', Float32, sound_callback)
    
    # Créer la base de données si elle n'existe pas
    create_database()
    
    rospy.loginfo("Écoute du topic 'topic_micro'. Enregistrement du volume sonore dans la base de données.")
    
    # Maintenir le nœud actif
    rospy.spin()

# Programme principal
if __name__ == '__main__':
    try:
        sound_listener()
    except rospy.ROSInterruptException:
        pass

