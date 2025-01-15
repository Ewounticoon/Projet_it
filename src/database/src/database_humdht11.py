#!/usr/bin/env python3

#code permettant de lire le topic "humDHT11", récupère la valeur de l'humidite, et écrit l'info dans une BBD sous la forme :
# ID (int) | Heure (datatime) |  Valeur (float32)

import sqlite3
import os
import rospy  # ROS Python
from std_msgs.msg import Float32  # Message ROS pour la température (type Float32)
from datetime import datetime

db_path=os.path.expanduser('~/ros_workspace/src/database/src/dht11_humidite.db') #chemin d'acces
# Création de la base de données
def create_database():
    
    conn = sqlite3.connect(db_path) 
    cursor = conn.cursor()
    try :
        cursor.execute('''
        	CREATE TABLE IF NOT EXISTS humidite (
            	id INTEGER PRIMARY KEY AUTOINCREMENT,
            	date_time TEXT NOT NULL,
                humidite REAL NOT NULL
                )
        ''')
        rospy.loginfo("Creation de la table") #DEBUG
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de la creation : {e}")
    finally:
        conn.commit()
        conn.close()

# Insertion des mesures dans la base de données
def insert_measurement(humidite):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute('''
        INSERT INTO humidite (date_time, humidite)
        VALUES (?, ?)
    ''', (datetime.now().strftime('%Y-%m-%d %H:%M:%S'), humidite))
    rospy.loginfo(f"Ecriture dans la table : {humidite}") #DEBUG
    conn.commit()
    conn.close()

# Callback pour traiter les messages du topic
def hum_callback(msg):
    humidite = msg.data  # La température est stockée dans msg.data
    rospy.loginfo(f"humidite reçue : {humidite} %")
    insert_measurement(humidite)  # Enregistrer dans la base de données

# Nœud ROS pour écouter le topic et enregistrer les données
def hum_listener():
    # Initialisation du nœud ROS
    rospy.init_node('humidite_listener', anonymous=True)
    
    # S'abonner au topic "topic_tempDHT11" pour récupérer les données de température /!\ Penser a modif en cas de chgnt de nom
    rospy.Subscriber('topic_humDHT11', Float32, hum_callback)
    
    # Créer la base de données si elle n'existe pas
    create_database()
    
    rospy.loginfo("Écoute du topic 'topic_humDHT11'. Enregistrement de l'humidite dans la base de données.")
    
    # Maintenir le nœud actif
    rospy.spin()

# Programme principal
if __name__ == '__main__':
    try:
        hum_listener()
    except rospy.ROSInterruptException:
        pass
