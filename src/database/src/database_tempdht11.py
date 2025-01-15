#!/usr/bin/env python3

#code permettant de lire le topic "tempDHT11", récupère la valeur de la température, et écrit l'info dans une BBD sous la forme :
# ID (int) | Heure (datatime) |  Valeur (float32)

import sqlite3
import rospy  # ROS Python
from std_msgs.msg import Float32  # Message ROS pour la température (type Float32)
from datetime import datetime

# Création de la base de données
def create_database():
    conn = sqlite3.connect('./dht11_temperature.db')  # Nom de la base de données
    cursor = conn.cursor()
    try :
    	cursor.execute('''
        	CREATE TABLE IF NOT EXISTS temperature (
            	id INTEGER PRIMARY KEY AUTOINCREMENT,
            	date_time TEXT NOT NULL,
            	temperature REAL NOT NULL
                )
        ''')
    	rospy.loginfo("Creation de la table") #DEBUG
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de la creation : {e}")
    finally:
        conn.commit()
        conn.close()

# Insertion des mesures dans la base de données
def insert_measurement(temperature):
    conn = sqlite3.connect('dht11_temperature.db')
    cursor = conn.cursor()
    cursor.execute('''
        INSERT INTO temperature (date_time, temperature)
        VALUES (?, ?)
    ''', (datetime.now().strftime('%Y-%m-%d %H:%M:%S'), temperature))
    rospy.loginfo(f"Ecriture dans la table : {temperature}") #DEBUG
    conn.commit()
    conn.close()

# Callback pour traiter les messages du topic
def temperature_callback(msg):
    temperature = msg.data  # La température est stockée dans msg.data
    rospy.loginfo(f"Température reçue : {temperature}°C")
    insert_measurement(temperature)  # Enregistrer dans la base de données

# Nœud ROS pour écouter le topic et enregistrer les données
def temperature_listener():
    # Initialisation du nœud ROS
    rospy.init_node('temperature_listener', anonymous=True)
    
    # S'abonner au topic "topic_tempDHT11" pour récupérer les données de température /!\ Penser a modif en cas de chgnt de nom
    rospy.Subscriber('topic_tempDHT11', Float32, temperature_callback)
    
    # Créer la base de données si elle n'existe pas
    create_database()
    
    rospy.loginfo("Écoute du topic 'topic_tempDHT11'. Enregistrement des températures dans la base de données.")
    
    # Maintenir le nœud actif
    rospy.spin()

# Programme principal
if __name__ == '__main__':
    try:
        temperature_listener()
    except rospy.ROSInterruptException:
        pass
