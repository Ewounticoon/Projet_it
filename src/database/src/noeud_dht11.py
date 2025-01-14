#!/usr/bin/env python3


#Noeud ROS permettant de lire les donnees du capteur DHT11, de les enregistrer dans une table SQL, et de publier les info sur un topic "dht11"
#Attention, penser a installer toutes les librairies

#!/usr/bin/env python3

import sqlite3
from datetime import datetime
import rospy  # ROS Python
from std_msgs.msg import String  # Message standard ROS
import adafruit_dht
import board
import time

# Initialisation du capteur DHT11
dht_device = adafruit_dht.DHT11(board.D4)  # Remplacez D4 par le GPIO utilise

# Creation de la base de donnees (appelee une seule fois)
def create_database():
    conn = sqlite3.connect('sensors_data.db')  # Nom global pour plusieurs capteurs
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS dht11 (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            date_time TEXT NOT NULL,
            temperature REAL NOT NULL,
            humidity REAL NOT NULL
        )
    ''')
    conn.commit()
    conn.close()

# Insertion des mesures dans la base de donnees
def insert_measurement(temperature, humidity):
    conn = sqlite3.connect('sensors_data.db')
    cursor = conn.cursor()
    cursor.execute('''
        INSERT INTO dht11 (date_time, temperature, humidity)
        VALUES (?, ?, ?)
    ''', (datetime.now().strftime('%Y-%m-%d %H:%M:%S'), temperature, humidity))
    conn.commit()
    conn.close()

# Noeud ROS
def dht11_node():
    # Initialisation du noeud ROS
    rospy.init_node('dht11_node', anonymous=True)
    publisher = rospy.Publisher('/dht11', String, queue_size=10)
    rate = rospy.Rate(0.5)  # Frequence : 0.5 Hz (1 mesure toutes les 2 secondes)

    rospy.loginfo("DHT11 Node demarre. Publie sur le topic '/dht11'. Appuyez sur Ctrl+C pour arreter.")

    while not rospy.is_shutdown():
        try:
            # Lecture des mesures
            temperature = dht_device.temperature
            humidity = dht_device.humidity

            if temperature is not None and humidity is not None:
                # Publier les mesures
                message = f"Temperature: {temperature}°C, Humidity: {humidity}%"
                publisher.publish(message)
                rospy.loginfo(message)

                # Enregistrer dans la base de donnees
                insert_measurement(temperature, humidity)
            else:
                rospy.logwarn("Lecture echouee. Reessai...")
        except RuntimeError as error:
            # Gestion des erreurs de lecture
            rospy.logerr(f"Erreur de lecture : {error.args[0]}")
        except Exception as error:
            dht_device.exit()
            rospy.logfatal(f"Erreur critique : {error}")
            raise error
        
        rate.sleep()

# Programme principal
if __name__ == '__main__':
    try:
        create_database()  # Appele une seule fois au demarrage
        dht11_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        dht_device.exit()
