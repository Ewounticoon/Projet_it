#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import Adafruit_DHT
import time
from sensors.msg import dht11


class Node_dht11:
    def __init__(self):
        self.sensor = Adafruit_DHT.DHT11
        self.pin = 2

        # Initialisation des publisher (nom du topic, type ,taille)
        self.pub=rospy.Publisher('/topic_dht11', dht11, queue_size=10)
        self.pub_temp = rospy.Publisher('/topic_tempDHT11', Float32, queue_size=10)
        self.pub_hum = rospy.Publisher('/topic_humDHT11', Float32, queue_size=10)

        # Lancer une fonction à intervalle régulier :
        rospy.Timer(rospy.Duration(5), self.pub_donne_dht11)
        
        # Ecrit dans le terminal
        rospy.loginfo("Démarrage du node pour DHT11")

    # Publier sur un topic
    def pub_donne_dht11(self, event):
        humidity, temperature = Adafruit_DHT.read_retry(self.sensor, self.pin)
        
        if humidity is not None and temperature is not None:
            # Créer un message Float32 pour la température et l'humidité
            msg_temp = Float32()
            msg_hum = Float32()
            msg_dht11=dht11()

            msg_temp.data = temperature
            msg_hum.data = humidity

            msg_dht11.temperature=temperature
            msg_dht11.humidity=humidity
            # Publier les messages
            self.pub_temp.publish(msg_temp)
            self.pub_hum.publish(msg_hum)
            self.pub.publish(msg_dht11)
        else:
            rospy.logwarn("Erreur de lecture du capteur DHT11")

# Démarrer le noeud
def main():
    # Initialise le noeud
    rospy.init_node('node_dht11')
    serial_node = Node_dht11()
    # Lance le noeud
    rospy.spin()

if __name__ == '__main__':
    main()
