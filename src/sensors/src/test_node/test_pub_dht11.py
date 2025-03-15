#!/usr/bin/env python3

import rospy
from sensors.msg import dht11
import random

class Node_dht11:
    def __init__(self):


        # Initialisation des publisher (nom du topic, type ,taille)
        self.pub=rospy.Publisher('/topic_dht11', dht11, queue_size=10)

        # Lancer une fonction à intervalle régulier :
        rospy.Timer(rospy.Duration(5), self.pub_donne_dht11)
        
        # Ecrit dans le terminal
        rospy.loginfo("Démarrage du node pour DHT11")

    # Publier sur un topic
    def pub_donne_dht11(self, event):
        msg_dht11=dht11()
        temperature = round(random.uniform(20, 30), 2)  # Température entre 20 et 30°C
        humidity = round(random.uniform(40, 60), 2)  # Humidité entre 40% et 60%


        msg_dht11.temperature=temperature
        msg_dht11.humidity=humidity
        # Publier les messages
        self.pub.publish(msg_dht11)


# Démarrer le noeud
def main():
    # Initialise le noeud
    rospy.init_node('node_dht11_no_sensor')
    serial_node = Node_dht11()
    # Lance le noeud
    rospy.spin()

if __name__ == '__main__':
    main()
