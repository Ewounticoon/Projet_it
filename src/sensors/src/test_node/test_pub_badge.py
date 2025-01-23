#!/usr/bin/env python3.5
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int32  # Ajouter l'import pour le message Int32
import time


class Node_RFID:
    def __init__(self):
       
        # Initialisation du publisher (topic pour l'ID RFID)
        self.pub_rfid = rospy.Publisher('/topic_rfid', Int32, queue_size=10)  # Utilisation de self.pub_rfid
        
        # Affichage dans le terminal pour l'utilisateur
        rospy.loginfo("En attente d'un badge (pour quitter, Ctrl + c)")

    def read_rfid(self):
        # Boucle infinie pour lire les tags RFID
        while not rospy.is_shutdown():
            rfid_id=123456
            # Publier l'ID sur le topic /topic_rfid
            self.pub_rfid.publish(rfid_id)
            
            # Attente de 1 seconde pour éviter une lecture rapide en boucle
            time.sleep(1)

def main():
    # Initialise le noeud ROS
    rospy.init_node('node_rfid', anonymous=True)
    
    # Création de l'objet pour la lecture RFID
    rfid_node = Node_RFID()
    
    # Lancer la lecture RFID
    rfid_node.read_rfid()
    
    # Garder le noeud en fonctionnement
    rospy.spin()

if __name__ == '__main__':
    main()
