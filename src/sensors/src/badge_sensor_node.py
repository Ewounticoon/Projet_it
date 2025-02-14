#!/usr/bin/env python3.5
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int32  # Ajouter l'import pour le message Int32
import RPi.GPIO as GPIO
from pirc522 import RFID
import time


class Node_RFID:
    def __init__(self):
        # Initialisation des GPIOs
        GPIO.setmode(GPIO.BOARD)  # Définit le mode de numérotation (Board)
        GPIO.setwarnings(False)  # Désactive les messages d'alerte
        
        # Initialisation du lecteur RFID
        self.rc522 = RFID()
        
        # Initialisation du publisher (topic pour l'ID RFID)
        self.pub_rfid = rospy.Publisher('/topic_rfid', Int32, queue_size=10)  # Utilisation de self.pub_rfid
        
        # Affichage dans le terminal pour l'utilisateur
        rospy.loginfo("En attente d'un badge (pour quitter, Ctrl + c)")

    def read_rfid(self, max_iterations=None):
        iteration_count = 0

        # Boucle infinie pour lire les tags RFID
        while not rospy.is_shutdown():
            self.rc522.wait_for_tag()  # Attente qu'une puce RFID soit détectée
            (error, tag_type) = self.rc522.request()  # Récupération des infos de la puce

            if not error:
                # En cas de succès, on récupère l'UID de la puce
                (error, uid) = self.rc522.anticoll()
                
                if not error:
                    # Convertir l'UID en un entier (si nécessaire)
                    rfid_id = int(''.join(map(str, uid)))  # Convertir l'UID en un entier
                    
                    rospy.loginfo(f"Badge détecté avec l'ID : {rfid_id}")
                    
                    # Publier l'ID sur le topic /topic_rfid
                    self.pub_rfid.publish(rfid_id)
                    
                    # Attente de 1 seconde pour éviter une lecture rapide en boucle
                    time.sleep(1)


            if max_iterations is not None:
                iteration_count += 1
                if iteration_count >= max_iterations:
                    break
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
