#!/usr/bin/env python3.5
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
from pirc522 import RFID
import time


class Node_RFID:
    def __init__(self):
        # Initialisation des GPIOs
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Initialisation du lecteur RFID
        self.rc522 = RFID()

        # Initialisation du publisher
        self.pub_rfid = rospy.Publisher('/topic_rfid', Int32, queue_size=10)

        rospy.loginfo("En attente d'un badge (Ctrl + C pour quitter)")

    def read_rfid(self, test_mode=False):
        """Lit un badge RFID, s'arrête après une lecture si test_mode=True"""
    
        while not rospy.is_shutdown():
            self.rc522.wait_for_tag()
            (error, tag_type) = self.rc522.request()
    
            if error:
                rospy.logwarn("Erreur lors de la détection du badge RFID")  # ✅ Ajout du log d'erreur
                if test_mode:
                    return  
    
            (error, uid) = self.rc522.anticoll()
    
            if error:
                rospy.logwarn("Erreur lors de la récupération de l'UID du badge RFID")  # ✅ Ajout du log d'erreur
                if test_mode:
                    return  

            rfid_id = int(''.join(map(str, uid)))  # Convertir l'UID en entier
            rospy.loginfo(f"Badge détecté avec l'ID : {rfid_id}")
            self.pub_rfid.publish(rfid_id)
    
            if test_mode:  # ✅ Sortir immédiatement après une seule lecture en mode test
                return  
    
            time.sleep(1)  # Évite une lecture trop rapide en boucle
    



def main():
    rospy.init_node('node_rfid', anonymous=True)
    rfid_node = Node_RFID()
    rfid_node.read_rfid()
    rospy.spin()


if __name__ == '__main__':
    main()
