#!/usr/bin/env python3

import unittest
import rospy
import rosunit
from unittest.mock import patch
from std_msgs.msg import Float32
from sensors.msg import dht11
import time

# Ajoute le bon chemin pour importer node_dht11
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src')

from dht11_sensor_node import Node_dht11  # Import du node
class TestDHT11Node(unittest.TestCase):

    def setUp(self):
        """Initialisation du test"""
        rospy.init_node('test_dht11', anonymous=True)
        self.node = Node_dht11()
        self.received_msg = None

        # Souscrire aux topics pour vérifier les publications
        self.sub_dht11 = rospy.Subscriber('/topic_dht11', dht11, self.callback_dht11)
      
    def callback_dht11(self, msg):
        self.received_msg = msg


    @patch('Adafruit_DHT.read_retry', return_value=(60.0, 22.5))
    def test_pub_donne_dht11(self, mock_dht):
        """Test si le node publie correctement les données du capteur"""

        # Simuler un appel à la fonction de publication
        self.node.pub_donne_dht11(event=None)

        # Attendre jusqu'à 5 secondes pour recevoir le message
        timeout = time.time() + 5  # Timeout après 5 secondes
        while self.received_msg is None and time.time() < timeout:
            rospy.sleep(0.1)  # Petite pause pour attendre les messages

        # Vérifier que les valeurs sont bien publiées
        self.assertIsNotNone(self.received_msg, "Le message DHT11 n'a pas été reçu")
        self.assertEqual(self.received_msg.temperature, 22.5)
        self.assertEqual(self.received_msg.humidity, 60.0)


    @patch('Adafruit_DHT.read_retry', return_value=(None, None))
    def test_pub_donne_dht11_erreur(self, mock_dht):
        """Test si le node gère correctement une erreur du capteur"""
        
        with self.assertLogs(level="WARN") as log:
            self.node.pub_donne_dht11(event=None)
        
        self.assertTrue(any("Erreur de lecture du capteur DHT11" in message for message in log.output))

if __name__ == '__main__':
    rosunit.unitrun('sensors', 'test_dht11', TestDHT11Node)
