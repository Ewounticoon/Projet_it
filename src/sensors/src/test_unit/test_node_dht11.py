#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
from mock import patch, MagicMock
import rospy
import sys
import os

# Ajouter dynamiquement le chemin parent au PYTHONPATH
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from dht11_sensor_node import Node_dht11  # Importer la classe Node_dht11

class TestNodeDHT11(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialiser un nœud ROS pour les tests
        if not rospy.get_node_uri():
            rospy.init_node('test_node', anonymous=True)

    @patch("rospy.Publisher")  # Simule le Publisher ROS
    @patch("Adafruit_DHT.read_retry")  # Simule la lecture du capteur
    def test_lecture_valide(self, mock_read_retry, mock_publisher):
        # Simuler des valeurs valides pour le capteur
        mock_read_retry.return_value = (50.0, 25.0)  # Humidité = 50%, Température = 25°C

        # Créer une instance du noeud
        node = Node_dht11()

        # Appeler la méthode qui publie les données
        node.pub_donne_dht11(None)  # On ignore l'événement `event`

        # Vérifier que le Publisher ROS a été appelé
        self.assertTrue(mock_publisher.return_value.publish.called)

    @patch("rospy.logwarn")  # Simule le log d'erreur ROS
    @patch("Adafruit_DHT.read_retry")  # Simule la lecture du capteur
    def test_lecture_invalide(self, mock_read_retry, mock_logwarn):
        # Simuler une lecture invalide du capteur
        mock_read_retry.return_value = (None, None)

        # Créer une instance du noeud
        node = Node_dht11()

        # Appeler la méthode qui publie les données
        node.pub_donne_dht11(None)

        # Vérifier qu'un message de log d'avertissement a été émis
        mock_logwarn.assert_called_once_with("Erreur de lecture du capteur DHT11")


if __name__ == "__main__":
    unittest.main()
