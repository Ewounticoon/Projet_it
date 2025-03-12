#!/usr/bin/env python3
import unittest
from unittest.mock import patch, MagicMock  # Correctement utiliser unittest.mock
import rospy
import RPi.GPIO as GPIO

# Ajouter dynamiquement le chemin parent pour trouver le fichier principal
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from badge_sensor_node import Node_RFID  # Importer la classe principale

class TestNodeRFID(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialiser un nœud ROS pour les tests."""
        if not rospy.get_node_uri():
            rospy.init_node('test_node_rfid', anonymous=True)

    @classmethod
    def tearDownClass(cls):
        """Nettoyage des GPIO après tous les tests."""
        try:
            GPIO.cleanup()
        except RuntimeWarning:
            pass

    @patch("pirc522.RFID")  # Simuler l'initialisation du lecteur RFID
    @patch("rospy.Publisher")  # Simuler le Publisher ROS
    def test_lecture_valide(self, mock_publisher, mock_rfid):
        """Tester la lecture valide d'un badge RFID."""
        # Simuler le comportement du lecteur RFID
        mock_rc522_instance = MagicMock()
        mock_rfid.return_value = mock_rc522_instance
        mock_rc522_instance.wait_for_tag.return_value = None
        mock_rc522_instance.request.return_value = (False, "Tag_Type")  # Pas d'erreur
        mock_rc522_instance.anticoll.return_value = (False, [1, 2, 3, 4, 5])  # UID valide

        # Créer une instance du noeud
        node = Node_RFID()

        # Lire une seule fois le badge
        node.read_rfid(max_iterations=1)

        # Vérifier que le Publisher a publié l'ID attendu
        expected_id = int(''.join(map(str, [1, 2, 3, 4, 5])))
        mock_publisher.return_value.publish.assert_called_once_with(expected_id)

    @patch("rospy.logwarn")  # Simuler les logs ROS pour les erreurs
    @patch("pirc522.RFID")  # Simuler le lecteur RFID
    def test_lecture_invalide(self, mock_rfid, mock_logwarn):
        """Tester la lecture invalide d'un badge RFID."""
        # Simuler une erreur lors de la lecture du tag
        mock_rc522_instance = MagicMock()
        mock_rfid.return_value = mock_rc522_instance
        mock_rc522_instance.wait_for_tag.return_value = None
        mock_rc522_instance.request.return_value = (True, None)  # Erreur détectée

        # Créer une instance du noeud
        node = Node_RFID()

        # Lire une seule fois le badge
        node.read_rfid(max_iterations=1)

        # Vérifier qu'un log d'avertissement a été généré
        mock_logwarn.assert_called_once_with("Erreur de lecture du tag RFID")

if __name__ == "__main__":
    unittest.main()
