#!/usr/bin/env python3
import sys
import os
import unittest
import rospy
import rosunit
import time
from unittest.mock import MagicMock, patch
from std_msgs.msg import Float32

# Ajouter le chemin du dossier src/ pour que Python trouve node_micro.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src')

# Mock du module pyfirmata qui contrôle l’Arduino
sys.modules["pyfirmata"] = MagicMock()

from micro_sensor_node import Node_micro  # Import après avoir mocké

class TestNodeMicro(unittest.TestCase):

    @patch('micro_sensor_node.Arduino')
    def setUp(self, mock_arduino):
        """Initialisation du test"""
        rospy.init_node('test_micro', anonymous=True)

        # Simuler une carte Arduino connectée sur '/dev/ttyUSB0'
        self.mock_arduino_instance = mock_arduino.return_value
        self.mock_analog_pin = MagicMock()
        self.mock_arduino_instance.analog = [self.mock_analog_pin]

        # Simuler une valeur analogique (par exemple 0.5 correspondant à 2.5V)
        self.mock_analog_pin.read.return_value = 0.5

        # Initialiser le node avec le mock
        self.node = Node_micro()

        self.received_msg = None

        # Subscriber pour écouter les messages publiés
        self.sub_micro = rospy.Subscriber('/topic_micro', Float32, self.callback_micro)

        # Attendre que le subscriber soit bien connecté
        rospy.sleep(1)

    def callback_micro(self, msg):
        """Callback pour recevoir la valeur publiée"""
        self.received_msg = msg.data

    def test_voltage_to_db(self):
        """Test de conversion tension -> dB"""
        self.assertAlmostEqual(self.node.voltage_to_db(1.0, 1.0), 0.0)  # 1V / 1V -> 0 dB
        self.assertAlmostEqual(self.node.voltage_to_db(0.5, 1.0), -6.02, places=2)  # 0.5V / 1V
        self.assertAlmostEqual(self.node.voltage_to_db(2.0, 1.0), 6.02, places=2)  # 2V / 1V
        self.assertEqual(self.node.voltage_to_db(0, 1.0), -float('inf'))  # Cas où voltage = 0

    def test_pub_donne_micro(self):
        """Test si le node publie correctement les valeurs en dB"""
        self.node.pub_donne_micro(event=None)  # Simuler un appel à la fonction

        # Attendre la publication du message
        timeout = time.time() + 5
        while self.received_msg is None and time.time() < timeout:
            rospy.sleep(0.1)

        # Vérifier que la valeur publiée correspond bien à celle attendue (0.5 * 5.0 = 2.5V)
        expected_db_value = self.node.voltage_to_db(2.5, 1.0)
        self.assertIsNotNone(self.received_msg, "La valeur n'a pas été publiée")
        self.assertAlmostEqual(self.received_msg, expected_db_value, places=2)

    def test_pub_donne_micro_none(self):
        """Test si le node gère bien une valeur None en ne publiant rien"""
        self.mock_analog_pin.read.return_value = None  # ✅ Simule un retour None
    
        with self.assertLogs(level="WARN") as log:  # ✅ Vérifie que le warning est bien loggé
            self.node.pub_donne_micro(event=None)
    
        # Vérifier qu'il y a bien un message WARN "Valeur analogique None, aucune publication"
        self.assertTrue(any("Valeur analogique None, aucune publication" in message for message in log.output))
    


if __name__ == '__main__':
    rosunit.unitrun('sensors', 'test_micro', TestNodeMicro)
