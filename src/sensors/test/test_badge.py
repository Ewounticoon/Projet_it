#!/usr/bin/env python3

import sys
import os
import unittest
import rospy
import rosunit
import time
from unittest.mock import MagicMock, patch
from std_msgs.msg import Int32

# Ajouter le chemin du dossier src/ pour que Python trouve badge_sensor_node.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src')

# Mock des modules qui ne fonctionnent que sur Raspberry Pi
sys.modules["RPi.GPIO"] = MagicMock()
sys.modules["pirc522"] = MagicMock()
sys.modules["pirc522.rfid"] = MagicMock()

from badge_sensor_node import Node_RFID  # Import après avoir mocké


class TestRFIDNode(unittest.TestCase):

    @patch('badge_sensor_node.RFID')
    def setUp(self, mock_rfid):
        """Initialisation du test"""
        rospy.init_node('test_rfid', anonymous=True)

        # Créer un mock de l'instance RFID AVANT d'instancier Node_RFID
        self.mock_rfid_instance = mock_rfid.return_value
        self.mock_rfid_instance.wait_for_tag.return_value = None
        self.mock_rfid_instance.request.return_value = (False, "TAG_TYPE")  # ✅ Assure 2 valeurs
        self.mock_rfid_instance.anticoll.return_value = (False, [12, 34, 56, 78, 90])  # ✅ UID simulé

        self.node = Node_RFID()  # Instancier APRES avoir mocké

        self.received_rfid = None

        # Subscriber pour écouter les messages publiés
        self.sub_rfid = rospy.Subscriber('/topic_rfid', Int32, self.callback_rfid)

        # Attendre que le subscriber soit bien connecté
        rospy.sleep(1)

    def callback_rfid(self, msg):
        """Callback pour recevoir l'ID publié"""
        self.received_rfid = msg.data

    def test_rfid_read_success(self):
        """Test si un badge est bien détecté et publié"""

        # Exécuter la lecture RFID en mode test
        self.node.read_rfid(test_mode=True)

        # Attendre la publication du message
        timeout = time.time() + 5
        while self.received_rfid is None and time.time() < timeout:
            rospy.sleep(0.1)

        # Vérifier que l'ID est bien publié
        self.assertIsNotNone(self.received_rfid, "L'ID RFID n'a pas été publié")
        self.assertEqual(self.received_rfid, 1234567890)  # ✅ Vérifie l'UID converti en entier

    def test_rfid_read_error(self):
        """Test si le node gère une erreur de lecture du badge"""

        # Modifier le mock pour simuler une erreur de lecture
        self.mock_rfid_instance.request.return_value = (True, None)  # ✅ Assure toujours 2 valeurs

        with self.assertLogs(level="INFO") as log:
            self.node.read_rfid(test_mode=True)

        # Vérifier que l'erreur est bien loggée
        self.assertTrue(any("Badge détecté" not in message for message in log.output))


if __name__ == '__main__':
    rosunit.unitrun('sensors', 'test_badge', TestRFIDNode)
