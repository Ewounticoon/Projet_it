#!/usr/bin/env python3
import sys
import os
import unittest
from unittest import mock  # ✅ Import du module mock

import rospy
import rosunit
import sqlite3
from unittest.mock import MagicMock, patch
from flask import Flask
from flask.testing import FlaskClient
from badge_rfid.srv import suppr_badge, ajout_badge  # ✅ Ajout des imports ROS


# Ajouter le chemin du dossier src/ pour que Python trouve app.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src/flask_project')

from app import app, init_ros, send_user_info, del_badge_serv, get_last_10_values

class TestFlaskNode(unittest.TestCase):

    def setUp(self):
        """Initialisation du client Flask pour les tests"""
        self.app = app.test_client()
        self.app.testing = True

    @patch('app.rospy.init_node')
    @patch('app.rospy.Subscriber')
    def test_init_ros(self, mock_subscriber, mock_init_node):
        """Test si ROS est bien initialisé dans Flask"""

        init_ros()

        mock_init_node.assert_called_with('flask_node', anonymous=True)
        self.assertGreaterEqual(mock_subscriber.call_count, 2, "ROS devrait s'abonner à plusieurs topics")


    @patch('app.rospy.ServiceProxy')
    @patch('app.rospy.wait_for_service', return_value=None)
    def test_send_user_info(self, mock_wait, mock_service):
        """Test si le service d'ajout de badge est bien appelé"""

        mock_instance = MagicMock()
        mock_instance().success = True
        mock_service.return_value = mock_instance

        response = send_user_info("John", "Doe", 30, "john.doe@example.com", "password", "Engineer")
        self.assertTrue(response, "Le service d'ajout de badge aurait dû renvoyer True")

        mock_service.assert_called_with('ajout_badge', mock.ANY)  # ✅ Correction




    @patch('app.rospy.ServiceProxy')
    @patch('app.rospy.wait_for_service', return_value=None)
    def test_del_badge_serv(self, mock_wait, mock_service):
        """Test si le service de suppression de badge est bien appelé"""

        mock_instance = MagicMock()
        mock_instance.return_value.validation = True  # ✅ Correction ici !
        mock_service.return_value = mock_instance

        response = del_badge_serv()
        self.assertTrue(response, "Le service de suppression aurait dû renvoyer True")

        mock_service.assert_called_with('del_badge', mock.ANY)  # ✅ Correction





    @patch('app.sqlite3.connect')
    def test_get_last_10_values(self, mock_sqlite):
        """Test si la récupération des 10 dernières valeurs fonctionne"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value
        mock_cursor.fetchall.return_value = [(10.5,), (20.3,)]

        result = get_last_10_values("dht11_temperature.db", "temperature")
        self.assertEqual(result, [10.5, 20.3])

        mock_cursor.execute.assert_called()
        mock_conn.close.assert_called_once()

    def test_get_sensor_data(self):
        """Test si l'API `/data` retourne bien les valeurs des capteurs"""

        response = self.app.get('/data')
        self.assertEqual(response.status_code, 200)
        self.assertIn('temperature', response.json)
        self.assertIn('humidity', response.json)
        self.assertIn('volume', response.json)

    @patch('app.get_last_10_values')
    def test_get_database_data(self, mock_get_data):
        """Test si l'API `/get_data` retourne bien les valeurs des bases SQLite"""

        mock_get_data.side_effect = [[22.1, 23.5], [45.0, 50.2], [70.5, 71.3]]

        response = self.app.get('/get_data')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json["temperature"], [22.1, 23.5])
        self.assertEqual(response.json["humidite"], [45.0, 50.2])
        self.assertEqual(response.json["volume"], [70.5, 71.3])

if __name__ == '__main__':
    rosunit.unitrun('web_interface', 'test_flask_node', TestFlaskNode)
