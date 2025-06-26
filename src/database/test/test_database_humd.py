#!/usr/bin/env python3
import sys
import os
import unittest
import rospy
import rosunit
import sqlite3
from unittest.mock import MagicMock, patch
from datetime import datetime
from sensors.msg import dht11

# Ajouter le chemin du dossier src/ pour que Python trouve humidite_listener.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src')

from database_humdht11 import create_database, insert_measurement, hum_callback

class TestHumiditeListener(unittest.TestCase):

    @patch('database_humdht11.sqlite3.connect')
    def test_create_database(self, mock_sqlite):
        """Test si la base de données est bien créée"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value

        create_database()

        mock_cursor.execute.assert_called()

        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()

    @patch('database_humdht11.sqlite3.connect')
    def test_insert_measurement(self, mock_sqlite):
        """Test si l'insertion d'une mesure fonctionne"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value

        insert_measurement(45.3)  # Simuler l'insertion d'une humidité de 45.3%

        mock_cursor.execute.assert_called()

        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()

    @patch('database_humdht11.insert_measurement')
    def test_hum_callback(self, mock_insert):
        """Test si le callback reçoit bien les messages du topic"""

        msg = dht11()
        msg.humidity = 55.0  # Simuler un message avec humidité à 55%

        hum_callback(msg)

        mock_insert.assert_called_with(55.0)

if __name__ == '__main__':
    rosunit.unitrun('sensors', 'test_humidite_listener', TestHumiditeListener)
