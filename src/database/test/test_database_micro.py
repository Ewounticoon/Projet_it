#!/usr/bin/env python3
import sys
import os
import unittest
import rospy
import rosunit
import sqlite3
from unittest.mock import MagicMock, patch
from datetime import datetime
from std_msgs.msg import Float32

# Ajouter le chemin du dossier src/ pour que Python trouve sound_listener.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src')

from database_sonore import create_database, insert_measurement, sound_callback

class TestSoundListener(unittest.TestCase):

    @patch('database_sonore.sqlite3.connect')
    def test_create_database(self, mock_sqlite):
        """Test si la base de données est bien créée"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value

        create_database()

        mock_cursor.execute.assert_called()
        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()

    @patch('database_sonore.sqlite3.connect')
    def test_insert_measurement(self, mock_sqlite):
        """Test si l'insertion d'une mesure fonctionne"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value

        insert_measurement(72.5)  # Simuler l'insertion d'un volume sonore de 72.5 dB

        mock_cursor.execute.assert_called()
        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()

    @patch('database_sonore.insert_measurement')
    def test_sound_callback(self, mock_insert):
        """Test si le callback reçoit bien les messages du topic"""

        msg = Float32()
        msg.data = 80.0  # Simuler un message avec un volume sonore de 80 dB

        sound_callback(msg)

        mock_insert.assert_called_with(80.0)

if __name__ == '__main__':
    rosunit.unitrun('database', 'test_sound_listener', TestSoundListener)
