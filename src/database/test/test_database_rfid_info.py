#!/usr/bin/env python3
import sys
import os
import unittest
import rospy
import rosunit
import sqlite3
from unittest.mock import MagicMock, patch
from datetime import datetime
from std_msgs.msg import Int32

# Ajouter le chemin du dossier src/ pour que Python trouve database_RFID.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src')

from database_RFID import create_database_mesure, check_badge_in_infos, rfid_callback

class TestRFIDListener(unittest.TestCase):

    @patch('database_RFID.sqlite3.connect')
    def test_create_database_mesure(self, mock_sqlite):
        """Test si la base de données des mesures RFID est bien créée"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value

        create_database_mesure()

        mock_cursor.execute.assert_called()
        mock_conn.commit.assert_called_once()
        self.assertGreaterEqual(mock_conn.close.call_count, 1)

    @patch('database_RFID.sqlite3.connect')
    def test_check_badge_in_infos(self, mock_sqlite):
        """Test si la vérification du badge dans la base d'infos fonctionne"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value

        # Simuler un badge existant
        mock_cursor.fetchone.return_value = (123456, "John Doe", "Engineer")

        result = check_badge_in_infos(123456)
        self.assertTrue(result)

        # Simuler un badge inexistant
        mock_cursor.fetchone.return_value = None

        result = check_badge_in_infos(999999)
        self.assertFalse(result)

        mock_cursor.execute.assert_called()
        self.assertGreaterEqual(mock_conn.close.call_count, 1)


    @patch('database_RFID.sqlite3.connect')
    @patch('database_RFID.check_badge_in_infos', return_value=True)
    def test_rfid_callback_known_badge(self, mock_check, mock_sqlite):
        """Test si le callback traite correctement un badge connu"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value

        msg = Int32()
        msg.data = 123456  # Simuler un badge existant

        rfid_callback(msg)

        mock_cursor.execute.assert_called()
        mock_conn.commit.assert_called_once()
        self.assertGreaterEqual(mock_conn.close.call_count, 1)


    @patch('database_RFID.sqlite3.connect')
    @patch('database_RFID.check_badge_in_infos', return_value=False)
    def test_rfid_callback_unknown_badge(self, mock_check, mock_sqlite):
        """Test si le callback traite correctement un badge inconnu"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value

        msg = Int32()
        msg.data = 999999  # Simuler un badge inexistant

        rfid_callback(msg)

        mock_cursor.execute.assert_called()
        mock_conn.commit.assert_called_once()
        self.assertGreaterEqual(mock_conn.close.call_count, 1)


if __name__ == '__main__':
    rosunit.unitrun('database', 'test_rfid_listener', TestRFIDListener)
