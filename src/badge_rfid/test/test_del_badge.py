#!/usr/bin/env python3
import sys
import os
import unittest
import rospy
import rosunit
import sqlite3
from unittest.mock import MagicMock, patch
from std_msgs.msg import Int32
from badge_rfid.srv import suppr_badge, suppr_badgeRequest, suppr_badgeResponse

# Ajouter le chemin du dossier src/ pour que Python trouve del_badge.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src')

import delete_badge 

class TestDelBadge(unittest.TestCase):

    def setUp(self):
        """Initialisation du test"""
        rospy.init_node('test_del_badge', anonymous=True)
        global global_badge
        global_badge = None  # Simule le stockage du badge

    def test_lecture_badge(self):
        """Test si la lecture d'un badge met à jour global_badge"""
        delete_badge.global_badge = None
        msg = Int32()
        msg.data = 123456
        delete_badge.lecture_badge(msg)  # Simuler la réception d'un message

        self.assertEqual(delete_badge.global_badge, 123456, "Le numéro de badge n'a pas été correctement mis à jour")

    @patch('delete_badge.sqlite3.connect')
    def test_del_badge_base_success(self, mock_sqlite):
        """Test si la suppression d'un badge existant fonctionne"""

        global global_badge
        global_badge = 123456  # ✅ Simuler un badge reçu
        delete_badge.global_badge = 123456 
        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value
        mock_conn.total_changes = 1  # ✅ Simuler qu'une ligne a été supprimée

        req = suppr_badgeRequest()
        response = delete_badge.del_badge_base(req)

        self.assertTrue(response.validation, "Le badge aurait dû être supprimé")
        mock_cursor.execute.assert_called_with("DELETE FROM infos WHERE numBadge = ?", (global_badge,))
        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()

    @patch('delete_badge.sqlite3.connect')
    def test_del_badge_base_not_found(self, mock_sqlite):
        """Test si la suppression d'un badge inexistant retourne False"""

        global global_badge
        global_badge=999999
        delete_badge.global_badge = 999999  # ✅ Simuler un badge inexistant
        print(f"📌 [DEBUG] Valeur de global_badge avant appel au service : {global_badge}")
        sys.stdout.flush()

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value
        mock_conn.total_changes = 0  # ✅ Simuler qu'aucune ligne n'a été supprimée

        req = suppr_badgeRequest()
        response = delete_badge.del_badge_base(req)

        print(f"📌 [DEBUG] Réponse du service : {response.validation}")
        sys.stdout.flush()

        self.assertFalse(response.validation, "Le service aurait dû renvoyer False car le badge n'existe pas")
        mock_cursor.execute.assert_called_with("DELETE FROM infos WHERE numBadge = ?", (global_badge,))
        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()


    @patch('delete_badge.sqlite3.connect', side_effect=sqlite3.Error("Erreur SQL"))
    def test_del_badge_base_sql_error(self, mock_sqlite):
        """Test si une erreur SQL est bien gérée"""
        
        global global_badge
        global_badge = 123456  # ✅ Simuler un badge existant

        req = suppr_badgeRequest()
        response = delete_badge.del_badge_base(req)

        self.assertFalse(response.validation, "Le service aurait dû renvoyer False en cas d'erreur SQL")

if __name__ == '__main__':
    rosunit.unitrun('badge_rfid', 'test_del_badge', TestDelBadge)
