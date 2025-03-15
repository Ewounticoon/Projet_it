#!/usr/bin/env python3

import sys
import os
import unittest
import rospy
import rosunit
from unittest.mock import MagicMock, patch
from badge_rfid.srv import login_member, login_memberResponse  # ✅ Import correct du service ROS

# Ajouter le chemin du dossier src pour trouver `nav_database_badge.py`
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

# Importer le bon module
import nav_database_badge  # ✅ Correction

class TestNavDatabaseBadge(unittest.TestCase):

    def setUp(self):
        """Initialisation avant chaque test."""
        rospy.init_node('test_nav_database_badge', anonymous=True)

        # ✅ Création correcte d'une requête ROS avec le service `login_member`
        self.mock_service_req = login_member()  # ✅ Correction
        self.mock_service_req.username="johndoe"

    @patch('nav_database_badge.sqlite3.connect')  # ✅ Correction du mock
    def test_extract_data_success(self, mock_sqlite_connect):
        """Test si l'extraction des données fonctionne pour un utilisateur existant."""

        mock_conn = MagicMock()
        mock_cursor = MagicMock()
        mock_sqlite_connect.return_value = mock_conn
        mock_conn.cursor.return_value = mock_cursor

        # Simuler une réponse de la base de données
        mock_cursor.fetchone.return_value = (1, "hashedpassword123", "Ingénieur")

        # Exécuter la fonction
        response = nav_database_badge.extract_data(self.mock_service_req)  # ✅ Correction ici

        # Vérifications SQL
        mock_cursor.execute.assert_called_once_with(
            '''SELECT id, mdp, poste FROM infos WHERE user = ?;''', ("johndoe",)
        )
        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()

        # Vérifications du retour du service
        self.assertTrue(response.success, "Le service aurait dû réussir.")
        self.assertEqual(response.id, 1, "L'ID ne correspond pas.")
        self.assertEqual(response.username, "johndoe", "Le username devrait être 'johndoe' si trouvé dans la base.")
        self.assertEqual(response.password, "hashedpassword123", "Le mot de passe ne correspond pas.")
        self.assertEqual(response.role, "Ingénieur", "Le rôle ne correspond pas.")

    @patch('nav_database_badge.sqlite3.connect')  # ✅ Correction du mock
    def test_extract_data_failure_no_user(self, mock_sqlite_connect):
        """Test si l'extraction échoue pour un utilisateur inexistant."""

        mock_conn = MagicMock()
        mock_cursor = MagicMock()
        mock_sqlite_connect.return_value = mock_conn
        mock_conn.cursor.return_value = mock_cursor

        # Simuler une réponse vide (aucun utilisateur trouvé)
        mock_cursor.fetchone.return_value = None

        # Exécuter la fonction
        response = nav_database_badge.extract_data(self.mock_service_req)  

        # Vérifications SQL
        mock_cursor.execute.assert_called_once_with(
            '''SELECT id, mdp, poste FROM infos WHERE user = ?;''', ("johndoe",)
        )
        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()

        # Vérifications du retour du service
        self.assertFalse(response.success, "Le service aurait dû échouer.")
        self.assertEqual(response.id, 0, "L'ID aurait du etre 0.")
        self.assertEqual(response.username, '', "Le username aurait du etre vide")
        self.assertEqual(response.password,'', "Le mot de passe aurait dû être vide.")
        self.assertEqual(response.role,'', "Le rôle aurait dû être vide")

if __name__ == '__main__':
    rosunit.unitrun('badge_rfid', 'test_nav_database_badge', TestNavDatabaseBadge)
