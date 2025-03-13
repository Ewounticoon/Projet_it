#!/usr/bin/env python3

import sys
import os
import unittest
import rospy
import rosunit
from unittest.mock import MagicMock, patch, call
from std_msgs.msg import Int32
from badge_rfid.srv import ajout_badge, ajout_badgeResponse

# Ajouter le chemin du dossier src pour trouver `ajout_badge.py`
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

# Importer tout le module
import ajout_badge  # ✅ Correction ici

class TestAjoutBadge(unittest.TestCase):
    def setUp(self):
        """Initialisation avant chaque test."""
        rospy.init_node('test_ajout_badge', anonymous=True)
        
        # Simuler un badge reçu
        self.mock_badge_msg = Int32()
        self.mock_badge_msg.data = 123456
        
        # Simuler une requête valide pour le service
        self.mock_service_req = ajout_badge.ajout_badge()  # ✅ Correction ici
        self.mock_service_req.prenom = "John"
        self.mock_service_req.nom = "Doe"
        self.mock_service_req.age = 25
        self.mock_service_req.mail = "john.doe@example.com"
        self.mock_service_req.password = "securepassword"
        self.mock_service_req.poste = "Ingénieur"
    
    @patch('ajout_badge.sqlite3.connect')
    def test_create_database_infos(self, mock_sqlite_connect):
        """Test de la création de la base de données."""
        mock_conn = MagicMock()
        mock_cursor = MagicMock()
        mock_sqlite_connect.return_value = mock_conn
        mock_conn.cursor.return_value = mock_cursor

        ajout_badge.create_database_infos()  # ✅ Correction ici

        mock_cursor.execute.assert_called()
        mock_conn.commit.assert_called()
        mock_conn.close.assert_called()
    
    def test_lecture_badge(self):
        """Test de la lecture du badge via un topic."""
        ajout_badge.global_badge = None  # ✅ Réinitialisation dans le bon module

        print(f"DEBUG TEST - global_badge avant lecture: {ajout_badge.global_badge}")  # ✅ Debug

        # Appel direct de la fonction callback `lecture_badge()`
        ajout_badge.lecture_badge(self.mock_badge_msg)  # ✅ Correction ici

        print(f"DEBUG TEST - global_badge après lecture: {ajout_badge.global_badge}")  # ✅ Debug

        self.assertIsNotNone(ajout_badge.global_badge, "Le badge ne devrait pas être None après lecture.")
        self.assertEqual(ajout_badge.global_badge, 123456, "Le numéro de badge n'a pas été correctement mis à jour.")

    @patch('ajout_badge.sqlite3.connect')  # Mock SQLite
    def test_ajout_badge_base_success(self, mock_sqlite_connect):
        """Test si l'ajout d'un badge fonctionne correctement."""

        mock_conn = mock_sqlite_connect.return_value
        mock_cursor = mock_conn.cursor.return_value

        ajout_badge.global_badge = 123456  # ✅ Assurer que le badge est bien défini
        print(f"DEBUG TEST - Valeur de global_badge juste avant appel: {ajout_badge.global_badge}")  # ✅ Debug

        response = ajout_badge.ajout_badge_base(self.mock_service_req)  # ✅ Correction ici

        print("DEBUG TEST - SQL EXECUTE CALLS:", mock_cursor.execute.call_args_list)  # ✅ Debug console

        # Vérifier que execute() a bien été appelé
        mock_cursor.execute.assert_called()

        # Vérifier si la requête est correcte
        expected_sql = "INSERT INTO infos (numBadge, prenom, nom, age, mail, mdp, poste) VALUES (?, ?, ?, ?, ?, ?, ?)"
        actual_sql, actual_values = mock_cursor.execute.call_args[0]  # Récupère l'argument SQL
        
        # Normalisation des espaces et suppression des sauts de ligne
        actual_sql = " ".join(actual_sql.split())
        
        self.assertEqual(expected_sql, actual_sql, "La requête SQL exécutée ne correspond pas à celle attendue.")
        self.assertEqual(
            actual_values, 
            (123456, "John", "Doe", 25, "john.doe@example.com", "securepassword", "Ingénieur"),
            "Les valeurs insérées ne correspondent pas."
        )
        

        # Vérifier que commit et close sont appelés
        mock_conn.commit.assert_called_once()
        mock_conn.close.assert_called_once()

        # Vérifier que la réponse est bien un succès
        self.assertTrue(response.success)

    def test_ajout_badge_base_failure_no_badge(self):
        """Test d'un échec d'ajout sans badge scanné."""
        ajout_badge.global_badge = None  # ✅ Correction ici

        response = ajout_badge.ajout_badge_base(self.mock_service_req)  # ✅ Correction ici

        self.assertFalse(response.success)

if __name__ == '__main__':
    rosunit.unitrun('badge_rfid', 'test_ajout_badge', TestAjoutBadge)
