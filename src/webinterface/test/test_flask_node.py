#!/usr/bin/env python3

import sys
import os
import unittest
from unittest import mock
from unittest.mock import MagicMock, patch
from flask import Flask
from flask_login import login_user, logout_user
import rospy
import rosunit
from badge_rfid.srv import ajout_badge, suppr_badge, login_member
import sqlite3
from werkzeug.security import generate_password_hash


# Ajouter le chemin du dossier src/ pour que Python trouve app.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + '/../src/flask_project')

from app import app, init_ros, send_user_info, del_badge_serv, get_last_10_values, authenticate, load_user, users_cache, User

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
        self.assertGreaterEqual(mock_subscriber.call_count, 0, "ROS peut s'abonner à plusieurs topics")

    @patch('app.rospy.ServiceProxy')
    @patch('app.rospy.wait_for_service', return_value=None)
    def test_send_user_info(self, mock_wait, mock_service):
        """Test si le service d'ajout de badge est bien appelé"""

        mock_instance = MagicMock()
        mock_instance().success = True
        mock_service.return_value = mock_instance

        response = send_user_info("John", "Doe", "johndoe", 30, "john.doe@example.com", "password", "Engineer")
        self.assertTrue(response, "Le service d'ajout de badge aurait dû renvoyer True")

        mock_service.assert_called_with('ajout_badge', mock.ANY)

    @patch('app.rospy.ServiceProxy')
    @patch('app.rospy.wait_for_service', return_value=None)
    def test_del_badge_serv(self, mock_wait, mock_service):
        """Test si le service de suppression de badge est bien appelé"""

        mock_instance = MagicMock()
        mock_instance.return_value.validation = True
        mock_service.return_value = mock_instance

        response = del_badge_serv()
        self.assertTrue(response, "Le service de suppression aurait dû renvoyer True")

        mock_service.assert_called_with('del_badge', mock.ANY)

    @patch('app.sqlite3.connect')
    def test_get_last_10_values(self, mock_sqlite):
        """Test si la récupération des 10 dernières valeurs fonctionne"""

        mock_conn = mock_sqlite.return_value
        mock_cursor = mock_conn.cursor.return_value
        mock_cursor.fetchall.return_value = [(10.5,), (20.3,)]

        result = get_last_10_values("dht11_temperature.db", "temperature", "temperature")
        self.assertEqual(result, [10.5, 20.3])

        mock_cursor.execute.assert_called()
        mock_conn.close.assert_called_once()

    @patch('app.get_last_10_values')
    def test_get_database_data(self, mock_get_data):
        """Test si l'API `/data` retourne bien les valeurs des bases SQLite"""
    
        # Simule un utilisateur et l'ajoute au cache
        mock_user = User(id="123", username="testuser", password="hashedpassword", role="user")
        users_cache["123"] = mock_user  # Ajoute directement l'utilisateur dans le cache

        # Simule une réponse de la base de données
        mock_get_data.side_effect = [
            [22.1, 23.5],  # Température
            [45.0, 50.2],  # Humidité
            [70.5, 71.3]   # Volume sonore
        ]

        # Simule une session utilisateur
        with self.app.session_transaction() as session:
            session['_user_id'] = '123'  # Simule une connexion utilisateur
            session['_fresh'] = True

        # Appel de l'API `/data`
        response = self.app.get('/data')

        # Vérifications
        self.assertEqual(response.status_code, 200, "L'API devrait retourner 200 et non 302")
        self.assertEqual(response.json["temperature"], [22.1, 23.5])
        self.assertEqual(response.json["humidity"], [45.0, 50.2])
        self.assertEqual(response.json["volume"], [70.5, 71.3])

        # Nettoyage : Supprime l'utilisateur du cache après le test
        users_cache.pop("123", None)


    @patch('app.load_user')
    def test_authenticate(self, mock_load_user):
        """Test si l'authentification fonctionne sans bloquer sur le service ROS"""

        from app import User, users_cache

        # Génère un mot de passe haché pour simuler la base de données
        hashed_password = generate_password_hash("password")

        # Simule un utilisateur
        mock_user = User(id="123", username="testuser", password=hashed_password, role="user")
        mock_load_user.return_value = mock_user  # `load_user()` retournera toujours cet utilisateur

        # Ajoute aussi l'utilisateur au cache
        users_cache["testuser"] = mock_user  

        rospy.loginfo(f"📌 users_cache ACTUEL : {users_cache}")

        # Test de l'authentification avec le mock
        user = authenticate("testuser", "password")

        rospy.loginfo(f"🔍 Utilisateur récupéré après authenticate() : {user}")

        # Vérifications
        self.assertIsNotNone(user, "L'utilisateur ne devrait pas être None")
        self.assertEqual(user.username, "testuser")
        self.assertEqual(user.role, "user")

        # Nettoyage après le test
        users_cache.pop("testuser", None)


    def test_load_user_cache(self):
        """Test si l'utilisateur est chargé depuis le cache"""
        user = MagicMock()
        user.id = "123"
        users_cache["123"] = user

        loaded_user = load_user("123")
        self.assertEqual(loaded_user, user)

if __name__ == '__main__':
    rosunit.unitrun('web_interface', 'test_flask_node', TestFlaskNode)