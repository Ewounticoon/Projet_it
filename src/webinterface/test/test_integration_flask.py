#!/usr/bin/env python3

import unittest
import rospy
import requests
import time

FLASK_URL = "http://localhost:5000"

class TestIntegrationFlask(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Vérifier que Flask est bien démarré et se connecter"""
        rospy.init_node('test_integration_flask', anonymous=True)
        time.sleep(5)  # Laisser Flask se lancer
        cls.session = requests.Session()  # 🔥 Créer une session persistante

        # 🔑 Connexion avec un utilisateur valide
        login_data = {
            "username": "admin",  # 🔥 Mets ici un identifiant valide
            "password": "admin"  # 🔥 Mets ici le bon mot de passe
        }
        response = cls.session.post(f"{FLASK_URL}/", data=login_data)

        if response.status_code != 200:
            print("🔴 Échec de la connexion :", response.text)

    def test_get_database_data(self):
        """Tester si Flask récupère bien les données des bases SQLite après connexion."""
        response = self.session.get(f"{FLASK_URL}/data")  # 🔥 Utiliser la session connectée

        print("Réponse brute :", response.text)  # 🔥 Debug
        print("Statut HTTP :", response.status_code)  # 🔥 Debug

        self.assertEqual(response.status_code, 200, "L'API /data devrait retourner 200")
        
        try:
            data = response.json()
            print("Données JSON :", data)  # 🔥 Debug
        except requests.exceptions.JSONDecodeError as e:
            print("Erreur JSON :", e)
            print("Contenu brut reçu :", response.text)
            self.fail("Le serveur n'a pas retourné de JSON valide.")

        self.assertIn("temperature", data, "La clé temperature doit être présente")
        self.assertIn("humidity", data, "La clé humidite doit être présente")
        self.assertIn("volume", data, "La clé volume doit être présente")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('webinterface', 'test_integration_flask', TestIntegrationFlask)
