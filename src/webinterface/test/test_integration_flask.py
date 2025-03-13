#!/usr/bin/env python3

import unittest
import rospy
import requests
import time

FLASK_URL = "http://localhost:5000"

class TestIntegrationFlask(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Vérifier que Flask est bien démarré"""
        rospy.init_node('test_integration_flask', anonymous=True)
        time.sleep(5)  # 🔥 Laisser Flask se lancer

    def test_get_sensor_data(self):
        """Tester si Flask récupère bien les données ROS des capteurs."""
        response = requests.get(f"{FLASK_URL}/data")
        self.assertEqual(response.status_code, 200, "L'API /data devrait retourner 200")
        
        data = response.json()
        self.assertIn("temperature", data, "La clé temperature doit être présente")
        self.assertIn("humidity", data, "La clé humidity doit être présente")
        self.assertIn("volume", data, "La clé volume doit être présente")

    def test_get_database_data(self):
        """Tester si Flask récupère bien les données des bases SQLite."""
        response = requests.get(f"{FLASK_URL}/get_data")
        self.assertEqual(response.status_code, 200, "L'API /get_data devrait retourner 200")
        
        data = response.json()
        self.assertIn("temperature", data, "La clé temperature doit être présente")
        self.assertIn("humidite", data, "La clé humidite doit être présente")
        self.assertIn("volume", data, "La clé volume doit être présente")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('webinterface', 'test_integration_flask', TestIntegrationFlask)
