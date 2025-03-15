#!/usr/bin/env python3

import unittest
import rospy
import time
import sqlite3
from std_msgs.msg import Int32
from badge_rfid.srv import ajout_badge, ajout_badgeRequest

DB_PATH = "/root/Projet_it/src/database/database/RFID_infos.db"  # 🔥 Chemin correct

class TestIntegrationAjoutBadge(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Démarrage du nœud ROS pour les tests."""
        rospy.init_node('test_integration_ajout_badge', anonymous=True)
        cls.client_ajout = rospy.ServiceProxy('ajout_badge', ajout_badge)

        # ✅ On s'assure que le service est bien actif
        rospy.wait_for_service('ajout_badge', timeout=10)

        # ✅ Nettoyer la base de test avant le test
        cls.conn = sqlite3.connect(DB_PATH)
        cls.cursor = cls.conn.cursor()
        cls.cursor.execute("DELETE FROM infos WHERE numBadge = ?", (999999,))
        cls.conn.commit()

    def test_ajout_badge_integration(self):
        """Test d'intégration de l'ajout de badge"""
        
        # ✅ 1. Simuler un badge scanné
        badge_pub = rospy.Publisher("topic_rfid", Int32, queue_size=1)
        rospy.sleep(1)  # Attendre la connexion

        badge_pub.publish(Int32(999999))  # 🔥 Simuler un badge 999999
        rospy.sleep(1)  # Laisser le temps au topic de propager l'info

        # ✅ 2. Appeler le service `ajout_badge`
        request = ajout_badgeRequest()
        request.prenom = "Alice"
        request.nom = "Dupont"
        request.age = 30
        request.mail = "alice.dupont@example.com"
        request.password = "securepassword"
        request.poste = "Développeur"

        response = self.client_ajout(request)
        self.assertTrue(response.success, "L'ajout de badge aurait dû réussir !")

        # ✅ 3. Vérifier que le badge est bien dans la base
        self.cursor.execute("SELECT * FROM infos WHERE numBadge = ?", (999999,))
        result = self.cursor.fetchone()

        self.assertIsNotNone(result, "Le badge devrait être enregistré en base !")
        self.assertEqual(result[2], "Alice", "Le prénom est incorrect")
        self.assertEqual(result[3], "Dupont", "Le nom est incorrect")
        self.assertEqual(result[5], "alice.dupont@example.com", "L'email est incorrect")

    @classmethod
    def tearDownClass(cls):
        """Nettoyage après les tests."""
        cls.cursor.execute("DELETE FROM infos WHERE numBadge = ?", (999999,))
        cls.conn.commit()
        cls.conn.close()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('badge_rfid', 'test_integration_ajout_badge', TestIntegrationAjoutBadge)
