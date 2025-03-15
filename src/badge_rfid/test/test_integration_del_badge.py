#!/usr/bin/env python3

import unittest
import rospy
import sqlite3
import time
from std_msgs.msg import Int32
from badge_rfid.srv import suppr_badge

DB_PATH = "/root/Projet_it/src/database/database/RFID_infos.db"

class TestIntegrationSuppressionBadge(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Préparer le test en ajoutant un badge en base."""
        rospy.init_node('test_integration_del_badge', anonymous=True)
        cls.client_suppression = rospy.ServiceProxy('del_badge', suppr_badge)
        rospy.wait_for_service('del_badge', timeout=10)

        # ✅ Ajouter un badge manuellement en base
        cls.conn = sqlite3.connect(DB_PATH)
        cls.cursor = cls.conn.cursor()
        cls.cursor.execute("INSERT INTO infos (numBadge, prenom, nom, age, mail, mdp, poste) VALUES (888888, 'Bob', 'Martin', 40, 'bob.martin@example.com', 'mdp_test', 'Manager')")
        cls.conn.commit()

    def test_suppression_badge_integration(self):
        """Test de suppression d'un badge"""
        
        # ✅ 1. Simuler un badge scanné
        badge_pub = rospy.Publisher("topic_rfid", Int32, queue_size=1)
        rospy.sleep(1)  # Attendre la connexion

        badge_pub.publish(Int32(888888))  # 🔥 Publier le badge 888888
        rospy.sleep(1)

        # ✅ 2. Appeler le service `del_badge`
        response = self.client_suppression(True)
        self.assertTrue(response.validation, "Le badge aurait dû être supprimé !")

        # ✅ 3. Vérifier que le badge **n'est plus dans la base**
        self.cursor.execute("SELECT * FROM infos WHERE numBadge = ?", (888888,))
        result = self.cursor.fetchone()

        self.assertIsNone(result, "Le badge devrait avoir été supprimé !")

    @classmethod
    def tearDownClass(cls):
        """Nettoyage après les tests."""
        cls.cursor.execute("DELETE FROM infos WHERE numBadge = ?", (888888,))
        cls.conn.commit()
        cls.conn.close()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('badge_rfid', 'test_integration_del_badge', TestIntegrationSuppressionBadge)
