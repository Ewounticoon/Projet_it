#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sqlite3
import os
from datetime import datetime
from badge_rfid.srv import suppr_badge, suppr_badgeResponse

# Chemin de la base de données
db_path = os.path.expanduser('~/ros_workspace/src/database/src/RFID_infos.db')

# Variable globale pour stocker le numéro de badge
global_badge = None

def lecture_badge(msg):
    """Callback pour lire les messages du topic."""
    global global_badge
    global_badge = msg.data
    rospy.loginfo(f"Numéro de badge reçu : {global_badge}")

def del_badge_base(req):
    """Supprime toutes les entrées avec le numéro de badge si le service est activé."""
    global global_badge

    if req.delete:  # Vérifie si la requête est True
        if global_badge is None:
            rospy.logwarn("Aucun badge reçu pour suppression.")
            return suppr_badgeResponse(False)

        try:
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()

            # Suppression de toutes les entrées avec le numéro de badge
            cursor.execute("DELETE FROM infos WHERE numBadge = ?", (global_badge,))
            deleted_rows = conn.total_changes  # Nombre de lignes supprimées

            conn.commit()
            conn.close()

            if deleted_rows > 0:
                rospy.loginfo(f"Badge {global_badge} supprimé de la base de données.")
                return suppr_badgeResponse(True)
            else:
                rospy.logwarn(f"Aucune entrée trouvée pour le badge {global_badge}.")
                return suppr_badgeResponse(False)

        except sqlite3.Error as e:
            rospy.logerr(f"Erreur SQLite lors de la suppression : {e}")
            return suppr_badgeResponse(False)
    
    return suppr_badgeResponse(False)

def listener():
    """Initialise le noeud ROS et les services."""
    rospy.init_node('Del_badge', anonymous=True)

    # Souscrire au topic
    rospy.Subscriber("topic_rfid", Int32, lecture_badge)

    # Définir le service
    rospy.Service('del_badge', suppr_badge, del_badge_base)

    rospy.loginfo("Noeud Del_badge démarré et en attente de l'instruction...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noeud Del_badge arrêté.")
