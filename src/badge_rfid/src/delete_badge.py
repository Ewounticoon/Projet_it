#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sqlite3
import os
import sys
from badge_rfid.srv import suppr_badge, suppr_badgeResponse

# Chemin de la base de données
db_path = '/root/Projet_it/src/database/database/RFID_infos.db'

# Variable globale pour stocker le numéro de badge
global_badge = None

def lecture_badge(msg):
    """Callback pour lire les messages du topic."""
    global global_badge  # ✅ Assurer que la variable globale est mise à jour
    global_badge = msg.data
    rospy.loginfo(f"Numéro de badge reçu : {global_badge}")

    # ✅ Debug print
    print(f"📌 [DEBUG] Numéro de badge mis à jour : {global_badge}")
    sys.stdout.flush()


def del_badge_base(req):
    """Supprime un badge de la base de données."""
    global global_badge

    print(f"📌 [DEBUG] Valeur de global_badge avant suppression : {global_badge}")
    sys.stdout.flush()

    if global_badge is None:
        rospy.logwarn("Aucun badge reçu.")
        print("❌ [ERROR] Aucun badge reçu.")
        sys.stdout.flush()
        return suppr_badgeResponse(validation=False)

    conn = None  # ✅ Initialise conn pour éviter l'erreur

    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        print(f"📌 [DEBUG] Suppression du badge en cours : {global_badge}")
        sys.stdout.flush()

        cursor.execute("DELETE FROM infos WHERE numBadge = ?", (global_badge,))
        conn.commit()

        print(f"📌 [DEBUG] total_changes après suppression : {conn.total_changes}")
        sys.stdout.flush()

        if conn.total_changes > 0:
            rospy.loginfo(f"Badge {global_badge} supprimé.")
            print(f"✅ [SUCCESS] Badge {global_badge} supprimé.")
            sys.stdout.flush()
            return suppr_badgeResponse(validation=True)
        else:
            rospy.logwarn(f"Aucune entrée pour le badge : {global_badge}")
            print(f"⚠️ [WARNING] Aucune entrée trouvée pour le badge {global_badge}.")
            sys.stdout.flush()
            return suppr_badgeResponse(validation=False)

    except sqlite3.Error as e:
        rospy.logerr(f"Erreur SQLite : {e}")
        print(f"❌ [ERROR] Erreur SQL : {e}")
        sys.stdout.flush()
        return suppr_badgeResponse(validation=False)

    finally:
        if conn:  # ✅ Vérifie que la connexion a bien été créée avant de la fermer
            conn.close()



def listener():
    """Initialise le noeud ROS et les services."""
    rospy.init_node('Del_badge', anonymous=True)
    rospy.Subscriber("topic_rfid", Int32, lecture_badge)
    rospy.Service('del_badge', suppr_badge, del_badge_base)

    rospy.loginfo("Noeud Del_badge démarré et en attente d'instructions...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noeud Del_badge arrêté.")
