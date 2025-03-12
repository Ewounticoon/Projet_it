#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sqlite3
import os
from datetime import datetime
from badge_rfid.srv import suppr_badge, suppr_badgeResponse

db_path = '/root/ros_workspace/src/database/src/RFID_infos.db'

global_badge = None

def lecture_badge(msg):
    global global_badge
    global_badge = msg.data
    rospy.loginfo(f"Numéro de badge reçu : {global_badge}")

def del_badge_base(req):
    global global_badge

    if global_badge is None:
        rospy.logwarn("Aucun badge reçu.")
        return suppr_badgeResponse(False)

    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        cursor.execute("DELETE FROM infos WHERE numBadge = ?", (global_badge,))
        conn.commit()

        if conn.total_changes > 0:
            rospy.loginfo(f"Badge {global_badge} supprimé.")
            return suppr_badgeResponse(True)
        else:
            rospy.logwarn(f"Aucune entrée pour le badge : {global_badge}")
            return suppr_badgeResponse(False)

    except sqlite3.Error as e:
        rospy.logerr(f"Erreur SQLite : {e}")
        return suppr_badgeResponse(False)

    finally:
        conn.close()

def listener():
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
