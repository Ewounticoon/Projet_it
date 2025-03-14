#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sqlite3
import os

from badge_rfid.srv import login_member, login_memberReponse

# Chemin de la base de données
db_path = os.path.expanduser('~/ros_workspace/src/database/database/RFID_infos.db')

def extract_data(req):
    username = req.user

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    try:
        
        cursor.execute('''
            SELECT * FROM infos WHERE user = 'username';
        ''')
        data_extracted = cursor.fetchall() #stockage des données dans une variable

        rospy.loginfo("Donnees extraites avec succes")
        success = True
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de l'extraction des données : {e}")
        success = False
    finally:
        conn.commit()
        conn.close()



def listener():
    """Initialise le noeud ROS et les services."""
    rospy.init_node('nav_database', anonymous=True)

    # Définir le service
    rospy.Service('login_serv', login_member)

    rospy.loginfo("Noeud nav_database démarré et en attente de messages...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noeud de naviagtion database arrêté.")