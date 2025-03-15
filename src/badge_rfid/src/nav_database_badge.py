#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sqlite3
import os

from badge_rfid.srv import login_member, login_memberResponse
import rospkg

#Chemin vers database
rospack = rospkg.RosPack()
package_path = rospack.get_path('database')

db_path=os.path.join(package_path, 'database', 'RFID_infos.db') #chemin d'acces

def extract_data(req):
    username = req.username

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    try:
        # Correction : utilisation de la variable username dans la requête SQL
        cursor.execute('''SELECT id, mdp, poste FROM infos WHERE user = ?;''', (username,))
        data_extracted = cursor.fetchone()  # Stockage des données dans une variable

        if data_extracted:
            id, password, role = data_extracted  # Séparation des informations dans des variables distinctes

            rospy.loginfo("Données extraites avec succès")
            success = True
        else:
            rospy.logwarn(f"Aucune donnée trouvée pour l'utilisateur {username}")
            success = False
            id = None
            username=None
            role =None
            password=None

    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de l'extraction des données : {e}")
        success = False
        id = None
        username=None
        role =None
        password=None

    finally:
        conn.commit()
        conn.close()

    return login_memberResponse(success,id, username, password, role)  # Retour des résultats pour le service ROS



def listener():
    """Initialise le noeud ROS et les services."""
    rospy.init_node('nav_database', anonymous=True)


    # Définir le service et appel de la fonction de lecture de la databse
    rospy.Service('login_serv', login_member, extract_data)

    rospy.loginfo("Noeud nav_database démarré et en attente de messages...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noeud de naviagtion database arrêté.")