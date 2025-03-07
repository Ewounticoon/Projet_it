#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sqlite3
import os
from datetime import datetime
from badge_rfid.srv import ajout_badge, ajout_badgeResponse
import bcrypt

# Chemin de la base de données
db_path = os.path.expanduser('~/ros_workspace/src/database/src/RFID_infos.db')

# Variable globale pour stocker le numéro de badge
global_badge = None

def create_database_infos():
    """Création de la table pour les infos du badge."""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    try:
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS infos (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                numBadge INTEGER NOT NULL,
                prenom TEXT NOT NULL,
                nom TEXT NOT NULL,
                age INTEGER NOT NULL,
                mail TEXT NOT NULL,
                mdp TEXT NOT NULL,
                poste TEXT NOT NULL
            )
        ''')
        rospy.loginfo("Création de la table pour les infos du badge")  # DEBUG
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de la création de la table : {e}")
    finally:
        conn.commit()
        conn.close()

def lecture_badge(msg):
    """Callback pour lire les messages du topic."""
    global global_badge
    global_badge = msg.data
    rospy.loginfo(f"Numéro de badge reçu : {global_badge}")

def ajout_badge_base(req):
    """Ajoute les informations du badge dans la base de données."""
    global global_badge
    if global_badge is None:
        rospy.logerr("Aucun badge détecté !")
        return ajout_badgeResponse(False)

    rospy.loginfo(f"Infos utilisateur reçues : {req.prenom} {req.nom}, Âge: {req.age}, mail: {req.mail}, Poste: {req.poste}")

    # Hachage sécurisé du mot de passe
    hashed_password = bcrypt.hashpw(req.password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')

    # Enregistrement dans la base de données
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    try:
        cursor.execute('''
            INSERT INTO infos (numBadge, prenom, nom, age, mail, mdp, poste)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (global_badge, req.prenom, req.nom, req.age, req.mail, req.password, req.poste))
        rospy.loginfo(f"Badge {global_badge} enregistré avec succès")
        success = True
    except sqlite3.Error as e:
        rospy.logerr(f"Erreur lors de l'insertion des données : {e}")
        success = False
    finally:
        conn.commit()
        conn.close()

    return ajout_badgeResponse(success)

def listener():
    """Initialise le noeud ROS et les services."""
    rospy.init_node('ajout_badge', anonymous=True)

    # Créer la base de données si elle n'existe pas
    create_database_infos()

    # Souscrire au topic
    rospy.Subscriber("topic_rfid", Int32, lecture_badge)

    # Définir le service
    rospy.Service('ajout_badge', ajout_badge, ajout_badge_base)

    rospy.loginfo("Noeud ajout_badge démarré et en attente de messages...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noeud ajout_badge arrêté.")