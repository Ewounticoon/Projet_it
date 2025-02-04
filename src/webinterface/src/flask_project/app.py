#!/usr/bin/env python3

from flask import Flask, render_template, request, redirect, url_for
import rospy
from badge_rfid.srv import ajout_badge

app = Flask(__name__)

# Fonction pour envoyer les informations au service ROS
def send_user_info(prenom, nom, age, email, mdp, job_title):
    rospy.wait_for_service('ajout_badge')
    try:
        ajout_badge = rospy.ServiceProxy('ajout_badge', ajout_badge)
        response = ajout_badge(prenom, nom, age, email, mdp, job_title)
        rospy.loginfo(f"Service response: success = {response.success}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# Initialisation de ROS dans le contexte de Flask
def init_ros():
    rospy.init_node('ajout_badge', anonymous=True)

@app.route('/')
def formulaire_badge():
    return render_template('formulaire_badge.html')
'''

@app.route('/')
def graph_capteurs():
    return render_template('graph_capteurs.html')'''

@app.route('/traitement', methods=['POST'])
def traitement():
    donnee = request.form
    prenom = donnee.get('prenom')
    nom = donnee.get('nom')
    age = donnee.get('age')# Assurez-vous que ces champs existent dans le formulaire
    email = donnee.get('email')
    mdp = donnee.get('mdp')
    job_title = donnee.get('job_title')  # Assurez-vous d'utiliser "job_title" pour correspondre à l'attribut HTML
    
    # Afficher les valeurs reçues
    print(prenom, nom, age, email, mdp, job_title)

    # Appeler la fonction d'envoi des données au service ROS
    send_user_info(prenom, nom, int(age), email, mdp, job_title)  # Remplace "password123" par le mot de passe réel

    return "Traitement des données effectué", 200

if __name__ == '__main__':
    init_ros()  # Initialiser ROS avant de lancer Flask
    app.run(host="0.0.0.0", port=5000, debug=True)
