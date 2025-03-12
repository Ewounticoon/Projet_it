#!/usr/bin/env python3

from flask import Flask, render_template, request, redirect, url_for, jsonify
import sqlite3
import os
import rospy
from badge_rfid.srv import ajout_badge
from std_msgs.msg import Float32

app = Flask(__name__)

# ====================== #
#      SECTION ROS       #
# ====================== #

# Variables globales pour stocker les valeurs des capteurs
latest_temperature = 0.0
latest_humidity = 0.0
latest_volume = 0.0

# Callbacks ROS pour mettre à jour les valeurs
def temp_callback(msg):
    global latest_temperature
    latest_temperature = msg.data

def hum_callback(msg):
    global latest_humidity
    latest_humidity = msg.data

def volume_callback(msg):
    global latest_volume
    latest_volume = msg.data

# Initialisation de ROS dans le contexte de Flask
def init_ros():
    rospy.init_node('flask_node', anonymous=True)
    rospy.Subscriber('/topic_tempDHT11', Float32, temp_callback)
    rospy.Subscriber('/topic_humDHT11', Float32, hum_callback)
    rospy.Subscriber('/topic_micro', Float32, volume_callback)

# Service ROS pour ajouter un badge
def send_user_info(prenom, nom, age, email, mdp, job_title):
    rospy.wait_for_service('ajout_badge')
    try:
        ajout_badge_service = rospy.ServiceProxy('ajout_badge', ajout_badge)
        response = ajout_badge_service(prenom, nom, age, email, mdp, job_title)
        rospy.loginfo(f"Service response: success = {response.success}")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

# ============================ #
#   SECTION BASE DE DONNÉES    #
# ============================ #

DB_PATH = "/Projet_it/src/database/src"

def get_last_10_values(db_name, column_name):
    """ Récupère les 10 dernières valeurs d'une colonne d'une base SQLite """
    db_path = os.path.join(DB_PATH, db_name)
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    try:
        cursor.execute(f"SELECT {column_name} FROM {column_name} ORDER BY id DESC LIMIT 10;")
        results = cursor.fetchall()
        return [row[0] for row in results]
    except sqlite3.Error as e:
        print(f"Erreur SQLite : {e}")
        return []
    finally:
        conn.close()

# ========================== #
#      SECTION ROUTES        #
# ========================== #

@app.route('/')
def page_accueil():
    """ Page d'accueil principale """
    return render_template('page_accueil.html')

@app.route('/graph_capteurs', methods=['GET', 'POST'])
def graph_capteurs():
    """ Page affichant les graphiques des capteurs """
    return render_template('graph_capteurs.html')

@app.route('/placez_badge', methods=['GET', 'POST'])
def placez_badge():
    """ Page intermédiaire pour ajouter ou supprimer un badge """
    if request.method == 'POST':
        action = request.form.get('action')
        if action == "ajouter":
            return redirect(url_for('formulaire_badge'))
        elif action == "supprimer":
            return redirect(url_for('page_validation'))

    action = request.args.get('action')
    if not action:
        return "Erreur : aucune action spécifiée", 400  

    return render_template('placez_badge.html', action=action)

@app.route('/formulaire_badge')
def formulaire_badge():
    """ Formulaire pour ajouter un badge """
    return render_template('formulaire_badge.html')

@app.route('/page_validation')
def page_validation():
    """ Page de validation avant suppression d'un badge """
    return render_template('page_validation.html')

@app.route('/traitement', methods=['POST'])
def traitement():
    """ Traitement du formulaire d'ajout de badge """
    donnee = request.form
    prenom = donnee.get('prenom')
    nom = donnee.get('nom')
    age = donnee.get('age')
    email = donnee.get('email')
    mdp = donnee.get('mdp')
    job_title = donnee.get('job_title')

    print(prenom, nom, age, email, mdp, job_title)

    if age:
        age = int(age)

    send_user_info(prenom, nom, age, email, mdp, job_title)

    return "Traitement des données effectué", 200

# ================================ #
#     ROUTES POUR LES DONNÉES      #
# ================================ #

@app.route('/data')
def get_sensor_data():
    """ Renvoie les valeurs en direct des capteurs ROS """
    return jsonify({
        "temperature": latest_temperature,
        "humidity": latest_humidity,
        "volume": latest_volume
    })

@app.route('/get_data')
def get_database_data():
    """ Renvoie les 10 dernières valeurs des bases de données SQLite """
    temperature = get_last_10_values("dht11_temperature.db", "temperature")
    humidite = get_last_10_values("dht11_humidite.db", "humidite")
    volume = get_last_10_values("volumeMicro.db", "volume")

    return jsonify({
        "temperature": temperature,
        "humidite": humidite,
        "volume": volume
    })

# ======================== #
#     LANCEMENT FLASK      #
# ======================== #

if __name__ == '__main__':
    init_ros()  # Initialiser ROS avant Flask
    app.run(host="0.0.0.0", port=5000, debug=True)
