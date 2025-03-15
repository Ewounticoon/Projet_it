#!/usr/bin/env python3

from flask import Flask, render_template, request, redirect, url_for, jsonify, flash
from flask_login import LoginManager, UserMixin, login_user, logout_user, login_required, current_user
import sqlite3
import os
import rospy
from badge_rfid.srv import ajout_badge, suppr_badge, login_member
from std_msgs.msg import Float32
from sensors.msg import dht11
from werkzeug.security import check_password_hash
import rospkg

#Chemin vers database
rospack = rospkg.RosPack()
package_path = rospack.get_path('database')


app = Flask(__name__)
app.secret_key = 'Enpiwen_mdp'

# ====================== #
#         LOGIN          #
# ====================== #

login_manager = LoginManager()
login_manager.init_app(app)
login_manager.login_view = 'login'

class User(UserMixin):
    def __init__(self, id, username, password, role):
        self.id = str(id)  # Assure que l'ID est bien une chaîne
        self.username = username
        self.password = password
        self.role = role

    def get_id(self):
        return str(self.id)  # Retourne une chaîne pour éviter les erreurs



# Dictionnaire temporaire pour stocker les utilisateurs en mémoire
users_cache = {}

@login_manager.user_loader
def load_user(user_id):
    if user_id in users_cache:
        return users_cache[user_id]  # Retourne l'utilisateur en cache sans appeler ROS
    
    rospy.loginfo(f"Trying to load user with ID: {user_id}")
    
    rospy.wait_for_service('login_serv')
    try:
        login_service = rospy.ServiceProxy('login_serv', login_member)
        response = login_service(user_id)

        if response.success:
            user = User(id=response.id, username=response.username, password=response.password, role=response.role)
            users_cache[user_id] = user  # Stocke l'utilisateur en cache
            return user
        else:
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None


    

def authenticate(username, password):
    """ Vérifie si l'utilisateur existe dans la base de données et si le mot de passe est correct """
    user = load_user(username)  # Récupère l'objet User
    
    if user:
        if check_password_hash(user.password, password):  # Vérifie le mot de passe haché
            # Si le mot de passe est correct, renvoie l'objet utilisateur
            return user
        else:
            return None  # Mot de passe incorrect
    else:
        return None  # Utilisateur non trouvé


# ====================== #
#      SECTION ROS       #
# ====================== #


# Initialisation de ROS dans le contexte de Flask
def init_ros():
    rospy.init_node('flask_node', anonymous=True)

# Service ROS pour ajouter un badge
def del_badge_serv():
    try:
        rospy.wait_for_service('del_badge', timeout=2)  # Timeout de 2 secondes
    except rospy.ROSException:
        rospy.logerr("Timeout : Service 'del_badge' non disponible.")
        return False

    rospy.loginfo("Request Del badge")
    try:
        del_badge_service = rospy.ServiceProxy('del_badge', suppr_badge)
        response = del_badge_service(True)
        rospy.loginfo(f"Service response: success = {response.validation}")
        return response.validation
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False



# Service ROS pour ajouter un badge
def send_user_info(prenom, nom, username, age, email, mdp, job_title):
    rospy.wait_for_service('ajout_badge')
    try:
        ajout_badge_service = rospy.ServiceProxy('ajout_badge', ajout_badge)
        response = ajout_badge_service(prenom, nom, username, age, email, mdp, job_title)
        rospy.loginfo(f"Service response: success = {response.success}")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

# ============================ #
#   SECTION BASE DE DONNÉES    #
# ============================ #
DB_PATH=os.path.join(package_path, 'database') #chemin d'acces

def get_last_10_values(db_name,table_name, column_name):
    """ Récupère les 10 dernières valeurs d'une colonne d'une base SQLite """
    db_path = os.path.join(DB_PATH, db_name)
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    try:
        cursor.execute(f"SELECT {column_name} FROM {table_name} ORDER BY id DESC LIMIT 10;")
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
@app.route('/', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form['username']
        password = request.form['password']
        user = authenticate(username, password)

        if user:
            login_user(user)
            users_cache[user.id] = user  # Stocke l'utilisateur pour éviter d'appeler ROS à chaque requête
            
            print(f"Utilisateur connecté ? {current_user.is_authenticated}")

            return redirect(url_for('page_accueil'))
        else:
            flash('Nom d\'utilisateur ou mot de passe incorrect', 'danger')

    return render_template('login.html')



@app.route('/page_accueil')
@login_required

def page_accueil():
    """ Page d'accueil principale """
    return render_template('page_accueil.html')

@app.route('/graph_capteurs', methods=['GET', 'POST'])
@login_required
def graph_capteurs():
    """ Page affichant les graphiques des capteurs """
    return render_template('graph_capteurs.html')

@app.route('/placez_badge', methods=['GET', 'POST'])
@login_required

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

@app.route('/succes_enregistrement', methods=['GET', 'POST'])
def succes_enregistrement():
    """ Page après avoir rempli le formulaire avec succès """
    return render_template('succes_enregistrement.html')

@app.route('/page_validation')
@login_required
def page_validation():
    """ Page de validation avant suppression d'un badge """
    del_badge_serv()
    return render_template('page_validation.html')

@app.route('/traitement', methods=['POST'])
@login_required
def traitement():
    """ Traitement du formulaire d'ajout de badge """
    donnee = request.form
    prenom = donnee.get('prenom')
    nom = donnee.get('nom')
    username = donnee.get('username')
    age = donnee.get('age')
    email = donnee.get('email')
    mdp = donnee.get('mdp')
    job_title = donnee.get('job_title')

    print(prenom, nom, username, age, email, mdp, job_title)

    if age:
        age = int(age)

    send_user_info(prenom, nom, username, age, email, mdp, job_title)

    return "Traitement des données effectué", 200

# ================================ #
#     ROUTES POUR LES DONNÉES      #
# ================================ #


@app.route('/data') # On triche tkt
@login_required
def get_database_data():
    """ Renvoie les 10 dernières valeurs des bases de données SQLite """
    rospy.loginfo("Recherche dans database")

    temperature = get_last_10_values("dht11_temperature.db", "temperature","temperature")
    humidite = get_last_10_values("dht11_humidite.db","humidite", "humidite")
    volume = get_last_10_values("volumeMicro.db", "son","volSon")
    return jsonify({
        "temperature": temperature,
        "humidity": humidite,
        "volume": volume
    })


@app.route('/logout')
@login_required
def logout():
    logout_user()
    flash('Vous avez ete deconnecte')
    return redirect(url_for('login'))

# ======================== #
#     LANCEMENT FLASK      #
# ======================== #

if __name__ == '__main__':
    init_ros()  # Initialiser ROS avant Flask
    app.run(host="0.0.0.0", port=5000, debug=True)