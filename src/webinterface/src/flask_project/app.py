#!/usr/bin/env python

from flask import Flask, render_template, request, redirect, url_for
import mysql.connector
import rospy
from my_package.srv import UserInfo

app = Flask(__name__)

def send_user_info(first_name, last_name, age, email, password, job_title):
    rospy.wait_for_service('user_info_service')
    try:
        user_info_service = rospy.ServiceProxy('user_info_service', UserInfo)
        response = user_info_service(first_name, last_name, age, email, password, job_title)
        rospy.loginfo(f"Service response: success = {response.success}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('user_info_client')
    send_user_info("John", "Doe", 30, "john.doe@example.com", "password123", "Software Engineer")


# Route formulaire_badge

@app.route('/')

def formulaire_badge():

    return render_template('formulaire_badge.html')

@app.route('/traitement', methods=['POST'])
def traitement():
    donnee = request.form
    nom = donnee.get('nom')
    mdp = donnee.get('mdp') 
    print(nom, mdp)
    return "Traitement de données", 200

if __name__ == '__main__':
    app.run(debug=True)
