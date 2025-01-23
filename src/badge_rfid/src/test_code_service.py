#!/usr/bin/env python3
import rospy
from badge_rfid.srv import ajout_badge

def send_user_info(first_name, last_name, age, mail, password, job_title):
    """Envoie les informations utilisateur au service ROS."""
    rospy.wait_for_service('ajout_badge')
    try:
        # Initialisation de la connexion au service
        user_info_service = rospy.ServiceProxy('ajout_badge', ajout_badge)
        
        # Appel du service
        response = user_info_service(first_name, last_name, age, mail, password, job_title)
        
        # Affichage de la réponse
        if response.success:
            rospy.loginfo("Les informations ont été enregistrées avec succès.")
        else:
            rospy.logwarn("L'enregistrement des informations a échoué.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Erreur lors de l'appel au service : {e}")

def main():
    """Noeud principal pour envoyer les informations utilisateur."""
    rospy.init_node('user_info_client')

    # Informations utilisateur à envoyer
    user_data = {
        "first_name": "John",
        "last_name": "Doe",
        "age": 30,
        "mail": "john.doe@example.com",
        "password": "password123",
        "job_title": "Software Engineer"
    }

    # Vérification des données
    if not isinstance(user_data['age'], int):
        rospy.logerr("L'âge doit être un entier.")
        return

    # Envoi des informations
    send_user_info(
        user_data["first_name"],
        user_data["last_name"],
        user_data["age"],
        user_data["mail"],
        user_data["password"],
        user_data["job_title"]
    )

if __name__ == '__main__':
    main()
