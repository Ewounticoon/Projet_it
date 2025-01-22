#!/usr/bin/env python3
import rospy
from badge_rfid.srv import ajout_badge

def send_user_info(first_name, last_name, age, email, password, job_title):
    rospy.wait_for_service('user_info_service')
    try:
        user_info_service = rospy.ServiceProxy('user_info_service', ajout_badge)
        response = user_info_service(first_name, last_name, age, email, password, job_title)
        rospy.loginfo(f"Service response: success = {response.success}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('user_info_client')
    send_user_info("John", "Doe", 30, "john.doe@example.com", "password123", "Software Engineer")
    rospy.spin()

if __name__ == '__main__':
    main()
