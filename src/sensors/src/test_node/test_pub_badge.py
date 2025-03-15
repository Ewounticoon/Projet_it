#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import random
import time

class TestNodeRFID:
    def __init__(self):
        self.pub_rfid = rospy.Publisher('/topic_rfid', Int32, queue_size=10)
        rospy.loginfo("Simulation RFID en cours...")

    def simulate_rfid(self):
        while not rospy.is_shutdown():
            time.sleep(5)  # Simule l'attente d'un badge RFID
            rfid_id = random.randint(10000, 99999)  # Génère un ID aléatoire
            rospy.loginfo(f"Badge détecté : {rfid_id}")
            self.pub_rfid.publish(rfid_id)

def main():
    rospy.init_node('test_node_rfid', anonymous=True)
    rfid_tester = TestNodeRFID()
    rfid_tester.simulate_rfid()
    rospy.spin()

if __name__ == '__main__':
    main()