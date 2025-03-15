#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import random

class TestNodeMicro:
    def __init__(self):
        self.pub_micro = rospy.Publisher('/topic_micro', Float32, queue_size=10)
        rospy.Timer(rospy.Duration(0.5), self.pub_donnees)
        rospy.loginfo("Simulation Microphone en cours...")

    def pub_donnees(self, event):
        db_value = round(random.uniform(30, 80), 2)  # Niveau sonore entre 30 et 80 dB
        rospy.loginfo(f"Niveau sonore : {db_value} dB")
        self.pub_micro.publish(db_value)

def main():
    rospy.init_node('test_node_micro', anonymous=True)
    TestNodeMicro()
    rospy.spin()

if __name__ == '__main__':
    main()
