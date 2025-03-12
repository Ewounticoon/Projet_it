#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import random

class TestNodeDHT11:
    def __init__(self):
        self.pub_temp = rospy.Publisher('/topic_tempDHT11', Float32, queue_size=10)
        self.pub_hum = rospy.Publisher('/topic_humDHT11', Float32, queue_size=10)
        rospy.Timer(rospy.Duration(5), self.pub_donnees)
        rospy.loginfo("Simulation DHT11 en cours...")

    def pub_donnees(self, event):
        temperature = round(random.uniform(20, 30), 2)  # Température entre 20 et 30°C
        humidity = round(random.uniform(40, 60), 2)  # Humidité entre 40% et 60%
        
        rospy.loginfo(f"Température : {temperature}°C | Humidité : {humidity}%")
        self.pub_temp.publish(temperature)
        self.pub_hum.publish(humidity)

def main():
    rospy.init_node('test_node_dht11', anonymous=True)
    TestNodeDHT11()
    rospy.spin()

if __name__ == '__main__':
    main()
