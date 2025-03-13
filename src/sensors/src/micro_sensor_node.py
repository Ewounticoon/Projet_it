import rospy
from std_msgs.msg import Float32
from pyfirmata import Arduino, util
import time
import math

class Node_micro:
    def __init__(self):
        # Définir le port série de l'Arduino
        self.board = Arduino('/dev/ttyUSB0')  # Remplacez '/dev/ttyUSB0' par le port approprié
        it = util.Iterator(self.board)
        it.start()

        self.board.analog[0].enable_reporting()
        
        # Initialisation des publisher (nom du topic, type, taille)
        self.topic_micro = rospy.Publisher('/topic_micro', Float32, queue_size=10)

        # Lancer une fonction à intervalle régulier
        rospy.Timer(rospy.Duration(0.5), self.pub_donne_micro)
        
        # Ecrit dans le terminal
        rospy.loginfo("Publication micro")


    def voltage_to_db(self, voltage, v_ref=1.0):
        if voltage > 0 :
            return 20*math.log10(voltage/v_ref)
        else :
            return -float('inf')
    # Publier sur un topic
    def pub_donne_micro(self, event):
        analog_value = self.board.analog[0].read()  # Lire la valeur analogique de la broche A0
    
        if analog_value is None:
            rospy.logwarn("Valeur analogique None, aucune publication")  # ✅ Ajout d'un warning
            return  # ✅ Arrête l'exécution pour éviter une publication
    
        # Conversion de la valeur analogique en tension (de 0 à 5V)
        voltage = analog_value * 5.0
        db_value = self.voltage_to_db(voltage, 1)
    
        msg = Float32()
        msg.data = db_value
        self.topic_micro.publish(msg)  # ✅ Publie seulement si une valeur est valide
    

# Démarrer le noeud
def main():
    # Initialise le noeud
    rospy.init_node('node_micro')
    serial_node = Node_micro()
    # Lance le noeud
    rospy.spin()

if __name__ == '__main__':
    main()
