#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np

class CommandVelocity():

    def __init__(self):
        self.estado_anterior=' '
        rospy.loginfo("Starting node")
        rospy.on_shutdown(self.safety_stop)
        self.pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist)
        rospy.Subscriber('/scan', LaserScan, self.send_velocities)
        rospy.spin()

    def send_velocities(self, data):
        #rospy.loginfo("reading LiDAR")
        datos_laser = np.asarray(data.ranges)
        #### AQUÍ EMPIEZA SU CÓDIGO
        self.angle_min=(data.angle_min*180)/np.pi
        self.angle_max=(data.angle_max*180)/np.pi
        self.ang_inc = (self.angle_max-self.angle_min)/len(data.ranges)
        #self.pared()
        self.atras()
        self.atras_derecha()
        #self.esquivar()
        self.avanzar()
        #rospy.loginfo(datos_laser)

        S1 = datos_laser[0:213]
        S2 = datos_laser[213:426]
        S3 = datos_laser[426:640]

        SS1 = np.nanmin(S1)
        SS2 = np.nanmin(S2)
        SS3 = np.nanmin(S3)

        distancias = str(SS1)+"\t"+str(SS2)+"\t"+str(SS3)

        if SS1 < 1 or SS2 < 1 or SS3 < 1:
            self.parar()
            for i in range(10):
                datos_laser = np.asarray(data.ranges)
                S1 = datos_laser[0:213]
                S2 = datos_laser[213:426]
                S3 = datos_laser[426:640]

                SS1 = np.nanmin(S1)
                SS2 = np.nanmin(S2)
                SS3 = np.nanmin(S3)
                self.girar_derecha()

                if SS1 >= 1 and SS2 >= 1 and SS3 >= 1:
                    self.avanzar()
                    break

                elif SS1 < 1:
                    self.girar_derecha()
                    break

                elif SS3 < 1:
                    self.girar_derecha()
                    break

                elif SS2 == np.isnan(SS2):
                    self.avanzar()
                    break

                else:
                    self.avanzar()
                    break

                time.sleep(0.1)
            rospy.loginfo(distancias)

        else:
            self.avanzar()


#        twist_msg = Twist()
#        for rango in datos_laser:
#            if rango < 1:
#                twist_msg.linear.x=-0.1
#            else:
#                twist_msg.linear.x=0.3
#                twist_msg.linear.y=0.1
#                twist_msg.angular.z=-0.2
#        self.pub.publish(twist_msg)

    def avanzar(self):
        #rospy.loginfo("reading LiDAR22")
        twist_msg = Twist()
        twist_msg.linear.x=0.3
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=0.0
        self.pub.publish(twist_msg)

    def girar_izquierda(self):
        twist_msg = Twist()
        twist_msg.linear.x=0.0
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=1.0
        self.pub.publish(twist_msg)

    def girar_derecha(self):
        twist_msg = Twist()
        twist_msg.linear.x=0.0
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=-1.0
        self.pub.publish(twist_msg)

    def atras(self):
        twist_msg = Twist()
        twist_msg.linear.x=0.1
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=3.0
        self.pub.publish(twist_msg)

    def pared(self):
        twist_msg = Twist()
        twist_msg.linear.x=-5.0
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=0.0
        self.pub.publish(twist_msg)

    def atras_derecha(self):
        twist_msg = Twist()
        twist_msg.linear.x=-2.0
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=3.0
        self.pub.publish(twist_msg)

    def atras_izquierda(self):
        twist_msg = Twist()
        twist_msg.linear.x=-2.0
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=-3.0
        self.pub.publish(twist_msg)

    def parar(self):
        twist_msg = Twist()
        twist_msg.linear.x=0.0
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=0.0
        self.pub.publish(twist_msg)

    def safety_stop(self):
        for i in range(0,1):
            self.parar()
            rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
