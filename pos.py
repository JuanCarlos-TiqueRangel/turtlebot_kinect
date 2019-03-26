import rospy
from nav_msgs.msg import Odometry #Para el laserScan.
import numpy as np
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32

msg = Twist()

class posicion(): # Crea clase

    def __init__(self):
        self.phi=0    #una variable con memoria de theta
        self.x=0
        self.y=0
        self.ref=0
        self.error=0
        global Ts, yaw
        Ts = 0.001
        self.yaw=0
        self.Z1 = 0
        self.estado_anterior=' '
        rospy.loginfo("Starting node")
        rospy.Subscriber('/odom', Odometry, self.enviaPos)
        rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.enviatheta)
        self.pub = rospy.Publisher('/pos', Twist, queue_size=10)
        rospy.spin()

    def enviatheta (self,data):
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        euler = euler_from_quaternion (orientation_list)
        self.yaw = euler[2]

        self.phi = self.yaw+Ts*self.yaw
        Z1 = self.phi
        #print self.phi
        msg.linear.z = Z1

    def enviaPos(self,data):
        x = (data.twist.twist.linear.x)
        #W = (data.twist.twist.angular.z)
        #self.phi=self.phi+Ts*W
        vx=x*np.cos(self.phi)
        vy=x*np.sin(self.phi)

        self.x=self.x+vx*Ts
        self.y=self.y+vy*Ts

        X1 = self.x
        Y1 = self.y
        Z1 = self.phi

        msg.linear.x = X1
        msg.linear.y = Y1
        self.pub.publish(msg)

        #print str(self.phi)+"\t"+str(self.x)+"\t"+str(self.y)

if __name__ == '__main__':
    rospy.init_node("posicion")
    cv = posicion()
