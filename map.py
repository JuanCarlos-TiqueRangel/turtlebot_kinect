import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from matplotlib import pyplot as plt
from sensor_msgs.msg import PointCloud2
from mpl_toolkits import mplot3d
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes
import time, os, sys
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import matplotlib.pyplot as plt



class Map(object): # Crea clase

    def __init__(self):
        self.estado_anterior=' '
        rospy.loginfo("Starting node")
        global xi, yi, Yfv, Xfv, Ts
        Ts = 0.001
        tx = 0
        ty = 0
        phi = 0
        Yfv = []
        Xfv = []
        self.phi=0
        self.tx = 0
        self.ty =0
        self.Xf = []
        self.Yf = []
        self.Zf = []
        self.Yfv=[]
        self.Xfv=[]
        self.Zfv=[]
        self.Xfv_array=[]
        self.Yfv_array=[]
        self.Zfv_array=[]
        self.xg=[]
        self.yg=[]
        self.zg=[]
        self.counter=0
        self.X_array=[]
        self.Y_array=[]
        self.Z_array=[]

        #rospy.Subscriber('/scan', LaserScan, self.scan)
        rospy.Subscriber('/pos', Twist, self.distancia)
        rospy.Subscriber('/joy', Joy, self.plot)
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.enviaKinect)
        #rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.anguloPHI)
        rospy.spin()


    def distancia(self,data):
        self.tx = data.linear.x
        self.ty = data.linear.y
        self.phi = data.linear.z

    def enviaKinect(self,data):
        #datos_kinect = np.asarray(data.height)
        #print "hola"
        pc = pc2.read_points(data,skip_nans=True)
        int_data=list(pc)
        M=[]
        #self.counter=0

        for P in range(0,len(int_data),100):
            self.x=int_data[P][0]
            self.z=-int_data[P][1]
            self.y=int_data[P][2]
            s=struct.pack('>f',int_data[P][3])
            i=struct.unpack('>l',s)[0]
            pack = ctypes.c_uint32(i).value
            R=(pack & 0x00FF0000)>> 16
            G=(pack & 0x0000FF00)>> 8
            B=(pack & 0x000000FF)

            self.xg.append(self.x)
            self.yg.append(self.y)
            self.zg.append(self.z)

            del self.x,self.y,self.z,R,G,B
            self.counter = self.counter+1
            #if self.counter==20000:
            #    break


        self.X_array = np.array(self.xg)
        self.Y_array = np.array(self.yg)
        self.Z_array = np.array(self.zg)

        self.xg=[]
        self.yg=[]
        self.zg=[]

        print "--------------------------------------------------------------------------------------------"

        T = np.array([ [1, 0, 0, self.tx], [0, 1, 0, self.ty], [0, 0, 1, 0],[0, 0, 0, 1]])
        Xi =np.ones([4,len(self.X_array)])

        Xi[0,:]=  self.X_array
        Xi[1,:]=  self.Y_array
        Xi[2,:]=  self.Z_array

        Xa = np.array([[np.cos(self.phi),(np.sin(self.phi))*-1 ,0,0], [np.sin(self.phi), np.cos(self.phi), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Xo = np.matmul(T,Xi)
        F = np.matmul(Xa,Xo)
        NN = len(self.X_array)
        self.Xf= np.zeros([1,NN])
        self.Yf= np.zeros([1,NN])
        self.Zf= np.zeros([1,NN])
        self.Xf[0,:] = F[0,:]
        self.Yf[0,:] = F[1,:]          ## va el menos 1 stylozo
        self.Zf[0,:] = F[2,:]

        # print self.X_array

    def plot(self,data):
        buttons = data.buttons
        if(buttons[4] == 1):
          print self.Xf.shape
          self.Xfv = self.Xf[0,:]
          self.Yfv = self.Yf[0,:]
          self.Zfv = self.Zf[0,:]
          print self.Xfv.shape
          self.Xfv_array=np.concatenate((self.Xfv_array,self.Xfv),axis=0)
          self.Yfv_array=np.concatenate((self.Yfv_array,self.Yfv),axis=0)
          self.Zfv_array=np.concatenate((self.Zfv_array,self.Zfv),axis=0)
          print self.Xfv_array.shape
          print self.Yfv_array.shape
          print self.Zfv_array.shape

        if(buttons[5] == 1 and len(self.Xfv) == len(self.Yfv)):

            ax = plt.axes(projection='3d')

            x=self.Xfv_array
            y=self.Yfv_array
            z=self.Zfv_array
            #print M2.shape, cmap='jet',s=0.1
            plt.figure(1)
            ax.scatter3D(x,y,z,c=z,cmap='jet')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            ax.azim = -100
            ax.elev = 45

            plt.savefig('3D')
            plt.show()

            plt.figure(2)
            plt.scatter(x,y,c =z , cmap = 'jet')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.savefig('xy')

            plt.figure(3)
            plt.plot(x,z,'.')
            plt.xlabel('X')
            plt.ylabel('Z')
            plt.savefig('xz')

            plt.figure(4)
            plt.plot(y,z,'.')
            plt.xlabel('Y')
            plt.ylabel('Z')
            plt.savefig('yz')

if __name__ == '__main__':
    rospy.init_node("Map")
    cv = Map()
