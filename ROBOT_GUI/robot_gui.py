import rospy
from std_msgs.msg import String, UInt16, Float32MultiArray

import sys
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QMainWindow, QTabWidget, QWidget, QFormLayout, QDialog, QVBoxLayout
from PyQt5.QtCore import Qt

import keyboard
from pyqtgraph import PlotWidget
import pyqtgraph as pq
import numpy as np

PI = 3.141592
D2R = PI/180 

class RobotApp(QDialog):
    def __init__(self):
        super(RobotApp,self).__init__()
        self.initUI()
        
    def initUI(self):
        tabs=QTabWidget()

        tabs.addTab(FirstTab(),'First')
        tabs.addTab(SecondTab(),'Second')
        tabs.addTab(ThirdTab(),'Third')
        
        vbox=QVBoxLayout()
        vbox.addWidget(tabs)

        self.setLayout(vbox)
        self.setWindowTitle("Robot GUI")
        self.setGeometry(600,600,600,600)

class FirstTab(QMainWindow):
    def __init__(self):
        super(FirstTab,self).__init__()
        uic.loadUi("FirstTab.ui",self)    
        self.initUI()
        
    def initUI(self):
        self.text_sub="None"
        self.pub1=rospy.Publisher('ROSGUI_PUB_MODE',UInt16,queue_size=1)
        self.sub=rospy.Subscriber('ROSGUI_SUB_MODE', UInt16, self.subCallback)
        self.R1.clicked.connect(self.publishMode)
        self.R2.clicked.connect(self.publishMode)
        self.R3.clicked.connect(self.publishMode)
        
        self.timer = pq.QtCore.QTimer()
        self.timer.timeout.connect(self.updateLE)
        self.timer.start(50)

    def updateLE(self):
        self.LE1.setText(self.text_sub)

    def publishMode(self):
        if self.R1.isChecked():
            MODE=1
        elif(self.R2.isChecked()):
            MODE=2
        elif(self.R3.isChecked()):
            MODE=3
        else:
            MODE=0
        self.pub1.publish(MODE)
        
    def subCallback(self,msg):
        if msg.data==0:
            self.text_sub="None" 
        elif msg.data==1:
            self.text_sub="Init Mode" 
        elif msg.data==2:
            self.text_sub="Ready Mode" 
        elif msg.data==3:
            self.text_sub="Slow Walk Mode" 
    
class SecondTab(QMainWindow):

    def __init__(self):
        super(SecondTab,self).__init__()
        uic.loadUi("SecondTab.ui",self)
        self.initUI()
        
    def initUI(self):
        self.vel_x=0
        self.vel_y=0
        self.vel_yaw=0

        self.pub2=rospy.Publisher('ROSGUI_PUB_JOY',Float32MultiArray,queue_size=1)

    def keyPressEvent(self, event):
            global R2D
            vel_x_max=0.3
            vel_y_max=0.3
            vel_yaw_max=20
    
            key=event.key()
            if(key==Qt.Key_8):
                if(self.vel_x<vel_x_max):
                    self.vel_x=self.vel_x+0.01
                    self.B1.setDown(True)
                    self.B2.setDown(False)
                    self.B7.setStyleSheet("background-color:none")
        
            if(key==Qt.Key_5):
                if(self.vel_x>-vel_x_max):
                    self.vel_x=self.vel_x-0.01
                    self.B1.setDown(False)
                    self.B2.setDown(True)
                    self.B7.setStyleSheet("background-color:none")
               
            if(key==Qt.Key_4):
                if(self.vel_y<vel_y_max):
                    self.vel_y=self.vel_y+0.01
                    self.B3.setDown(True)
                    self.B4.setDown(False)
                    self.B7.setStyleSheet("background-color:none")
            
            if(key==Qt.Key_6):
                if(self.vel_y>-vel_y_max):
                    self.vel_y=self.vel_y-0.01
                    self.B3.setDown(False)
                    self.B4.setDown(True)
                    self.B7.setStyleSheet("background-color:none")
        
            if(key==Qt.Key_7):
                if(self.vel_yaw<vel_yaw_max):
                    self.vel_yaw=self.vel_yaw+1.0
                    self.B5.setDown(True)
                    self.B6.setDown(False)
                    self.B7.setStyleSheet("background-color:none")
            
            if(key==Qt.Key_9):
                if(self.vel_yaw>-vel_yaw_max):
                    self.vel_yaw=self.vel_yaw-1.0
                    self.B5.setDown(False)
                    self.B6.setDown(True)
                    self.B7.setStyleSheet("background-color:none")
            
            if(key==Qt.Key_2):
                    self.vel_x=0;
                    self.vel_y=0;
                    self.vel_yaw=0;
                    self.B7.setStyleSheet("background-color:yellow")
                    
            if round(self.vel_x,2)>0.0 :
                self.B1.setStyleSheet("background-color:yellow")
                self.B2.setStyleSheet("background-color:none")
            elif round(self.vel_x,2)<0.0 :
                self.B1.setStyleSheet("background-color:none")
                self.B2.setStyleSheet("background-color:yellow")
            elif round(self.vel_x,2)==0.0:
                self.B1.setDown(False)
                self.B2.setDown(False)
                self.B1.setStyleSheet("background-color:none")
                self.B2.setStyleSheet("background-color:none")

            if round(self.vel_y,2)>0.0 :
                self.B3.setStyleSheet("background-color:yellow")
                self.B4.setStyleSheet("background-color:none")
            elif round(self.vel_y,2)<0.0 :
                self.B3.setStyleSheet("background-color:none")
                self.B4.setStyleSheet("background-color:yellow")
            elif round(self.vel_y,2)==0.0:
                self.B3.setDown(False)
                self.B4.setDown(False)
                self.B3.setStyleSheet("background-color:none")
                self.B4.setStyleSheet("background-color:none")

            if round(self.vel_yaw,2)>0.0 :
                self.B5.setStyleSheet("background-color:yellow")
                self.B6.setStyleSheet("background-color:none")
            elif round(self.vel_yaw,2)<0.0 :
                self.B5.setStyleSheet("background-color:none")
                self.B6.setStyleSheet("background-color:yellow")
            elif round(self.vel_yaw,2)==0.0 :
                self.B5.setDown(False)
                self.B6.setDown(False)
                self.B5.setStyleSheet("background-color:none")
                self.B6.setStyleSheet("background-color:none")

            Joy=Float32MultiArray()
            Joy.data=[self.vel_x,self.vel_y,self.vel_yaw*D2R]
            self.LE2.setText(str(round(self.vel_x,3)))
            self.LE3.setText(str(round(self.vel_y,3)))
            self.LE4.setText(str(round(self.vel_yaw,3)))
            self.pub2.publish(Joy)

class ThirdTab(QMainWindow):
    def __init__(self):
        super(ThirdTab,self).__init__()
        uic.loadUi("ThirdTab.ui",self)
        self.initUI()
        
    def initUI(self):
        self.pw=PlotWidget(self)
        # self.pw.setGeometry(QtCore.QRect(25,25,550,550))  
        # self.data=np.random.normal(size=300)
        
        # self.curve = self.pw.plot(self.data, name="mode2")
        # self.ptr1 = 0

        # self.timer = pq.QtCore.QTimer()
        # self.timer.timeout.connect(self.update_data)
        # self.timer.start(50)

    # def update_data(self):
        # self.data[:-1] = self.data[1:]
        # self.data[-1] = np.random.normal()
        # # Data is filled into the drawing curve

        # x=np.random.normal(size=100)
        # y=np.random.normal(size=100)

        # self.curve.setData(self.data)
        # # x axis record point
        # self.ptr1 += 1
        # # Reset x-related coordinate origin
        # self.curve.setPos(self.ptr1,0)

if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)
    rospy.init_node("ROSGUI_NODE", anonymous=False)
    Window=RobotApp()

    Window.show()
    
    sys.exit( app.exec_() )
