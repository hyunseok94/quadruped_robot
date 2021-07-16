#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt16, Float32MultiArray

from PyQt5 import QtGui, QtWidgets, uic, QtCore
import sys
from PyQt5.QtWidgets import QMainWindow, QTabWidget, QWidget, QFormLayout
import keyboard
from PyQt5.QtCore import Qt

from pyqtgraph import PlotWidget
import pyqtgraph as pq


PI = 3.141592
D2R = PI/180 

class MainWindow(QMainWindow):
    __Initialized=False
    def __init__(self):
        global pub1, pub2
        global text_sub

        super(MainWindow,self).__init__()
        uic.loadUi("main_window.ui",self)
        self.setWindowTitle("Robot GUI")
        
        self.tab1=QWidget()
        self.tab2=QWidget()

        self.tabs=QTabWidget()
        self.tabs.addTab(self.tab1,'Tab1')
        self.tabs.addTab(self.tab2,'Tab2')


        self.tab1UI()

        # self.plotWidget=PlotWidget(self)
        # self.tabwidget.addTab(self.plotWidget,"Tab 2")
        # self.plotWidget.setGeometry(QtCore.QRect(25,25,550,550))
        # self.tab1=QWidget()
        # self.tab2=QWidget()
        # self.tabs.addTab(self.tab1,"Tab 1")
        # self.tabs.addTab(self.tab2,"Tab 2")


        # Plot
        #self.plotWidget=PlotWidget(self)
        # self.tab2.layout.addWidget(PlotWidget(self))
        

        rospy.init_node("ROSGUI_NODE", anonymous=True)
        
        self.R1.clicked.connect(self.publishMode)
        self.R2.clicked.connect(self.publishMode)
        self.R3.clicked.connect(self.publishMode)
        self.B1.setCheckable(True)

        rospy.Subscriber('ROSGUI_SUB_MODE', UInt16, self.subCallback)
        pub1=rospy.Publisher('ROSGUI_PUB_MODE',UInt16,queue_size=1)
        pub2=rospy.Publisher('ROSGUI_PUB_JOY',Float32MultiArray,queue_size=1)
    def tab1UI(self):
        layout=QFormLayout()

        self.tab1.setLayout(layout)  

    def subCallback(self,msg):
        if msg.data==0:
         text_sub="None" 
        elif msg.data==1:
         text_sub="Init Mode" 
        elif msg.data==2:
         text_sub="Ready Mode" 
        elif msg.data==3:
         text_sub="Slow Walk Mode" 
        self.LE1.setText(text_sub)   

    def publishMode(self):
        if self.R1.isChecked():
            MODE=1
        elif(self.R2.isChecked()):
            MODE=2
        elif(self.R3.isChecked()):
            MODE=3
        else:
            MODE=0
        pub1.publish(MODE)

    def keyPressEvent(self, event):
        global vel_x,vel_y,vel_yaw
        global R2D
        vel_x_max=0.3
        vel_y_max=0.3
        vel_yaw_max=20

        if MainWindow.__Initialized==False:
            vel_x=0
            vel_y=0
            vel_yaw=0
            MainWindow.__Initialized=True
        
        key=event.key()
        if(key==Qt.Key_8):
            if(vel_x<vel_x_max):
                vel_x=vel_x+0.01
                self.B1.setDown(True)
                self.B2.setDown(False)
                self.B7.setStyleSheet("background-color:none")
    
        if(key==Qt.Key_5):
            if(vel_x>-vel_x_max):
                vel_x=vel_x-0.01
                self.B1.setDown(False)
                self.B2.setDown(True)
                self.B7.setStyleSheet("background-color:none")
           
        if(key==Qt.Key_4):
            if(vel_y<vel_y_max):
                vel_y=vel_y+0.01
                self.B3.setDown(True)
                self.B4.setDown(False)
                self.B7.setStyleSheet("background-color:none")
        
        if(key==Qt.Key_6):
            if(vel_y>-vel_y_max):
                vel_y=vel_y-0.01
                self.B3.setDown(False)
                self.B4.setDown(True)
                self.B7.setStyleSheet("background-color:none")
    
        if(key==Qt.Key_7):
            if(vel_yaw<vel_yaw_max):
                vel_yaw=vel_yaw+1.0
                self.B5.setDown(True)
                self.B6.setDown(False)
                self.B7.setStyleSheet("background-color:none")
        
        if(key==Qt.Key_9):
            if(vel_yaw>-vel_yaw_max):
                vel_yaw=vel_yaw-1.0
                self.B5.setDown(False)
                self.B6.setDown(True)
                self.B7.setStyleSheet("background-color:none")
        
        if(key==Qt.Key_2):
                vel_x=0;
                vel_y=0;
                vel_yaw=0;
                self.B7.setStyleSheet("background-color:yellow")
                
        if round(vel_x,2)>0.0 :
            self.B1.setStyleSheet("background-color:yellow")
            self.B2.setStyleSheet("background-color:none")
        elif round(vel_x,2)<0.0 :
            self.B1.setStyleSheet("background-color:none")
            self.B2.setStyleSheet("background-color:yellow")
        elif round(vel_x,2)==0.0:
            self.B1.setDown(False)
            self.B2.setDown(False)
            self.B1.setStyleSheet("background-color:none")
            self.B2.setStyleSheet("background-color:none")

        if round(vel_y,2)>0.0 :
            self.B3.setStyleSheet("background-color:yellow")
            self.B4.setStyleSheet("background-color:none")
        elif round(vel_y,2)<0.0 :
            self.B3.setStyleSheet("background-color:none")
            self.B4.setStyleSheet("background-color:yellow")
        elif round(vel_y,2)==0.0:
            self.B3.setDown(False)
            self.B4.setDown(False)
            self.B3.setStyleSheet("background-color:none")
            self.B4.setStyleSheet("background-color:none")

        if round(vel_yaw,2)>0.0 :
            self.B5.setStyleSheet("background-color:yellow")
            self.B6.setStyleSheet("background-color:none")
        elif round(vel_yaw,2)<0.0 :
            self.B5.setStyleSheet("background-color:none")
            self.B6.setStyleSheet("background-color:yellow")
        elif round(vel_yaw,2)==0.0 :
            self.B5.setDown(False)
            self.B6.setDown(False)
            self.B5.setStyleSheet("background-color:none")
            self.B6.setStyleSheet("background-color:none")

        Joy=Float32MultiArray()
        Joy.data=[vel_x,vel_y,vel_yaw*D2R]
        self.LE2.setText(str(round(vel_x,3)))
        self.LE3.setText(str(round(vel_y,3)))
        self.LE4.setText(str(round(vel_yaw,3)))
        pub2.publish(Joy)

if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)
    Window=MainWindow()

    Window.show()
    sys.exit( app.exec_() )
