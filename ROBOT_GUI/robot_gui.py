import rospy
from std_msgs.msg import String, UInt16, Float32MultiArray, Bool

import sys
from PyQt5 import QtGui, QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QMainWindow, QTabWidget, QWidget, QFormLayout, QDialog, QVBoxLayout
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon, QIntValidator

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
        self.setWindowIcon(QIcon("icon/controller.png"))
        self.setGeometry(600,600,1000,800)
        

class FirstTab(QMainWindow):
    def __init__(self):
        super(FirstTab,self).__init__()
        uic.loadUi("FirstTab.ui",self)    
        self.initUI()
        
    def initUI(self):
        self.text_sub="None"
        self.Save_flag=0;

        self.pub_mode=rospy.Publisher('ROSGUI_PUB_MODE',UInt16,queue_size=1)
        self.pub_save=rospy.Publisher('ROSGUI_PUB_SAVE',UInt16,queue_size=1)
        self.sub=rospy.Subscriber('ROSGUI_SUB_MODE', UInt16, self.subCallback)
        self.R1.clicked.connect(self.publishMode)
        self.R2.clicked.connect(self.publishMode)
        self.R3.clicked.connect(self.publishMode)
        self.button_save.clicked.connect(self.publishSave)

        self.timer = pq.QtCore.QTimer()
        self.timer.timeout.connect(self.updateLE)
        self.timer.start(50)
        self.button_exit.clicked.connect(QtWidgets.qApp.quit)

        # self.button_exit.clicked.connect(self.close)
        # self.button_exit.clicked.connect(sys.exit())

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
        self.pub_mode.publish(MODE)
    
    def publishSave(self):
        self.Save_flag=1;
        self.pub_save.publish(self.Save_flag)

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
        self.MaxPitch=30.0
        self.MaxRoll=30.0
        self.MaxCoMx=0.05
        self.MaxCoMy=0.05
        self.MaxCoMz=0.05
        self.Maxheight=0.15
        self.pitch_ref=0
        self.roll_ref=0
        self.com_x_ref=0
        self.com_y_ref=0
        self.com_z_ref=0.45
        self.foot_ref=0
        self.leg=0
        self.roll_p=3000
        self.roll_d=100
        self.pitch_p=3000
        self.pitch_d=100
        self.x_EP_p=3000
        self.x_EP_d=100
        self.y_EP_p=3000
        self.y_EP_d=100
        self.z_EP_p=3000
        self.z_EP_d=100
        self.Roll_P_in.setText(str(self.roll_p))
        self.Roll_D_in.setText(str(self.roll_d))
        self.Pitch_P_in.setText(str(self.pitch_p))
        self.Pitch_D_in.setText(str(self.pitch_d))
        self.X_EP_P_in.setText(str(self.x_EP_p))
        self.X_EP_D_in.setText(str(self.x_EP_d))
        self.Y_EP_P_in.setText(str(self.y_EP_p))
        self.Y_EP_D_in.setText(str(self.y_EP_d))
        self.Z_EP_P_in.setText(str(self.z_EP_p))
        self.Z_EP_D_in.setText(str(self.z_EP_d))

        self.slider_roll.valueChanged.connect(self.PublishRef)
        self.slider_pitch.valueChanged.connect(self.PublishRef)
        self.slider_com_x.valueChanged.connect(self.PublishRef)
        self.slider_com_y.valueChanged.connect(self.PublishRef)
        self.slider_com_z.valueChanged.connect(self.PublishRef)
        self.slider_foot.valueChanged.connect(self.PublishRef)

        self.reset_ori.clicked.connect(self.PublishRef)
        self.reset_com_pos.clicked.connect(self.PublishRef)
        self.reset_foot.clicked.connect(self.PublishRef)

        self.Roll_P_in.setValidator(QIntValidator())
        self.Roll_D_in.setValidator(QIntValidator())
        self.Pitch_P_in.setValidator(QIntValidator())
        self.Pitch_D_in.setValidator(QIntValidator())
        self.X_EP_P_in.setValidator(QIntValidator())
        self.X_EP_D_in.setValidator(QIntValidator())
        self.Y_EP_P_in.setValidator(QIntValidator())
        self.Y_EP_D_in.setValidator(QIntValidator())
        self.Z_EP_P_in.setValidator(QIntValidator())
        self.Z_EP_D_in.setValidator(QIntValidator())
        self.publish_gain.clicked.connect(self.PublishGain)

        self.pub3=rospy.Publisher('ROSGUI_PUB_POS_REF',Float32MultiArray,queue_size=1)
        self.pub4=rospy.Publisher('ROSGUI_PUB_QP_GAIN',Float32MultiArray,queue_size=1)

    def PublishRef(self):
        self.pitch_ref=int(self.slider_pitch.value()/50.0*self.MaxPitch)
        self.roll_ref=int(self.slider_roll.value()/50.0*self.MaxRoll)

        self.com_x_ref=self.slider_com_x.value()/50.0*self.MaxCoMx
        self.com_y_ref=self.slider_com_y.value()/50.0*self.MaxCoMy
        self.com_z_ref=0.45+self.slider_com_z.value()/50.0*self.MaxCoMz

        self.foot_ref=0.0+self.slider_foot.value()/50.0*self.Maxheight

        self.reset_ori.clicked.connect(self.ResetOri)
        self.reset_com_pos.clicked.connect(self.ResetCoM)
        self.reset_foot.clicked.connect(self.ResetFoot)
        
        if self.RL_button.isChecked(): self.leg=1
        elif self.RR_button.isChecked(): self.leg=2
        elif self.FL_button.isChecked(): self.leg=3
        elif self.FR_button.isChecked(): self.leg=4

        self.roll_out.setText(str(self.roll_ref))
        self.pitch_out.setText(str(self.pitch_ref))
        self.com_x_out.setText(str(self.com_x_ref))
        self.com_y_out.setText(str(self.com_y_ref))
        self.com_z_out.setText(str(self.com_z_ref))
        self.foot_out.setText(str(self.foot_ref))

        Reference=Float32MultiArray()
        Reference.data=[self.roll_ref*D2R,self.pitch_ref*D2R,self.com_x_ref,self.com_y_ref,self.com_z_ref,self.foot_ref,self.leg]
        self.pub3.publish(Reference)
    
    def ResetOri(self):
        self.roll_ref=0;
        self.pitch_ref=0;
        self.slider_roll.setValue(self.roll_ref)
        self.slider_pitch.setValue(self.pitch_ref)

    def ResetCoM(self):
        self.com_x_ref=0;
        self.com_y_ref=0;
        self.com_y_ref=0.45;
        self.slider_com_x.setValue(self.com_x_ref)
        self.slider_com_y.setValue(self.com_y_ref)
        self.slider_com_z.setValue(self.com_z_ref)
     
    def ResetFoot(self):
        self.foot_ref=0
        self.slider_foot.setValue(self.foot_ref)

    def PublishGain(self):
        self.roll_p=float(self.Roll_P_in.text())
        self.roll_d=float(self.Roll_D_in.text())
        self.pitch_p=float(self.Pitch_P_in.text())
        self.pitch_d=float(self.Pitch_D_in.text())
        self.x_EP_p=float(self.X_EP_P_in.text())
        self.x_EP_d=float(self.X_EP_D_in.text())
        self.y_EP_p=float(self.Y_EP_P_in.text())
        self.y_EP_d=float(self.Y_EP_D_in.text())
        self.z_EP_p=float(self.Z_EP_P_in.text())
        self.z_EP_d=float(self.Z_EP_D_in.text())

        Gain=Float32MultiArray()
        Gain.data=[self.roll_p,self.roll_d,self.pitch_p,self.pitch_d,self.x_EP_p,self.x_EP_d,self.y_EP_p,self.y_EP_d,self.z_EP_p,self.z_EP_d]
        self.pub4.publish(Gain)
        # print(roll_p,"/",pitch_p,"/",x_EP_p,"/",y_EP_p,"/",z_EP_p)
        # print(roll_d,"/",pitch_d,"/",x_EP_d,"/",y_EP_d,"/",z_EP_d)
        # print("---")
        #Reference.data=[]

if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)
    rospy.init_node("ROSGUI_NODE", anonymous=False)
    Window=RobotApp()

    Window.show()
    
    sys.exit( app.exec_() )
