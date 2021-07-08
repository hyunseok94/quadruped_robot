#!/usr/bin/env python
import rospy
from std_msgs.msg import String, UInt16

from PyQt5 import QtCore, QtGui, QtWidgets, uic
import sys
from PyQt5.QtWidgets import QMainWindow

class MainWindow(QMainWindow):
    def __init__(self):
        global pub
        global text_sub
        super(MainWindow,self).__init__()
        uic.loadUi("main_window.ui",self)
        self.setWindowTitle("Robot GUI")

        rospy.init_node("ROSGUI_NODE", anonymous=True)
        
        self.R1.clicked.connect(self.publish)
        self.R2.clicked.connect(self.publish)
    
        rospy.Subscriber('ROSGUI_SUB_MODE', UInt16, self.subCallback)
        pub=rospy.Publisher('ROSGUI_PUB_MODE',UInt16,queue_size=1)
        
    def subCallback(self,msg):
        if msg.data==0:
         text_sub="None" 
        elif msg.data==1:
         text_sub="Init Mode" 
        elif msg.data==2:
         text_sub="Ready Mode" 
        self.LE1.setText(text_sub)   

    def publish(self):
        if self.R1.isChecked():
            MODE=1
        elif(self.R2.isChecked()):
            MODE=2
        else:
            MODE=0
        pub.publish(MODE)

if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)
    Window=MainWindow()

    Window.show()
    sys.exit( app.exec_() )
