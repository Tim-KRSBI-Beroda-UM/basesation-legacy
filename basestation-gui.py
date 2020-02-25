#!/usr/bin/env python3

#WHEN MAKE ANY CHANGE, ALWAYS UPDATE THE DATE
#LAST UPDATED 25 FEB 2020

#SETTING ROS DISINI
nama_node = 'basestation'
nama_message = 'basestationnya'

robot1_node = 'robot1'
robot1_message_pub = robot1_node+'_frombasestation'
robot1_message_sub = robot1_node+'_tobasestation'

robot2_node = 'robot2'
robot2_message_pub = robot2_node+'_frombasestation'
robot2_message_sub = robot2_node+'_tobasestation'

robot3_node = 'robot3'
robot3_message_pub = robot3_node+'_frombasestation'
robot3_message_sub = robot3_node+'_tobasestation'

from PyQt5.QtGui     import *
from PyQt5.QtCore    import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.Qt import Qt
import sys

import telnetlib,time
import numpy as np
import translator as tl
from basestation import Ui_MainWindow

ip = ""
port = ""
check_refbox = False
refbox = "Not Connect"
ownrefbox = ''

tim = 0
offset_lapangan_x = 105
offset_lapangan_y = 50

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def cart2pol(x, y):
    rho = math.sqrt(x ** 2 + y ** 2)
    phi = math.degrees(math.atan2(y, x))
    return (round(rho), round(phi))

def pol2cart(rho, phi):
    x = rho * math.cos(math.radians(phi))
    y = rho * math.sin(math.radians(phi))
    return(x, y)

class refbox_thread(QThread):
    def run(self):
        global ip,port,refbox
        tn = telnetlib.Telnet(ip, port)
        while(True):
            try :
                data = tn.read_eager().decode('ascii')
                if data is not '':
                    refbox = tl.refbox2human(data)
                    #print(data)
                    #time.sleep(0.1)
                #print(data)
            except EOFError as e:
                #print("not connect")
                refbox = 'EOFError/Is Down'

import rospy
from std_msgs.msg import Int16MultiArray

robot1_subsstat = np.array([-100,-100,90,-100,-100,-0,0,-1]) #posx posy hdg ballx bally ball gawanghdg behav
robot1_offsetFromMap = np.array([34,34,0])
robot1_offsetFromRobot = np.array([0,0,0])
robot1_overrideToggle = False
robot1_command = np.array([0,0,0,0])

robot2_subsstat = np.array([-100,-100,90,-100,-100,0,0,-1]) #posx posy hdg ballx bally ball gawanghdg behav
robot2_offsetFromMap = np.array([34,34+40,0])
robot2_offsetFromRobot = np.array([0,0,0])
robot2_overrideToggle = False
robot2_command = np.array([0,0,0,0])
robot2_posstat = np.array([0,0,0,140,60,0,0,0]) #posx posy posz wpx wpy wpz bolax bolay

robot3_subsstat = np.array([-100,-100,90,-100,-100,0,0,-1]) #posx posy hdg ballx bally ball gawanghdg behav
robot3_offsetFromMap = np.array([34,34+80,0])
robot3_offsetFromRobot = np.array([0,0,0])
robot3_overrideToggle = False
robot3_command = np.array([0,0,0,0])
robot3_posstat = np.array([0,0,0,0,0,0,0,0]) #posx posy posz wpx wpy wpz bolax bolay

class ros_thread(QThread):
    rospy.init_node('basestation', anonymous=True)
    robot1_pub = rospy.Publisher(robot1_message_pub, Int16MultiArray, queue_size=10)
    robot2_pub = rospy.Publisher(robot2_message_pub, Int16MultiArray, queue_size=10)
    robot3_pub = rospy.Publisher(robot3_message_pub, Int16MultiArray, queue_size=10)


    def callback_r1(self,data):
        #print(data)
        robot1_subsstat[0] = data.data[0] #posx
        robot1_subsstat[1] = data.data[1] #posy
        robot1_subsstat[2] = data.data[2] #hdg

        ##################################
        robot1_subsstat[3] = (0.012 * (data.data[3]**2)) - (0.815*data.data[3]) + 14.61 #ballx
        robot1_subsstat[4] = (0.012 * (data.data[4]**2)) - (0.815*data.data[4]) + 14.61 #bally

        #robot1_subsstat[3] = 7.9674 * np.exp(0.0077 * data.data[3])
        #robot1_subsstat[4] = 7.9674 * np.exp(0.0077 * data.data[4])
        ##################################
        #robot1_subsstat[7] = data.data[7] #behav

        #command x y z
        msg = Int16MultiArray(data = robot1_command)
        self.robot1_pub.publish(msg)
        #rospy.loginfo(msg)

    def callback_r2(self,data):

        robot2_subsstat[0] = data.data[0]
        robot2_subsstat[1] = data.data[1]
        robot2_subsstat[2] = data.data[2]

        ##################################
        robot2_subsstat[3] = (0.012 * (data.data[3]**2)) - (0.823*data.data[3]) + 14.53
        robot2_subsstat[4] = (0.012 * (data.data[4]**2)) - (0.823*data.data[4]) + 14.53

        #robot2_subsstat[4] = 7.9674 * np.exp(0.0077 * data.data[4])
        ##################################
        #robot2_subsstat[7] = data.data[7]

        #command x y z
        msg = Int16MultiArray(data = robot2_command)
        self.robot2_pub.publish(msg)
        #rospy.loginfo(msg)


    def callback_r3(self,data):
        #print(data.data[0])
        robot3_subsstat[0] = data.data[0]
        robot3_subsstat[1] = data.data[1]
        robot3_subsstat[2] = data.data[2]

        ################################## INI PERLU DIRUBAH DULU NILAINYA ASLINE
        robot3_subsstat[3] = data.data[3]
        robot3_subsstat[4] = data.data[4]
        ##################################
        #robot3_subsstat[7] = data.data[7]

        #command x y z
        msg = Int16MultiArray(data = robot3_command)
        self.robot3_pub.publish(msg)
        #rospy.loginfo(msg)

    def listener(self):
        rospy.Subscriber(robot1_message_sub, Int16MultiArray, self.callback_r1)
        rospy.Subscriber(robot2_message_sub, Int16MultiArray, self.callback_r2)
        rospy.Subscriber(robot3_message_sub, Int16MultiArray, self.callback_r3)
        rospy.spin()

    def run(self):
        while not rospy.is_shutdown() :
            try:
                self.listener()
            except rospy.ROSInterruptException:
                pass

class command_thread(QThread):
    def gotowp1(self,inposx,inposy,inposz,inwpx,inwpy,inwpz):
        #z first baru, xy
        if inwpz - inposz >=20:
            x = 0
            y = 0
            z = -90
        elif inwpz - inposz <=-20:
            x = 0
            y = 0
            z = 90
        elif inwpz - inposz <=20 or inwpz - inposz >=-20:
            z = inposz - inwpz

            if abs(inwpy - inposy) <=20:
                x = 0
            else :
                x = (inposy - inwpy)*-1

            if abs(inwpx - inposx) <=20:
                y=0
            else:
                y = (inposx - inwpx)*-1
        return int(x),int(y),int(z)


    def perintah(self,input):
        if input == 'Stop':
            #MANCAL
            if robot1_overrideToggle is True :
                robot1_command[0]=6
            else :
                robot1_command[0]=0

            if robot2_overrideToggle is True :
                robot2_command[0]=6
            else :
                robot2_command[0]=0

            if robot3_overrideToggle is True :
                robot3_command[0]=6
            else :
                robot3_command[0]=0

            #robot1_command[1]=robot1_tempx
            #robot1_command[2]=robot1_tempy
            #robot1_command[3]=robot1_tempz

        elif input == 'Start':
            if robot1_overrideToggle is True :
                robot1_command[0]=0
            else :
                robot1_command[0]=3


            if robot2_overrideToggle is True :
                robot2_command[0]=0

            else :
                robot2_command[0]=3
                robot2_command[1],robot2_command[2],robot2_command[3] = self.gotowp1(robot2_posstat[0],robot2_posstat[1],robot2_posstat[2],robot2_posstat[3],robot2_posstat[4],robot2_posstat[5])


            if robot3_overrideToggle is True :
                robot3_command[0]=0
            else :
                robot3_command[0]=3

    def run(self):
        global check_refbox
        while(True):
            if check_refbox is True:
                self.perintah(refbox)
            else :
                self.perintah(ownrefbox)



class window(QtWidgets.QDialog, Ui_MainWindow):

    def __init__(self):
        global tim
        print("================================")
        print("=             SORA             =")
        print("=------------------------------=")
        print("=             2019             =")
        print("================================")

        super().__init__()
        self.setupUi(self)

        self.refbox_th = refbox_thread(self)
        self.connecting.clicked.connect(self.handle_connecting)

        self.ros_th = ros_thread(self)
        self.ros_th.start()

        self.command_th = command_thread(self)
        self.command_th.start()

        self.norm = QColor(200, 0, 200)
        self.clicked = QColor(150, 0, 150)
        self.hover = QColor(255, 0, 255)

        self.curpos_r1 = QVector3D(34,34,0)
        self.move_r1 = False
        self.rot_r1 = False
        self.color_r1 = self.norm
        self.atkwp_r1 = QPoint(34,0)
        self.defwp_r1 = QPoint(34,0)
        self.ball_r1 = QPoint(34,0)

        self.curpos_r2 = QVector3D(34,34+80,0)
        self.move_r2 = False
        self.rot_r2 = False
        self.color_r2 = self.norm
        self.atkwp_r2 = QPoint(34,0)
        self.defwp_r2 = QPoint(34,0)
        self.ball_r2 = QPoint(34,34+40)

        self.curpos_r3 = QVector3D(34,34+80,0)
        self.move_r3 = False
        self.rot_r3 = False
        self.color_r3 = self.norm
        self.atkwp_r3 = QPoint(34,34+80)
        self.defwp_r3 = QPoint(34,34+80)
        self.ball_r3 = QPoint(34,34+80)

        self.ball_real = QPoint(0,0)

        self.color_gawHome = QColor(255, 0, 255)
        self.color_gawAway = QColor(0, 255, 255)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(100)

        self.scRobot1 = QShortcut(QKeySequence("Ctrl+1"), self)
        self.scRobot1.activated.connect(self.toogle_overrider1)

        self.scRobot2 = QShortcut(QKeySequence("Ctrl+2"), self)
        self.scRobot2.activated.connect(self.toogle_overrider2)

        self.scRobot3 = QShortcut(QKeySequence("Ctrl+3"), self)
        self.scRobot3.activated.connect(self.toogle_overrider3)

        self.override_r1.stateChanged.connect(self.override_r1_handle)
        self.override_r2.stateChanged.connect(self.override_r2_handle)
        self.override_r3.stateChanged.connect(self.override_r3_handle)

        self.tim.currentIndexChanged.connect(self.selectionTim)

        self.lastPoint = QPoint()
        self.linePoint = QPoint()

        self.start.clicked.connect(self.start_handle)
        self.stop.clicked.connect(self.stop_handle)
        self.drop.clicked.connect(self.drop_handle)
        self.park.clicked.connect(self.park_handle)
        self.reset.clicked.connect(self.reset_handle)
        self.end.clicked.connect(self.end_handle)

        self.goal_cyan.clicked.connect(self.goal_cyan_handle)
        self.kickoff_cyan.clicked.connect(self.kickoff_cyan_handle)
        self.freekick_cyan.clicked.connect(self.freekick_cyan_handle)
        self.goalkick_cyan.clicked.connect(self.goalkick_cyan_handle)
        self.throwin_cyan.clicked.connect(self.throwin_cyan_handle)
        self.corner_cyan.clicked.connect(self.corner_cyan_handle)
        self.penalty_cyan.clicked.connect(self.penalty_cyan_handle)
        self.repair_cyan.clicked.connect(self.repair_cyan_handle)

        self.goal_magenta.clicked.connect(self.goal_magenta_handle)
        self.kickoff_magenta.clicked.connect(self.kickoff_magenta_handle)
        self.freekick_magenta.clicked.connect(self.freekick_magenta_handle)
        self.goalkick_magenta.clicked.connect(self.goalkick_magenta_handle)
        self.throwin_magenta.clicked.connect(self.throwin_magenta_handle)
        self.corner_magenta.clicked.connect(self.corner_magenta_handle)
        self.penalty_magenta.clicked.connect(self.penalty_magenta_handle)
        self.repair_magenta.clicked.connect(self.repair_magenta_handle)

        self.refbox.stateChanged.connect(self.handle_refbox)

        self.show()

    def keyPressEvent(self,event):

        #HOTKEY ROBOT1
        if event.key() == Qt.Key_A and robot1_overrideToggle is True:
            robot1_command[1] = -120
        elif event.key() == Qt.Key_D and robot1_overrideToggle is True:
            robot1_command[1] = 120
        elif event.key() == Qt.Key_S and robot1_overrideToggle is True:
            robot1_command[2] = -120
        elif event.key() == Qt.Key_W and robot1_overrideToggle is True:
            robot1_command[2] = 120
        elif event.key() == Qt.Key_Q and robot1_overrideToggle is True:
            robot1_command[3] = -120
        elif event.key() == Qt.Key_E and robot1_overrideToggle is True:
            robot1_command[3] = 120

        #HOTKEY ROBOT2
        if event.key() == Qt.Key_F and robot2_overrideToggle is True:
            robot2_command[1] = -120
        elif event.key() == Qt.Key_H and robot2_overrideToggle is True:
            robot2_command[1] = 120
        elif event.key() == Qt.Key_G and robot2_overrideToggle is True:
            robot2_command[2] = -120
        elif event.key() == Qt.Key_T and robot2_overrideToggle is True:
            robot2_command[2] = 120
        elif event.key() == Qt.Key_R and robot2_overrideToggle is True:
            robot2_command[3] = -120
        elif event.key() == Qt.Key_Y and robot2_overrideToggle is True:
            robot2_command[3] = 120

        #HOTKEY ROBOT3
        if event.key() == Qt.Key_J and robot3_overrideToggle is True:
            robot3_command[1] = -120
        elif event.key() == Qt.Key_L and robot3_overrideToggle is True:
            robot3_command[1] = 120
        elif event.key() == Qt.Key_K and robot3_overrideToggle is True:
            robot3_command[2] = -120
        elif event.key() == Qt.Key_I and robot3_overrideToggle is True:
            robot3_command[2] = 120
        elif event.key() == Qt.Key_U and robot3_overrideToggle is True:
            robot3_command[3] = -120
        elif event.key() == Qt.Key_O and robot3_overrideToggle is True:
            robot3_command[3] = 120

    def keyReleaseEvent(self, event):

        #HOTKEY ROBOT1
        if event.key() == Qt.Key_A and robot1_overrideToggle is True:
            robot1_command[1] = 0
        if event.key() == Qt.Key_D and robot1_overrideToggle is True:
            robot1_command[1] = 0
        if event.key() == Qt.Key_W and robot1_overrideToggle is True:
            robot1_command[2] = 0
        if event.key() == Qt.Key_S and robot1_overrideToggle is True:
            robot1_command[2] = 0
        if event.key() == Qt.Key_Q and robot1_overrideToggle is True:
            robot1_command[3] = 0
        if event.key() == Qt.Key_E and robot1_overrideToggle is True:
            robot1_command[3] = 0

        #HOTKEY ROBOT2
        if event.key() == Qt.Key_F and robot2_overrideToggle is True:
            robot2_command[1] = 0
        if event.key() == Qt.Key_H and robot2_overrideToggle is True:
            robot2_command[1] = 0
        if event.key() == Qt.Key_G and robot2_overrideToggle is True:
            robot2_command[2] = 0
        if event.key() == Qt.Key_T and robot2_overrideToggle is True:
            robot2_command[2] = 0
        if event.key() == Qt.Key_R and robot2_overrideToggle is True:
            robot2_command[3] = 0
        if event.key() == Qt.Key_Y and robot2_overrideToggle is True:
            robot2_command[3] = 0

        #HOTKEY ROBOT3
        if event.key() == Qt.Key_J and robot3_overrideToggle is True:
            robot3_command[1] = 0
        if event.key() == Qt.Key_L and robot3_overrideToggle is True:
            robot3_command[1] = 0
        if event.key() == Qt.Key_K and robot3_overrideToggle is True:
            robot3_command[2] = 0
        if event.key() == Qt.Key_I and robot3_overrideToggle is True:
            robot3_command[2] = 0
        if event.key() == Qt.Key_U and robot3_overrideToggle is True:
            robot3_command[3] = 0
        if event.key() == Qt.Key_O and robot3_overrideToggle is True:
            robot3_command[3] = 0

    def handle_refbox(self, state):
        global check_refbox,ownrefbox,refbox
        if state == QtCore.Qt.Checked :
            check_refbox = True
            refbox = 'Stop'
        else :
            check_refbox = False
            ownrefbox = 'Stop'

    def selectionTim(self,i):
        global tim
        tim = i
        if tim == 0:
            self.norm = QColor(200, 0, 200)
            self.clicked = QColor(150, 0, 150)
            self.hover = QColor(255, 0, 255)
            self.color_r1 = self.norm
            self.color_r2 = self.norm
            self.color_r3 = self.norm
            pal = self.tim.palette()
            pal.setColor(QtGui.QPalette.Button, QtGui.QColor(255,0,255))
            self.tim.setPalette(pal)
            self.color_gawHome = QColor(255, 0, 255)
            self.color_gawAway = QColor(0, 255, 255)
            self.update()

        if tim == 1:
            self.norm = QColor(0, 200, 200)
            self.clicked = QColor(0, 150, 150)
            self.hover = QColor(0, 255, 255)
            self.color_r1 = self.norm
            self.color_r2 = self.norm
            self.color_r3 = self.norm
            pal = self.tim.palette()
            pal.setColor(QtGui.QPalette.Button, QtGui.QColor(0,255,255))
            self.tim.setPalette(pal)
            self.color_gawHome = QColor(0, 255, 255)
            self.color_gawAway = QColor(255, 0, 255)
            self.update()

    def toogle_overrider1(self):
        if self.override_r1.isChecked():
            self.override_r1.setChecked(False)
        else:
            self.override_r1.setChecked(True)


    def toogle_overrider2(self):
        if self.override_r2.isChecked():
            self.override_r2.setChecked(False)
        else:
            self.override_r2.setChecked(True)

    def toogle_overrider3(self):
        if self.override_r3.isChecked():
            self.override_r3.setChecked(False)
        else:
            self.override_r3.setChecked(True)

    lastx_r1=0
    lasty_r1=0
    lastx_r2=0
    lasty_r2=0
    lastx_r3=0
    lasty_r3=0

    def update_frame(self):
        #REFBOX
        global check_refbox
        if check_refbox is True:
            self.state.setText(refbox)
        else :
            self.state.setText(ownrefbox)

        if robot1_overrideToggle is True:
            self.curpos_r1.setX(robot1_offsetFromMap[0])
            self.curpos_r1.setY(robot1_offsetFromMap[1])
            self.curpos_r1.setZ(robot1_offsetFromMap[2])

        elif robot1_overrideToggle is False:
            self.curpos_r1.setX((robot1_subsstat[0] - robot1_offsetFromRobot[0]) + robot1_offsetFromMap[0])
            self.curpos_r1.setY((robot1_subsstat[1] - robot1_offsetFromRobot[1]) + robot1_offsetFromMap[1])
            self.curpos_r1.setZ((robot1_subsstat[2] - robot1_offsetFromRobot[2]) + robot1_offsetFromMap[2])

        if self.curpos_r1.x() < 65 :
            self.curpos_r1.setX(65)
        elif self.curpos_r1.x() > 745 :
            self.curpos_r1.setX(745)
        if self.curpos_r1.y() < 50 :
            self.curpos_r1.setY(50)
        elif self.curpos_r1.y() > 450 :
            self.curpos_r1.setY(450)

        if robot2_overrideToggle is True:
            self.curpos_r2.setX(robot2_offsetFromMap[0])
            self.curpos_r2.setY(robot2_offsetFromMap[1])
            self.curpos_r2.setZ(robot2_offsetFromMap[2])

        elif robot2_overrideToggle is False:
            x_r2 = (robot2_subsstat[0] - robot2_offsetFromRobot[0]) + robot2_offsetFromMap[0]
            y_r2 = (robot2_subsstat[1] - robot2_offsetFromRobot[1]) + robot2_offsetFromMap[1]

            if x_r2 != self.lastx_r2 :
                outx_r2 = x_r2 - self.lastx_r2
            else :
                outx_r2 = 0
            if y_r2 != self.lasty_r2 :
                outy_r2 = y_r2 - self.lasty_r2
            else :
                outy_r2 = 0

            self.lastx_r2 = x_r2
            self.lasty_r2 = y_r2

            yout_r2 = outx_r2*np.cos(np.radians(self.curpos_r2.z())) + outy_r2*-np.sin(np.radians(self.curpos_r2.z()))
            xout_r2 = outx_r2*np.sin(np.radians(self.curpos_r2.z())) + outy_r2*np.cos(np.radians(self.curpos_r2.z()))

            self.curpos_r2.setX(self.curpos_r2.x()+xout_r2)
            self.curpos_r2.setY(self.curpos_r2.y()+yout_r2)
            self.curpos_r2.setZ((robot2_subsstat[2] - robot2_offsetFromRobot[2]) + robot2_offsetFromMap[2])

        if self.curpos_r2.x() < 65 :
            self.curpos_r2.setX(65)
        elif self.curpos_r2.x() > 745 :
            self.curpos_r2.setX(745)
        if self.curpos_r2.y() < 50 :
            self.curpos_r2.setY(50)
        elif self.curpos_r2.y() > 450 :
            self.curpos_r2.setY(450)

        robot2_posstat[0] = self.curpos_r2.x()-offset_lapangan_x
        robot2_posstat[1] = self.curpos_r2.y()-offset_lapangan_y
        robot2_posstat[2] = self.curpos_r2.z()
        self.defwp_r2.setX(robot2_posstat[3]+offset_lapangan_x)
        self.defwp_r2.setY(robot2_posstat[4]+offset_lapangan_y)

        if robot3_overrideToggle is True:
            self.curpos_r3.setX(robot3_offsetFromMap[0])
            self.curpos_r3.setY(robot3_offsetFromMap[1])
            self.curpos_r3.setZ(robot3_offsetFromMap[2])

        elif robot3_overrideToggle is False:
            x_r3 = (robot3_subsstat[0] - robot3_offsetFromRobot[0]) + robot3_offsetFromMap[0]
            y_r3 = (robot3_subsstat[1] - robot3_offsetFromRobot[1]) + robot3_offsetFromMap[1]

            if x_r3 != self.lastx_r3 :
                outx_r3 = x_r3-self.lastx_r3
            else :
                outx_r3 = 0

            if y_r3 != self.lasty_r3 :
                outy_r3 = y_r3-self.lasty_r3
            else :
                outy_r3 = 0

            self.lastx_r3 = x_r3
            self.lasty_r3 = y_r3

            yout_r3 = outx_r3*np.cos(np.radians(self.curpos_r3.z())) + outy_r3*-np.sin(np.radians(self.curpos_r3.z()))
            xout_r3 = outx_r3*np.sin(np.radians(self.curpos_r3.z())) + outy_r3*np.cos(np.radians(self.curpos_r3.z()))

            self.curpos_r3.setX(self.curpos_r3.x()+xout_r3)
            self.curpos_r3.setY(self.curpos_r3.y()+yout_r3)
            self.curpos_r3.setZ((robot3_subsstat[2] - robot3_offsetFromRobot[2]) + robot3_offsetFromMap[2])

        if self.curpos_r3.x() < 65 :
            self.curpos_r3.setX(65)
        elif self.curpos_r3.x() > 745 :
            self.curpos_r3.setX(745)
        if self.curpos_r3.y() < 50 :
            self.curpos_r3.setY(50)
        elif self.curpos_r3.y() > 450 :
            self.curpos_r3.setY(450)

        self.posisi_r1.setText( str(robot1_subsstat[0]) +" , "+ str(robot1_subsstat[1]) +" , "+ str(robot1_subsstat[2]))
        self.posisi_r2.setText( str(robot2_subsstat[0]) +" , "+ str(robot2_subsstat[1]) +" , "+ str(robot2_subsstat[2]))
        self.posisi_r3.setText( str(robot3_subsstat[0]) +" , "+ str(robot3_subsstat[1]) +" , "+ str(robot3_subsstat[2]))

        self.command_r1.setText(str(tl.behavior2human(robot1_command[0])) + str(robot1_command) )
        self.command_r2.setText(str(tl.behavior2human(robot2_command[0])) + str(robot2_command) )
        self.command_r3.setText(str(tl.behavior2human(robot3_command[0])) + str(robot3_command) )

        self.behavior_r1.setText(tl.behavior2human(robot1_subsstat[7]))
        self.behavior_r2.setText(tl.behavior2human(robot2_subsstat[7]))
        self.behavior_r3.setText(tl.behavior2human(robot3_subsstat[7]))

        #self.command_r1

        #IMPORTANT FOR map
        self.update()

        #SIN COS MASUK SINI
        #ROBOT1 STATS
        self.ball_r1.setX(robot1_subsstat[3] * np.cos(np.radians( self.curpos_r1.z())) + robot1_subsstat[4] *-np.sin(np.radians( self.curpos_r1.z())) )
        self.ball_r1.setY(robot1_subsstat[3] * np.sin(np.radians( self.curpos_r1.z())) + robot1_subsstat[4] * np.cos(np.radians( self.curpos_r1.z())) )

        #ROBOT2 STATS
        self.ball_r2.setX(robot2_subsstat[3] * np.cos(np.radians( self.curpos_r2.z())) + robot2_subsstat[4] *-np.sin(np.radians( self.curpos_r2.z())) )
        self.ball_r2.setY(robot2_subsstat[3] * np.sin(np.radians( self.curpos_r2.z())) + robot2_subsstat[4] * np.cos(np.radians( self.curpos_r2.z())) )

        #ROBOT3 STATS
        self.ball_r3.setX(robot3_subsstat[3] * np.cos(np.radians( self.curpos_r3.z())) + robot3_subsstat[4] *-np.sin(np.radians( self.curpos_r3.z())) )
        self.ball_r3.setY(robot3_subsstat[3] * np.sin(np.radians( self.curpos_r3.z())) + robot3_subsstat[4] * np.cos(np.radians( self.curpos_r3.z())) )

        try :
            A = (self.curpos_r3.x() , self.curpos_r3.y())
            B = (self.curpos_r3.x() + self.ball_r3.y(), self.curpos_r3.y() + self.ball_r3.x())
            C = (self.curpos_r2.x() , self.curpos_r2.y())
            D = (self.curpos_r2.x() + self.ball_r2.y(), self.curpos_r2.y() + self.ball_r2.x())
            x,y = line_intersection((A, B), (C, D))
            self.ball_real.setX(x)
            self.ball_real.setY(y)
        except :
            pass

        #MAP
        self.posmap_r1.setText(str(self.curpos_r1.x()-offset_lapangan_x)+" , "+str(self.curpos_r1.y()-offset_lapangan_y)+" , "+str(self.curpos_r1.z()))
        self.posmap_r2.setText(str(self.curpos_r2.x()-offset_lapangan_x)+" , "+str(self.curpos_r2.y()-offset_lapangan_y)+" , "+str(self.curpos_r2.z()))
        self.posmap_r3.setText(str(self.curpos_r3.x()-offset_lapangan_x)+" , "+str(self.curpos_r3.y()-offset_lapangan_y)+" , "+str(self.curpos_r3.z()))

        self.posmap_b1.setText(str(self.curpos_r1.x() + self.ball_r1.y() - offset_lapangan_x)+" , "+str(self.curpos_r1.y() + self.ball_r1.x() - offset_lapangan_y))
        self.posmap_b2.setText(str(self.curpos_r2.x() + self.ball_r2.y() - offset_lapangan_x)+" , "+str(self.curpos_r2.y() + self.ball_r2.x() - offset_lapangan_y))
        self.posmap_b3.setText(str(self.curpos_r3.x() + self.ball_r3.y() - offset_lapangan_x)+" , "+str(self.curpos_r3.y() + self.ball_r3.x() - offset_lapangan_y))

        self.posmap_rb.setText(str(self.ball_real.x() - offset_lapangan_x)+" , "+str(self.ball_real.y() - offset_lapangan_y))



    def handle_connecting(self):
        global ip,port
        ip = self.ip.text()
        port = self.port.text()
        self.refbox_th.start()

    def paintEvent(self, event):
        painter = QPainter(self)

        #background
        painter.setBrush(QColor(50, 50, 50))
        painter.drawRect(0,0, 1366, 768)
        #lapangan rumput
        painter.setBrush(QColor(24, 112, 15))
        painter.drawRect(20,20, 775, 480)
        #lapangan garis
        painter.setPen(QPen(Qt.white,  3, Qt.SolidLine))
        painter.drawRect(105,50,600,400)
        #kotak penalty
        painter.drawRect(630,109,75,283)
        painter.drawRect(105,109,75,283)
        #gawang
        painter.drawRect(680,158,25,183)
        painter.drawRect(105,158,25,183)
        #lingkaran
        painter.drawEllipse(339, 184, 133, 133)
        painter.drawEllipse(403, 248, 5, 5)
        painter.drawEllipse(225, 248, 3, 3)
        painter.drawEllipse(582, 248, 3, 3)
        #seperempat lingkaran
        painter.drawArc(88, 430, 38, 38, 0 * 16, 90 * 16)
        painter.drawArc(88, 32, 38, 38, 270 * 16, 90 * 16)
        painter.drawArc(685, 430, 38, 38, 90 * 16, 90 * 16)
        painter.drawArc(685, 32, 38, 38, 180 * 16, 90 * 16)
        #garis tengah
        painter.drawLine(405,50,405,450)
        #gawang kiri mag
        painter.setBrush(self.color_gawHome)
        painter.drawRect(65,190, 40, 120)
        #gawang kanan cya
        painter.setBrush(self.color_gawAway)
        painter.drawRect(705,190, 40, 120)
        #titik check point
        painter.setPen(QPen(Qt.black,  5, Qt.SolidLine))
        painter.drawEllipse(223, 148, 5, 5)
        painter.drawEllipse(223, 348, 5, 5)
        painter.drawEllipse(580, 148, 5, 5)
        painter.drawEllipse(580, 348, 5, 5)
        painter.drawEllipse(403, 148, 5, 5)
        painter.drawEllipse(403, 348, 5, 5)

        #markah
        painter.setPen(QPen(Qt.white,  1, Qt.DotLine))
        for i in range(30) :
            painter.drawLine(offset_lapangan_x + (i*20), offset_lapangan_y ,offset_lapangan_x + (i*20) , offset_lapangan_y + 400)
        for i in range(20) :
            painter.drawLine(offset_lapangan_x, offset_lapangan_y + (i*20),offset_lapangan_x + 600 , offset_lapangan_y + (i*20))

        # =================================================================================================================================
        #painter.setPen(QPen(Qt.red,  10, Qt.SolidLine))
        #painter.drawPoint(self.atkwp_r1)
        #painter.setPen(QPen(Qt.red,  1, Qt.DashDotLine))
        #painter.drawLine(self.curpos_r1.x(),self.curpos_r1.y(),self.atkwp_r1.x(),self.atkwp_r1.y())
        #painter.setPen(QPen(Qt.blue,  10, Qt.SolidLine))
        #painter.drawPoint(self.defwp_r1)
        #painter.setPen(QPen(Qt.blue,  1, Qt.DashDotLine))
        #painter.drawLine(self.curpos_r1.x(),self.curpos_r1.y(),self.defwp_r1.x(),self.defwp_r1.y())
        painter.setPen(QPen(QColor(255,107,3),  10, Qt.DotLine))
        painter.drawPoint(self.curpos_r1.x() + self.ball_r1.y(), self.curpos_r1.y() + self.ball_r1.x())
        painter.setPen(QPen(QColor(255,107,3),  1, Qt.DashDotLine))
        painter.drawLine(self.curpos_r1.x(), self.curpos_r1.y(), self.curpos_r1.x() + self.ball_r1.y(), self.curpos_r1.y() + self.ball_r1.x())


        # =================================================================================================================================
        #painter.setPen(QPen(Qt.red,  10, Qt.SolidLine))
        #painter.drawPoint(self.atkwp_r2)
        #painter.setPen(QPen(Qt.red,  1, Qt.DashDotLine))
        #painter.drawLine(self.curpos_r2.x(),self.curpos_r2.y(),self.atkwp_r2.x(),self.atkwp_r2.y())

        painter.setPen(QPen(Qt.blue,  10, Qt.SolidLine))
        painter.drawPoint(self.defwp_r2)
        painter.setPen(QPen(Qt.blue,  1, Qt.DashDotLine))
        painter.drawLine(self.curpos_r2.x(),self.curpos_r2.y(),self.defwp_r2.x(),self.defwp_r2.y())

        painter.setPen(QPen(QColor(255,107,3),  10, Qt.DotLine))
        painter.drawPoint(self.curpos_r2.x() + self.ball_r2.y(), self.curpos_r2.y() + self.ball_r2.x())
        painter.setPen(QPen(QColor(255,107,3),  1, Qt.DashDotLine))
        painter.drawLine(self.curpos_r2.x() , self.curpos_r2.y(), self.curpos_r2.x() + self.ball_r2.y(), self.curpos_r2.y() + self.ball_r2.x())

        # ==================================================================================================================================
        #painter.setPen(QPen(Qt.red,  10, Qt.SolidLine))
        #painter.drawPoint(self.atkwp_r3)
        #painter.setPen(QPen(Qt.red,  1, Qt.DashDotLine))
        #painter.drawLine(self.curpos_r3.x(),self.curpos_r3.y(),self.atkwp_r3.x(),self.atkwp_r3.y())

        #painter.setPen(QPen(Qt.blue,  10, Qt.SolidLine))
        #painter.drawPoint(self.defwp_r3)
        #painter.setPen(QPen(Qt.blue,  1, Qt.DashDotLine))
        #painter.drawLine(self.curpos_r3.x(),self.curpos_r3.y(),self.defwp_r3.x(),self.defwp_r3.y())

        painter.setPen(QPen(QColor(255,107,3),  10, Qt.DotLine))
        painter.drawPoint(self.curpos_r3.x() + self.ball_r3.y(), self.curpos_r3.y() + self.ball_r3.x())
        painter.setPen(QPen(QColor(255,107,3),  1, Qt.DashDotLine))
        painter.drawLine(self.curpos_r3.x(), self.curpos_r3.y(), self.curpos_r3.x() + self.ball_r3.y(), self.curpos_r3.y() + self.ball_r3.x())

        # =================================================================================================================================

        painter.setPen(QPen(QColor(255,22,3),  10, Qt.DotLine))
        painter.drawPoint(self.ball_real.x(), self.ball_real.y())

        painter.setPen(QPen(Qt.black,  1, Qt.SolidLine))
        painter.setBrush(self.color_r1)
        painter.drawChord(self.curpos_r1.x()-12,self.curpos_r1.y()-12,25,25, (50+self.curpos_r1.z()) * 16, 260 * 16);
        painter.drawText(self.curpos_r1.x()+8-12,self.curpos_r1.y()+18-12,"1")

        painter.setBrush(self.color_r2)
        painter.drawChord(self.curpos_r2.x()-12,self.curpos_r2.y()-12,25,25, (50+self.curpos_r2.z()) * 16, 260 * 16);
        painter.drawText(self.curpos_r2.x()+8-12,self.curpos_r2.y()+18-12,"2")

        painter.setBrush(self.color_r3)
        painter.drawChord(self.curpos_r3.x()-12,self.curpos_r3.y()-12,25,25, (50+self.curpos_r3.z()) * 16, 260 * 16);
        painter.drawText(self.curpos_r3.x()+8-12,self.curpos_r3.y()+18-12,"3")

        painter.drawLine(21,self.linePoint.y(),796,self.linePoint.y())
        painter.drawLine(self.linePoint.x(),21,self.linePoint.x(),500)

    def mousePressEvent(self, event):
        self.lastPoint = event.pos()
        self.linePoint = event.pos()
        #print(self.lastPoint)

        if event.button() == Qt.RightButton:
            if self.lastPoint.x()+12 >= self.curpos_r1.x() and self.lastPoint.x()+12 <= self.curpos_r1.x()+25 and self.lastPoint.y()+12 >= self.curpos_r1.y() and self.lastPoint.y()+12 <= self.curpos_r1.y()+25 and robot1_overrideToggle is True:
                self.color_r1 = self.clicked
                self.rot_r1 = True
                self.update()

            elif self.lastPoint.x()+12 >= self.curpos_r2.x() and self.lastPoint.x()+12 <= self.curpos_r2.x()+25 and self.lastPoint.y()+12 >= self.curpos_r2.y() and self.lastPoint.y()+12 <= self.curpos_r2.y()+25 and robot2_overrideToggle is True:
                self.color_r2 = self.clicked
                self.rot_r2 = True
                self.update()

            elif self.lastPoint.x()+12 >= self.curpos_r3.x() and self.lastPoint.x()+12 <= self.curpos_r3.x()+25 and self.lastPoint.y()+12 >= self.curpos_r3.y() and self.lastPoint.y()+12 <= self.curpos_r3.y()+25 and robot3_overrideToggle is True:
                self.color_r3 = self.clicked
                self.rot_r3 = True
                self.update()
            else:
                #print(self.lastPoint)
                self.posmap_mouse.setText(str(self.lastPoint.x()) + " , " + str(self.lastPoint.y()) + " (" + str(self.lastPoint.x() - offset_lapangan_x) + " , " + str(self.lastPoint.y()  - offset_lapangan_y) + ")")

        elif event.button() == Qt.LeftButton:
            if self.lastPoint.x()+12 >= self.curpos_r1.x() and self.lastPoint.x()+12 <= self.curpos_r1.x()+25 and self.lastPoint.y()+12 >= self.curpos_r1.y() and self.lastPoint.y()+12 <= self.curpos_r1.y()+25 and robot1_overrideToggle is True:
                self.color_r1 = self.clicked
                self.move_r1 = True
                self.update()

            elif self.lastPoint.x()+12 >= self.curpos_r2.x() and self.lastPoint.x()+12 <= self.curpos_r2.x()+25 and self.lastPoint.y()+12 >= self.curpos_r2.y() and self.lastPoint.y()+12 <= self.curpos_r2.y()+25 and robot2_overrideToggle is True:
                self.color_r2 = self.clicked
                self.move_r2 = True
                self.update()

            elif self.lastPoint.x()+12 >= self.curpos_r3.x() and self.lastPoint.x()+12 <= self.curpos_r3.x()+25 and self.lastPoint.y()+12 >= self.curpos_r3.y() and self.lastPoint.y()+12 <= self.curpos_r3.y()+25 and robot3_overrideToggle is True:
                self.color_r3 = self.clicked
                self.move_r3 = True
                self.update()
            else:
                #print(self.lastPoint)
                self.posmap_mouse.setText(str(self.lastPoint.x()) + " , " + str(self.lastPoint.y()) + " (" + str(self.lastPoint.x() - offset_lapangan_x) + " , " + str(self.lastPoint.y()  - offset_lapangan_y) + ")")

    def mouseMoveEvent(self, event):
        self.lastPoint = event.pos()
        self.linePoint = event.pos()
        if event.buttons() and Qt.LeftButton and self.move_r1:
            robot1_offsetFromMap[0] = self.lastPoint.x()
            robot1_offsetFromMap[1] = self.lastPoint.y()
            #self.curpos_r1.setX(robot1_offsetFromMap[0])
            #self.curpos_r1.setY(robot1_offsetFromMap[1])
            self.update()

        elif event.buttons() and Qt.LeftButton and self.move_r2:
            robot2_offsetFromMap[0] = self.lastPoint.x()
            robot2_offsetFromMap[1] = self.lastPoint.y()
            #self.curpos_r2.setX(self.lastPoint.x())
            #self.curpos_r2.setY(self.lastPoint.y())
            self.update()

        elif event.buttons() and Qt.LeftButton and self.move_r3:
            robot3_offsetFromMap[0] = self.lastPoint.x()
            robot3_offsetFromMap[1] = self.lastPoint.y()
            #self.curpos_r3.setX(self.lastPoint.x())
            #self.curpos_r3.setY(self.lastPoint.y())
            self.update()

        elif event.buttons() and Qt.RightButton and self.rot_r1:
            robot1_offsetFromMap[2] = int(-np.degrees(np.arctan2(-robot1_offsetFromMap[1] + self.lastPoint.y() , -robot1_offsetFromMap[0] + self.lastPoint.x() )))
            #self.curpos_r1.setZ(robot1_offsetFromMap[2])
            self.update()

        elif event.buttons() and Qt.RightButton and self.rot_r2:
            robot2_offsetFromMap[2] = int(-np.degrees(np.arctan2(-robot2_offsetFromMap[1] + self.lastPoint.y() , -robot2_offsetFromMap[0] + self.lastPoint.x() )))

            #self.curpos_r2.setZ(-np.degrees(np.arctan2(-self.curpos_r2.y()+self.lastPoint.y(), -self.curpos_r2.x()+self.lastPoint.x())))
            self.update()

        elif event.buttons() and Qt.RightButton and self.rot_r3:
            robot3_offsetFromMap[2] = int(-np.degrees(np.arctan2(-robot3_offsetFromMap[1] + self.lastPoint.y() , -robot3_offsetFromMap[0] + self.lastPoint.x() )))

            #self.curpos_r3.setZ(-np.degrees(np.arctan2(-self.curpos_r3.y()+self.lastPoint.y(), -self.curpos_r3.x()+self.lastPoint.x())))
            self.update()
        else:
            self.posmap_mouse.setText(str(self.lastPoint.x()) + " , " + str(self.lastPoint.y()) + " (" + str(self.lastPoint.x() - offset_lapangan_x) + " , " + str(self.lastPoint.y()  - offset_lapangan_y) + ")")
            self.update()

    def mouseReleaseEvent(self, event):
        self.linePoint = QPoint(-5,-5)
        if event.button() == Qt.LeftButton :
            self.color_r1 = self.norm
            self.color_r2 = self.norm
            self.color_r3 = self.norm
            self.move_r1 = False
            self.move_r2 = False
            self.move_r3 = False
            #self.posmap_mouse.setText(str(self.lastPoint.x()) + " , " + str(self.lastPoint.y()))
            self.update()

        elif event.button() == Qt.RightButton :
            self.color_r1 = self.norm
            self.color_r2 = self.norm
            self.color_r3 = self.norm
            self.rot_r1 = False
            self.rot_r2 = False
            self.rot_r3 = False
            #self.posmap_mouse.setText(str(self.lastPoint.x()) + " , " + str(self.lastPoint.y()))
            self.update()

    def override_r1_handle(self, state):
        global robot1_overrideToggle
        if state == QtCore.Qt.Checked:
            robot1_overrideToggle = True
            robot1_offsetFromMap[0] = self.curpos_r1.x()
            robot1_offsetFromMap[1] = self.curpos_r1.y()
            robot1_offsetFromMap[2] = self.curpos_r1.z()
            robot1_command[0]=0
            robot1_command[1]=0
            robot1_command[2]=0
            robot1_command[3]=0

        else :
            robot1_overrideToggle = False
            robot1_offsetFromRobot[0] = robot1_subsstat[0]
            robot1_offsetFromRobot[1] = robot1_subsstat[1]
            robot1_offsetFromRobot[2] = robot1_subsstat[2]

    def override_r2_handle(self, state):
        global robot2_overrideToggle
        if state == QtCore.Qt.Checked:
            robot2_overrideToggle = True
            robot2_command[0]=0
            robot2_command[1]=0
            robot2_command[2]=0
            robot2_command[3]=0

        else :
            robot2_offsetFromMap[0] = self.curpos_r2.x()
            robot2_offsetFromMap[1] = self.curpos_r2.y()
            robot2_offsetFromMap[2] = self.curpos_r2.z()
            robot2_offsetFromRobot[0] = robot2_subsstat[0]
            robot2_offsetFromRobot[1] = robot2_subsstat[1]
            robot2_offsetFromRobot[2] = robot2_subsstat[2]
            robot2_overrideToggle = False


    def override_r3_handle(self, state):
        global robot3_overrideToggle
        if state == QtCore.Qt.Checked:
            #pass
            robot3_overrideToggle = True
            robot3_command[0]=0
            robot3_command[1]=0
            robot3_command[2]=0
            robot3_command[3]=0
        else :

            robot3_offsetFromMap[0] = self.curpos_r3.x()
            robot3_offsetFromMap[1] = self.curpos_r3.y()
            robot3_offsetFromMap[2] = self.curpos_r3.z()
            robot3_offsetFromRobot[0] = robot3_subsstat[0]
            robot3_offsetFromRobot[1] = robot3_subsstat[1]
            robot3_offsetFromRobot[2] = robot3_subsstat[2]
            robot3_overrideToggle = False

    def start_handle(self):
        global ownrefbox
        ownrefbox = 'Start'
    def stop_handle(self):
        global ownrefbox
        ownrefbox = 'Stop'
    def drop_handle(self):
        global ownrefbox
        ownrefbox = 'DropBall'
    def park_handle(self):
        global ownrefbox
        ownrefbox = 'Park'
    def reset_handle(self):
        global ownrefbox
        ownrefbox = 'Reset'
    def end_handle(self):
        global ownrefbox
        ownrefbox = 'EndPart'

    def goal_cyan_handle(self):
        global ownrefbox
        ownrefbox = 'Cyan Goal'
    def kickoff_cyan_handle(self):
        global ownrefbox
        ownrefbox = 'Cyan KickOff'
    def freekick_cyan_handle(self):
        global ownrefbox
        ownrefbox = 'Cyan FreeKick'
    def goalkick_cyan_handle(self):
        global ownrefbox
        ownrefbox = 'Cyan GoalKick'
    def throwin_cyan_handle(self):
        global ownrefbox
        ownrefbox = 'Cyan ThrowIn'
    def corner_cyan_handle(self):
        global ownrefbox
        ownrefbox = 'Cyan Corner'
    def penalty_cyan_handle(self):
        global ownrefbox
        ownrefbox = 'Cyan Penalty'
    def repair_cyan_handle(self):
        global ownrefbox
        ownrefbox = 'Cyan Repair'

    def goal_magenta_handle(self):
        global ownrefbox
        ownrefbox = 'Magenta Goal'
    def kickoff_magenta_handle(self):
        global ownrefbox
        ownrefbox = 'Magenta KickOff'
    def freekick_magenta_handle(self):
        global ownrefbox
        ownrefbox = 'Magenta FreeKick'
    def goalkick_magenta_handle(self):
        global ownrefbox
        ownrefbox = 'Magenta GoalKick'
    def throwin_magenta_handle(self):
        global ownrefbox
        ownrefbox = 'Magenta ThrowIn'
    def corner_magenta_handle(self):
        global ownrefbox
        ownrefbox = 'Magenta Corner'
    def penalty_magenta_handle(self):
        global ownrefbox
        ownrefbox = 'Magenta Penalty'
    def repair_magenta_handle(self):
        global ownrefbox
        ownrefbox = 'Magenta Repair'





if __name__=='__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = window()
    window.setWindowTitle('SORA 2019 BASESTATION - Universitas Negeri Malang')
    window.show()
    sys.exit(app.exec_())
