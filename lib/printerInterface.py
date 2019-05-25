from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QSettings
import sys
import serial
import numpy as np


import time

def HommingFc(serialPort):
    print('port open?:',serialPort.isOpen())
    listPort = [x.device for x in serial.tools.list_ports.comports(include_links=False)]
    print(listPort)
    serialPort.flushOutput()
    while serialPort.inWaiting() > 0:
        serialPort.reset_input_buffer()
    serialPort.write(b'G28\r\n')
    count = 0
    startTime = time.time()
    while True:
        if time.time() - startTime > 5:
            print('timeout!')
            return True
        if serialPort.inWaiting() > 0:
            readByte = serialPort.readline()

            if b'\xb0' in readByte:
                readByte = readByte.replace(b'\xb0', b'\xc2\xb0')
            readStr = readByte.decode("utf-8")
            words = ''.join([x[0] for x in readStr.split(' ')])
            if words == 'XYZE':
                break
            elif 'busy' in readStr:
                print('printer busy!')
                print(time.time() - startTime)
                startTime = time.time()
    return False
class HomingThread(QtCore.QThread):
    finishSignal = pyqtSignal()
    error = False
    def __init__(self,serialPort):
        self.serialPort = serialPort
        QtCore.QThread.__init__(self)


    def __del__(self):
        self.wait()
    def run(self):
        self.error = HommingFc(self.serialPort)


        self.finishSignal.emit()

def cmdListSenderFc(serialPort,cmdList):
    serialPort.flushOutput()

    while serialPort.inWaiting() > 0:
        serialPort.reset_input_buffer()
    for cmd in cmdList:
        serialPort.write(cmd.encode("utf-8"))
        print('send:', cmd)
        while True:
            if serialPort.inWaiting() > 0:
                readByte = serialPort.readline()
                if b'\xb0' in readByte:
                    readByte = readByte.replace(b'\xb0', b'\xc2\xb0')

                readStr = readByte.decode("utf-8")
                if readStr.startswith('ok'):
                    break
class cmdListSenderThread(QtCore.QThread):
    finishSignal = pyqtSignal()
    error = False
    def __init__(self,serialPort,cmdList):
        self.serialPort = serialPort
        self.cmdList = cmdList
        QtCore.QThread.__init__(self)


    def __del__(self):
        self.wait()
    def run(self):
        cmdListSenderFc(self.serialPort, self.cmdList)

        print('complete!')




        self.finishSignal.emit()

class printerInterfaceUI(QWidget):



    def __init__(self, serialPort):
        self.serialPort = serialPort
        super(printerInterfaceUI, self).__init__()
        mainVlayout = QVBoxLayout(self)
        self.homeAllBtn = QPushButton('Home All', self)
        ####




        ####
        mainVlayout.addWidget(self.homeAllBtn)
        verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        mainVlayout.addItem(verticalSpacer)
        self.setLayout(mainVlayout)

        self.homeAllBtn.clicked.connect(self.homeAllBtnClicked)
    def homingComplete(self):
        if self.HomingThread.error == False:
            print('homingComplete')
        else:
            print('Homming Error!')
    def homeAllBtnClicked(self):
        print('homeAllBtnClicked')
        self.HomingThread = HomingThread(self.serialPort)
        self.HomingThread.finishSignal.connect(self.homingComplete)
        self.HomingThread.start()