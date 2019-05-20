from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QSettings
import sys
import serial

def HommingFc(serialPort):
    serialPort.flushOutput()
    while serialPort.inWaiting() > 0:
        serialPort.reset_input_buffer()
    serialPort.write(b'G28\r\n')
    count = 0
    while True:
        if serialPort.inWaiting() > 0:
            readByte = serialPort.readline()

            if b'\xb0' in readByte:
                readByte = readByte.replace(b'\xb0', b'\xc2\xb0')
            readStr = readByte.decode("utf-8")
            words = ''.join([x[0] for x in readStr.split(' ')])
            if words == 'XYZE':
                break
class HomingThread(QtCore.QThread):
    finishSignal = pyqtSignal()
    error = False
    def __init__(self,serialPort):
        self.serialPort = serialPort
        QtCore.QThread.__init__(self)


    def __del__(self):
        self.wait()
    def run(self):
        HommingFc(self.serialPort)


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

        mainVlayout.addWidget(self.homeAllBtn)
        verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        mainVlayout.addItem(verticalSpacer)
        self.setLayout(mainVlayout)

        self.homeAllBtn.clicked.connect(self.homeAllBtnClicked)
    def homingComplete(self):
        print('homingComplete')
    def homeAllBtnClicked(self):
        print('homeAllBtnClicked')
        self.HomingThread = HomingThread(self.serialPort)
        self.HomingThread.finishSignal.connect(self.homingComplete)
        self.HomingThread.start()