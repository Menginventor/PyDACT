from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QSettings
import sys
import serial

class loadEprThread(QtCore.QThread):
    finishSignal = pyqtSignal()
    error = False
    def __init__(self,serialPort):
        self.serialPort = serialPort
        QtCore.QThread.__init__(self)

    def __del__(self):
        self.wait()
    def run(self):
        self.serialPort.flushOutput()
        while  self.serialPort.inWaiting() > 0:
            self.serialPort.reset_input_buffer()

            self.serialPort.write(b'M205\r\n')
        count = 0
        self.eprData = []
        while count <91:
            if self.serialPort.inWaiting()>0:
                readByte = self.serialPort.readline()

                if b'\xb0' in readByte:
                    readByte=readByte.replace( b'\xb0', b'\xc2\xb0')



                readStr = readByte.decode("utf-8")
                if readStr.startswith('EPR:'):
                    self.eprData.append(readStr)
                    count+=1

                    #print(readStr,'connt = ',count)

        self.finishSignal.emit()



class eprConfigTabUI(QWidget):
    def __init__(self):
        super(eprConfigTabUI, self).__init__()
        mainVlayout = QVBoxLayout(self)
        self.eprLoadBtn = QPushButton('Load EEPROM', self)
        mainVlayout.addWidget(self.eprLoadBtn)
        self.eprTable = QTableWidget()
        self.eprTable.setRowCount(91)
        self.eprTable.setColumnCount(4)
        self.eprTable.setHorizontalHeaderLabels(['Name','Position','Type','Value'])
        mainVlayout.addWidget(self.eprTable)
        self.setLayout(mainVlayout)

