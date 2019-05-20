from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QSettings
import serial
import serial.tools.list_ports
class ComboBox(QComboBox):
    popupAboutToBeShown = QtCore.pyqtSignal()
    def showPopup(self):
        self.popupAboutToBeShown.emit()
        super(ComboBox, self).showPopup()

class serialSettingWidget(QWidget):

    def __init__(self,serialPort):
        super(serialSettingWidget, self).__init__()
        self.serialPort = serialPort
        self.IsSerialConnected = False
        mainLayout = QHBoxLayout()
        self.GBox = QGroupBox("Serial Setting")
        self.portName = ComboBox()
        self.buadRate = QComboBox()
        self.portName.popupAboutToBeShown.connect(self.portNameUpdate)
        self.portNameUpdate()
        self.buadRate.clear()
        self.buadRate.addItems(['250000'])
        self.serialConnectBtn = QPushButton('Connect', self)
        self.serialDisconnectBtn = QPushButton('Disconnect', self)

        mainLayout.addWidget(self.portName)
        mainLayout.addWidget(self.buadRate)
        mainLayout.addWidget(self.serialConnectBtn)
        mainLayout.addWidget(self.serialDisconnectBtn)
        self.GBox.setLayout(mainLayout)

        mainVLayout = QVBoxLayout()
        mainVLayout.addWidget(self.GBox)
        self.setLayout(mainVLayout)

        ##
        self.update()
        self.serialConnectBtn.clicked.connect(self.serialConnectHandle)
        self.serialDisconnectBtn.clicked.connect(self.serialDisconnectHandle)


    def portNameUpdate(self):
        #print('portNameUpdate')
        listPort = serial.tools.list_ports.comports(include_links=False)
        #listPortName = [x.device for x in listPort]
        listPortDescription = [x.description for x in listPort]

        self.portName.clear()
        self.portName.addItems(listPortDescription)
    def update(self):
        status = self.IsSerialConnected
        self.portName.setEnabled(not status)
        self.buadRate.setEnabled(not status)
        self.serialConnectBtn.setEnabled(not status)
        self.serialDisconnectBtn.setEnabled(status)

    def serialConnectHandle(self):
        listPort = serial.tools.list_ports.comports(include_links=False)
        portSelect = self.portName.currentText()
        portName = [x.device for x in listPort][[x.description for x in listPort].index(portSelect)]

        self.serialPort.port = portName

        self.serialPort.baudrate = int(self.buadRate.currentText())
        try:
            self.serialPort.open()
        except Exception as e:
            print('Serial error',e)
            #self.serial_error_dialog()
            return
        self.IsSerialConnected = True
        self.update()
        print('serialConnectHandle')

    def serialDisconnectHandle(self):
        self.serialPort.close()
        self.IsSerialConnected = False
        self.update()
        print('serialDisconnectHandle')