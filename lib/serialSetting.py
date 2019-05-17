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

    def __init__(self):
        super(serialSettingWidget, self).__init__()
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


    def portNameUpdate(self):
        #print('portNameUpdate')
        listPort = serial.tools.list_ports.comports(include_links=False)
        #listPortName = [x.device for x in listPort]
        listPortDescription = [x.description for x in listPort]

        self.portName.clear()
        self.portName.addItems(listPortDescription)