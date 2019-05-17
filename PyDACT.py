from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QSettings
import sys
import serial
import serial.tools.list_ports
from lib.serialSetting import *
from lib.eprConfig import *
serialPort =  serial.Serial()
serialFree = True

print(sys.path)



class manualControlTapUI(QWidget):

    def __init__(self):
        super(manualControlTapUI, self).__init__()
        mainVlayout = QVBoxLayout(self)
        self.homeAllBtn = QPushButton('Home All', self)

        mainVlayout.addWidget(self.homeAllBtn)
        verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        mainVlayout.addItem(verticalSpacer)
        self.setLayout(mainVlayout)

class mainWidget(QWidget):
    def __init__(self, parent,settings):
        self.settings = settings
        super().__init__(parent)
        self.IsSerialConnected = False
        self.setupUI()

    def serialSettingWidgetUpdate(self):
        status = self.IsSerialConnected
        self.serialSettingWidget.portName.setEnabled(not status)
        self.serialSettingWidget.buadRate.setEnabled(not status)
        self.serialSettingWidget.serialConnectBtn.setEnabled(not status)
        self.serialSettingWidget.serialDisconnectBtn.setEnabled(status)

    def serialConnectHandle(self):
        listPort = serial.tools.list_ports.comports(include_links=False)
        portSelect = self.serialSettingWidget.portName.currentText()
        portName = [x.device for x in listPort][[x.description for x in listPort].index(portSelect)]

        serialPort.port = portName

        serialPort.baudrate = int(self.serialSettingWidget.buadRate.currentText())
        try:
            serialPort.open()
        except Exception as e:
            print('Serial error',e)
            #self.serial_error_dialog()
            return
        self.IsSerialConnected = True
        self.serialSettingWidgetUpdate()
        global serialFree
        serialFree = True
        print('serialConnectHandle')
    def serialDisconnectHandle(self):
        serialPort.close()
        self.IsSerialConnected = False
        self.serialSettingWidgetUpdate()
        print('serialDisconnectHandle')
    def eprLoadBtnClicked(self):
        print('eprLoadBtnClicked')
        self.loadEprThread = loadEprThread(serialPort)
        self.loadEprThread.finishSignal.connect(self.eprUpdate)
        self.loadEprThread.start()
    def eprUpdate(self):
        print('eprUpdate')
        #print(self.loadEprThread.eprData)
        for i in range(91):
            splitStr = self.loadEprThread.eprData[i].split(' ')
            type = splitStr[0].replace('EPR:','')
            pos = splitStr[1]
            val = splitStr[2]
            description = ''.join(splitStr[3:])
            self.eprConfigTab.eprTable.setItem(i, 0, QTableWidgetItem(description))
            self.eprConfigTab.eprTable.setItem(i, 1, QTableWidgetItem(pos))
            self.eprConfigTab.eprTable.setItem(i, 2, QTableWidgetItem(type))
            self.eprConfigTab.eprTable.setItem(i, 3, QTableWidgetItem(val))

    def homeAllBtnClicked(self):
        print('Homing..')

    def setupUI(self):
        mainVlayout = QVBoxLayout(self)
        self.serialSettingWidget = serialSettingWidget()
        self.serialSettingWidgetUpdate()
        self.serialSettingWidget.serialConnectBtn.clicked.connect(self.serialConnectHandle)
        self.serialSettingWidget.serialDisconnectBtn.clicked.connect(self.serialDisconnectHandle)
        mainVlayout.addWidget( self.serialSettingWidget)
        mainTabWidget = QTabWidget()

        #main
        mainMenuTab = QWidget()
        mainTabWidget.addTab(mainMenuTab, "Main Menu")

        #serial monitor
        serialMonitorTab = QWidget()
        mainTabWidget.addTab(serialMonitorTab, "Serial Monitor")

        #EEPROM
        self.eprConfigTab = eprConfigTabUI()
        mainTabWidget.addTab(self.eprConfigTab, "EEPROM Config")
        self.eprConfigTab.eprLoadBtn.clicked.connect(self.eprLoadBtnClicked)

        #manual control
        self.manualControlTap = manualControlTapUI()
        mainTabWidget.addTab(self.manualControlTap, "Manual Control")
        self.manualControlTap.homeAllBtn.clicked.connect(self.homeAllBtnClicked)


        mainVlayout.addWidget(mainTabWidget)
        verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        #mainVlayout.addItem(verticalSpacer)

        self.setLayout(mainVlayout)
        pass

class mainWindow(QMainWindow):
    def __init__(self,settings ):
        self.settings = settings
        super(QMainWindow, self).__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 500, 600)
        self.setWindowTitle("PyDACT")
        self.setWindowIcon(QtGui.QIcon('logo.png'))

        self.mainWidget = mainWidget(self,self.settings)
        self.setCentralWidget( self.mainWidget)

    def closeEvent(self, event):
        if False:

            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)

            msg.setText("Are you sure you want to quit?")
            #msg.setInformativeText("Serial port still connected.")

            msg.setWindowTitle("Warning!")

            msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            retval = msg.exec_()
            if retval == QMessageBox.Ok:
                print('Exit confirm!')
                event.accept()
            else:
                event.ignore()
def main():
    app = QApplication(sys.argv)
    settings = QSettings('M Lab', 'Py DACT')
    w = mainWindow(settings)
    w.show()
    sys.exit(app.exec_())



if __name__ == '__main__':
    main()