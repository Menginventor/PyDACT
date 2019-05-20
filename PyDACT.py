from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QSettings
import sys
import serial
import serial.tools.list_ports
from lib.serialSetting import *
from lib.eprConfig import *
from lib.printerInterface import *
serialPort =  serial.Serial()





class mainWidget(QWidget):
    def __init__(self, parent,settings):
        self.settings = settings
        super().__init__(parent)

        self.setupUI()




    def homeAllBtnClicked(self):
        print('Homing..')

    def setupUI(self):
        mainVlayout = QVBoxLayout(self)
        self.serialSettingWidget = serialSettingWidget(serialPort)

        mainVlayout.addWidget( self.serialSettingWidget)
        mainTabWidget = QTabWidget()

        #main
        mainMenuTab = QWidget()
        mainTabWidget.addTab(mainMenuTab, "Main Menu")

        #serial monitor
        serialMonitorTab = QWidget()
        mainTabWidget.addTab(serialMonitorTab, "Serial Monitor")

        #EEPROM
        self.eprConfigTab = eprConfigTabUI(serialPort)
        mainTabWidget.addTab(self.eprConfigTab, "EEPROM Config")


        #manual control
        self.printerInterface = printerInterfaceUI(serialPort)
        mainTabWidget.addTab(self.printerInterface, "Printer Interface")



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