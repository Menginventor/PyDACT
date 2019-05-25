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

class uploadEprThread(QtCore.QThread):
    finishSignal = pyqtSignal()
    error = False
    def __init__(self,serialPort,cmdList):
        self.serialPort = serialPort
        self.cmdList = cmdList
        QtCore.QThread.__init__(self)


    def __del__(self):
        self.wait()
    def run(self):
        self.serialPort.flushOutput()

        while  self.serialPort.inWaiting() > 0:
            self.serialPort.reset_input_buffer()
        for cmd in self.cmdList:
            self.serialPort.write(cmd.encode("utf-8"))
            print('send:',cmd)
            while True:
                if self.serialPort.inWaiting() > 0:
                    readByte = self.serialPort.readline()
                    if b'\xb0' in readByte:
                        readByte = readByte.replace(b'\xb0', b'\xc2\xb0')

                    readStr = readByte.decode("utf-8")
                    if readStr.startswith('ok'):
                        break
        print('complete!')




        self.finishSignal.emit()

class eprItem():
    def __init__(self,description,pos,type,val):
        self.description = description
        self.pos = pos
        self.type = type
        self.val = val
class eprConfigTabUI(QWidget):
    def __init__(self,serialPort):
        super(eprConfigTabUI, self).__init__()
        mainVlayout = QVBoxLayout(self)
        self.serialPort = serialPort
        self.eprLoadBtn = QPushButton('Load EEPROM', self)
        self.eprUpdateBtn = QPushButton('Upload EEPROM', self)
        mainVlayout.addWidget(self.eprLoadBtn)
        mainVlayout.addWidget(self.eprUpdateBtn)
        self.eprTable = QTableWidget()
        self.eprTable.setRowCount(91)
        self.eprTable.setColumnCount(4)
        self.eprTable.setHorizontalHeaderLabels(['Name','Position','Type','Value'])
        self.eprTable.setColumnWidth(0,250)
        mainVlayout.addWidget(self.eprTable)
        self.setLayout(mainVlayout)
        self.eprLoadBtn.clicked.connect(self.eprLoadBtnClicked)
        self.eprUpdateBtn.clicked.connect(self.eprUpdateBtnClicked)
        self.eprValDict = {}
    def eprLoadBtnClicked(self):
        print('eprLoadBtnClicked')
        self.loadEprThread = loadEprThread(self.serialPort)
        self.loadEprThread.finishSignal.connect(self.eprUpdate)
        self.loadEprThread.start()
    def eprUpdateBtnClicked(self):
        print('eprUpdateBtnClicked')

        updateEprCmd = []
        eprHasChange = False
        for row in range (self.eprTable.rowCount()):
            discription = self.eprTable.item(row,0).text()
            pos =  self.eprTable.item(row,1).text()
            type =  self.eprTable.item(row,2).text()
            val =  self.eprTable.item(row,3).text()
            '''
                    case 0:
            if(com->hasS())
                HAL::eprSetByte(com->P, (uint8_t)com->S);
            break;
        case 1:
            if(com->hasS())
                HAL::eprSetInt16(com->P, (int16_t)com->S);
            break;
        case 2:
            if(com->hasS())
                HAL::eprSetInt32(com->P, (int32_t)com->S);
            break;
        case 3:
            if(com->hasX())
                HAL::eprSetFloat(com->P, com->X);
            break;
            '''
            if self.eprValDict[pos].val != val:
                eprHasChange = True
                print('Change at',pos)

                cmd = 'M206 T'+self.eprValDict[pos].type+' P'+pos
                if self.eprValDict[pos].type == '0':
                    cmd += ' S'
                elif self.eprValDict[pos].type == '1':
                    cmd += ' S'
                elif self.eprValDict[pos].type == '2':
                    cmd += ' S'
                elif self.eprValDict[pos].type == '3':
                    cmd += ' X'
                else:
                    print('error type mismatch')
                cmd += val
                cmd += '\r\n'

                updateEprCmd.append(cmd)

        self.uploadEprThread = uploadEprThread(self.serialPort,updateEprCmd)
        #self.uploadEprThread.finishSignal.connect(self.eprUpdate)
        self.uploadEprThread.start()


    def eprUpdate(self):
        print('eprUpdate')
        #print(self.loadEprThread.eprData)
        for i in range(91):
            splitStr = self.loadEprThread.eprData[i].split(' ')
            type = splitStr[0].replace('EPR:','')
            pos = splitStr[1]
            val = splitStr[2]
            description = ''.join(splitStr[3:])

            descriptionItem = QTableWidgetItem(description)
            descriptionItem.setFlags( QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsEnabled)

            posItem = QTableWidgetItem(pos)
            posItem.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)

            typeItem = QTableWidgetItem(type)
            typeItem.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)

            self.eprTable.setItem(i, 0, descriptionItem)
            self.eprTable.setItem(i, 1, posItem)
            self.eprTable.setItem(i, 2, typeItem)
            self.eprTable.setItem(i, 3, QTableWidgetItem(val))

            epr = eprItem(description = description,pos = pos,type = type,val = val)
            self.eprValDict[pos] = epr

