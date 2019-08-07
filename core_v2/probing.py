import serial
import serial.tools.list_ports
import time
import numpy as np
from core_v2.core_dfik import *
import yaml
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

serialPort = serial.Serial()


def serial_connect(port,buad,timeout = 5):
    serialPort.port = port
    serialPort.baudrate = buad
    serialPort.open()

    timeStamp = time.time()
    while True:
        if time.time() - timeStamp >= timeout:
            break
        elif serialPort.inWaiting() > 0:
            readByte = serialPort.readline().replace(b'\xb0', b'\xc2\xb0').replace(b'\r', b'')
            read_str = readByte.decode("utf-8")
            print(read_str)



def ports_descrip():
    listPort = [port for port in serial.tools.list_ports.comports(include_links=False)]
    listDescription = [port.description for port in listPort]
    return listDescription


def print_serial_buffer():
    while serialPort.inWaiting() > 0:
        readByte = serialPort.readline().replace(b'\xb0', b'\xc2\xb0').replace(b'\r', b'')
        read_str = readByte.decode("utf-8")
        print(read_str)


def send_ok(msg,timeout = 30):  # Send and wait retunr 'ok'
    serialPort.write(msg)
    startTime = time.time()
    while True:
        if serialPort.inWaiting() > 0:
            readByte = serialPort.readline().replace(b'\xb0', b'\xc2\xb0').replace(b'\r', b'')
            read_str = readByte.decode("utf-8")
            if 'ok' in read_str:
                break
            else:
                print(read_str)
        if time.time() - startTime >= timeout:
            raise Exception('time-out!')

def send_xyz(msg, timeout=30):  # G28
    serialPort.write(msg)
    startTime = time.time()
    while True:
        if serialPort.inWaiting() > 0:
            readByte = serialPort.readline().replace(b'\xb0', b'\xc2\xb0').replace(b'\r', b'')
            read_str = readByte.decode("utf-8")
            xyze = ''.join([x[0] for x in read_str.split(' ')])
            if xyze == 'XYZE':
                res = {}
                for w in read_str.split(' '):
                    ws = w.split(':')
                    label = ws[0]
                    val = float(ws[1])
                    res[label] = val
                return res
            elif 'busy' in read_str:
                print('busy!')
                startTime = time.time()
        if time.time() - startTime >= timeout:
            raise Exception('time-out!')


def z_probe(timeout=30):  # G30, wait for 'Z-probe'
    serialPort.write(b'G30\r\n')
    startTime = time.time()
    while True:
        if serialPort.inWaiting() > 0:
            readByte = serialPort.readline().replace(b'\xb0', b'\xc2\xb0').replace(b'\r', b'')
            read_str = readByte.decode("utf-8")
            words = [x for x in read_str.split(' ')]
            z_probe_word = words[0].split(':')[0]

            if z_probe_word == 'Z-probe':
                res = {}
                for w in read_str.split(' '):
                    ws = w.split(':')
                    label = ws[0]
                    val = float(ws[1])
                    res[label] = val
                return res
            elif 'busy' in read_str:
                print('busy!')
                startTime = time.time()
            else:
                print(read_str)
        if time.time() - startTime >= timeout:
            raise Exception('time-out!')


def planar_probe(probe_height,probe_radius,probe_num):
    half_side = probe_radius*np.sin(np.pi/4.0)
    x_axis = np.linspace(-half_side,half_side,probe_num)
    y_axis = np.linspace(-half_side, half_side, probe_num)
    send_ok(b'G0 Z'+str(probe_height).encode("utf-8") + b' F3600\r\n')
    res = []
    X, Y = np.meshgrid(x_axis, y_axis)

    probe_num = x_axis.size*y_axis.size
    probe_count = 0
    for i_y in range(y_axis.size):
        y = y_axis[i_y]
        if i_y %2 == 0:
            range_x = range(0,x_axis.size)
        else:
            range_x = range(x_axis.size-1,-1,-1)
        for i_x in range_x:
            x = x_axis[i_x]

            probe_count += 1
            print('probing', probe_count, 'of', probe_num)

            msg = b'G0'
            msg += b' X' + str(x).encode("utf-8")
            msg += b' Y' + str(y).encode("utf-8")
            msg += b' Z' + str(probe_height).encode("utf-8")
            msg += b'\r\n'

            send_ok(msg)
            for i in range(3):
                probe_res = z_probe()
                # res in [x,y,z] format
                res.append([ probe_res['X'],probe_res['Y'],probe_height-probe_res['Z-probe'] ])
                ####


    np_res = np.array(res)

    return  np_res


def load_planar_probe():
    return np.load('height_map.npy')


class eprItem():
    def __init__(self,pos,type,val,description):
        self.description = description
        self.pos = pos
        self.type = type
        self.val = val

    def __str__(self):
        return 'val = '+str(self.val)+', pos = '+str(self.pos)+', description = '+str(self.description)


def load_epr():
    serialPort.write(b'M205\r\n')
    eprData =[]
    startTime = time.time()
    count = 0
    while count < 91:
        if time.time() - startTime > 5:
            print('epr read timeout!')
            print('count=',count)
            raise Exception('time-out!')
        if serialPort.inWaiting() > 0:

            readByte = serialPort.readline()

            if b'\xb0' in readByte:
                readByte = readByte.replace(b'\xb0', b'\xc2\xb0')

            readStr = readByte.decode("utf-8")

            if readStr.startswith('EPR:'):
                startTime = time.time()
                splitStr = readStr.split(' ')
                epr_type = int(splitStr[0].replace('EPR:', ''))
                epr_pos = int(splitStr[1])
                if epr_type == 3:
                    epr_val = float(splitStr[2])
                else:
                    epr_val = int(splitStr[2])
                epr_description = ''.join(splitStr[3:])

                eprData.append(eprItem(epr_pos,epr_type,epr_val,epr_description))
                count += 1

    return eprData

def load_epr_param():
    epr_addr_dict = {
        'step_per_mm': 11,
        'diag_length': 881,
        'horiz_radius': 885,
        'A_offset': 893,
        'B_offset': 895,
        'C_offset': 897,
        'A_alpha': 901,
        'B_alpha': 905,
        'C_alpha': 909,
        'A_radius': 913,
        'B_radius': 917,
        'C_radius': 921,
    }
    epr_data = load_epr()

    #
    epr_data_dict = {}
    for epr_key in epr_addr_dict.keys():
        for epr in epr_data:
            if epr.pos == epr_addr_dict[epr_key]:
                epr_data_dict[epr_key] = epr.val
                break
        else:
            print('cannot find epr', epr_key)
    return epr_data_dict



def homing():
    send_xyz(b'G28\r\n')

def deactivateAutoLevel():
    send_ok(b'M321\r\n')

def disableDistortionCorr():
    send_ok(b'M323 S0\r\n')

if __name__ == '__main__':
    port = 'COM8'
    buad = 250000
    print('connecting serial.')
    serial_connect(port, buad)
    # Load epr kinematic param
    epr_param = load_epr_param()
    epr_kinematic_param = dict_to_dk_param(epr_param)
    yaml.dump(epr_param, open("epr_dkp.yaml", "w"))
    print(epr_kinematic_param)
    #
    homing()
    deactivateAutoLevel()
    disableDistortionCorr()
    #
    probing = True
    if probing:
        height_map = planar_probe(probe_height=10, probe_radius=70, probe_num=7)
        np.save('probe_point.npy', height_map)
    else:
        height_map = np.load('probe_point.npy')
    #
    homing()



    joint_pos = DIK(epr_kinematic_param, height_map)

    np.save('probe_joint_pos.npy', joint_pos)
    ####
