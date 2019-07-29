# write by H. Dhamdhawach
# 27/7/2019
'''
PYDACT Core
(Python Delta Auto Calibration Tools)
Purpose of this program
1. To be core program or backend that easily to use with UI (front end) and easily to debug.

Function
1. Homming
2. Disable Bed leveling/z-correction
3. move and prob with specific area and distance
4. Read EEPROM to get kinematic data
5. Do forward/inverse kinematio of delta
6. Using optimization to solve to correct parameter

'''
import serial
import serial.tools.list_ports
import time
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
serialPort = serial.Serial()
def serial_connect(port,buad,timeout = 5):

    serialPort.port = port
    serialPort.baudrate = buad
    try:
        serialPort.open()
    except Exception as exception:
        print('Serial error,', exception)
        success = False
        return success
    success = True
    timeStamp = time.time()

    while True:
        if time.time() - timeStamp >= timeout:
            break
    serialPort.flushInput()
    return success

def ports_descrip():
    listPort = [port for port in serial.tools.list_ports.comports(include_links=False)]
    #listDevice = [port.device for port in listPort]
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
                return True
        if time.time() - startTime >= timeout:
            return False

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
                return True,res
            elif 'busy' in read_str:
                print('busy!')
                startTime = time.time()
        if time.time() - startTime >= timeout:
            return False,None

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
                return True,res
            elif 'busy' in read_str:
                print('busy!')
                startTime = time.time()
        if time.time() - startTime >= timeout:
            return False,None

def planar_probe(probe_height,probe_radius,probe_num):
    x_axis = np.linspace(-probe_radius,probe_radius,probe_num)
    y_axis = np.linspace(-probe_radius, probe_radius, probe_num)
    send_ok(b'G0 Z'+str(probe_height).encode("utf-8") + b'\r\n')
    res = []
    X, Y = np.meshgrid(x_axis, y_axis)
    Z1 = np.zeros(X.shape)
    for i_y in range(y_axis.size):
        for i_x in range(x_axis.size):
            Z1[i_y,i_x] = None
    for i_y in range(y_axis.size):
        y = y_axis[i_y]
        if i_y %2 == 0:
            range_x = range(0,x_axis.size)
        else:
            range_x = range(x_axis.size-1,-1,-1)
        for i_x in range_x:
            x = x_axis[i_x]
            if np.sqrt(np.square(x)+np.square(y)) <= probe_radius:
                msg = b'G0'
                msg += b' X' + str(x).encode("utf-8")
                msg += b' Y' + str(y).encode("utf-8")
                msg += b' Z' + str(probe_height).encode("utf-8")
                msg += b'\r\n'

                send_ok(msg)
                suc,probe_res = z_probe()
                # res in [x,y,z] format
                res.append([ probe_res['X'],probe_res['Y'],probe_res['Z-probe'] ])
                Z1[i_y, i_x] = probe_res['Z-probe']
    
    np_res = np.array(res)
    print(np_res.shape)
    plt.axes().set_aspect('equal')
    plt.contourf(x_axis, y_axis, Z1, 50, cmap='jet')
    plt.colorbar()
    plt.scatter(np_res[:,0],np_res[:,1],facecolors='none', edgecolors='k')
    plt.show()
    return  res
#### Serial connection
port = 'COM8'
buad = 250000
print('connecting serial.')
suc = serial_connect(port,buad)
if not suc:
    exit(0)

print('Serial connection success.')



#### Homing
suc,res = send_xyz(b'G28\r\n')
if not suc:
    exit(0)
print('Homing success',res)
### disable auto level and z_corr
print('disable auto level and z_corr')
suc = send_ok(b'M321\r\n')
print_serial_buffer()
if not suc:
    exit(0)
suc = send_ok(b'M323 S0\r\n')
print_serial_buffer()
if not suc:
    exit(0)
planar_probe_res = planar_probe(probe_height=5,probe_radius=70,probe_num=21)
suc,res = send_xyz(b'G28\r\n')
if not suc:
    exit(0)
print('Homing success',res)
print('success')

'''
    if serialPort.inWaiting() > 0:
        readByte = serialPort.readline()
        readByte = readByte.replace(b'\xb0', b'\xc2\xb0')
        print(readByte,time.time() - timeStamp)
'''