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
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from core.pydact_math import *
from core.core_DIFK import *
from scipy import  optimize
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
    probe_num = 0
    for i_y in range(y_axis.size):
        for i_x in range(x_axis.size):
            Z1[i_y,i_x] = None
            if np.sqrt(np.square(x_axis[i_x]) + np.square(y_axis[i_y])) <= probe_radius:
                probe_num += 1
    probe_count = 0
    for i_y in range(y_axis.size):
        y = y_axis[i_y]
        if i_y %2 == 0:
            range_x = range(0,x_axis.size)
        else:
            range_x = range(x_axis.size-1,-1,-1)
        for i_x in range_x:
            x = x_axis[i_x]
            if np.sqrt(np.square(x)+np.square(y)) <= probe_radius:
                probe_count += 1
                print('probing', probe_count, 'of', probe_num)

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
    np.save('planar_probe.npy', np_res)
    return  np_res

def load_planar_probe():
    return np.load('planar_probe.npy')
class eprItem():
    def __init__(self,pos,type,val,description):
        self.description = description
        self.pos = pos
        self.type = type
        self.val = val
    def __str__(self):
        return 'val = '+str(self.val)+', pos = '+str(self.pos)+', description = '+str(self.description)
def load_epr():
    print('load epr')
    serialPort.write(b'M205\r\n')
    eprData =[]
    startTime = time.time()
    count = 0
    while count < 91:
        if time.time() - startTime > 5:
            print('epr read timeout!')
            print('count=',count)
            return False,None
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

    return True,eprData

# calib
def calib_cost(est_dkp,diag_length,joints):
    #print(len(args))
    #diag_length = args[0]
    #joints = args[1]
    points = DFK(delta_kinematic_param.init_argv(est_dkp,diag_length), joints)
    plane_fit_res, plane_fit_cost = plane_fit(points)
    print(plane_fit_cost)
    return plane_fit_cost

def delta_calib(init_dkp,joints):

    sol = optimize.minimize(calib_cost,init_dkp.argv(),args=(init_dkp.diag_length,joints),method='trust-constr')




    print(sol)
    return sol.success,sol.x
#### Serial connection
'''
EPR:3 11 100.0000 Steps per mm
EPR:3 881 217.000 Diagonal rod length [mm]
EPR:3 885 94.500 Horizontal rod radius at 0,0 [mm]

EPR:1 893 0 Tower X endstop offset [steps]
EPR:1 895 90 Tower Y endstop offset [steps]
EPR:1 897 61 Tower Z endstop offset [steps]
EPR:3 901 210.000 Alpha A(210):
EPR:3 905 330.000 Alpha B(330):
EPR:3 909 90.000 Alpha C(90):
EPR:3 913 0.000 Delta Radius A(0):
EPR:3 917 0.000 Delta Radius B(0):
EPR:3 921 0.000 Delta Radius C(0):


'''
epr_addr_dict = {
    'step_per_mm':11,
    'diag_length':881,
    'horiz_radius':885,
    'A_offset':893,
    'B_offset':895,
    'C_offset':897,
    'A_alpha':901,
    'B_alpha':905,
    'C_alpha':909,
    'A_radius':913,
    'B_radius':917,
    'C_radius':921,

}
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

### disable auto level and z_corr

suc = send_ok(b'M321\r\n')
print_serial_buffer()
if not suc:
    exit(0)
suc = send_ok(b'M323 S0\r\n')
print_serial_buffer()
if not suc:
    exit(0)
#planar_probe_res = planar_probe(probe_height=5,probe_radius=70,probe_num=15)
planar_probe_res = load_planar_probe()
plane_fit_res,plane_fit_cost = plane_fit(planar_probe_res)
plane_error = plane_fit_error(plane_fit_res,planar_probe_res)
print('plane_fit_error', plane_error)
print('plane_fit_cost', plane_fit_cost)



suc,res = send_xyz(b'G28\r\n')
if not suc:
    exit(0)
suc,epr_data = load_epr()


#
epr_data_dict = {}
for epr_key in epr_addr_dict.keys():
    for epr in epr_data:
        if epr.pos == epr_addr_dict[epr_key]:
            epr_data_dict[epr_key] = epr.val
            break
    else:
        print('cannot find epr',epr_key)
print(epr_data_dict)
diag_length = epr_data_dict['diag_length']
radius_a = epr_data_dict['horiz_radius'] + epr_data_dict['A_radius']
radius_b = epr_data_dict['horiz_radius'] + epr_data_dict['B_radius']
radius_c = epr_data_dict['horiz_radius'] + epr_data_dict['C_radius']
alpha_a = epr_data_dict['A_alpha']
alpha_b = epr_data_dict['B_alpha']
alpha_c = epr_data_dict['C_alpha']
offset_a = epr_data_dict['A_offset']
offset_b = epr_data_dict['B_offset']
offset_c = epr_data_dict['C_offset']
dk_param = delta_kinematic_param.init_form_printer(diag_length, radius_a, radius_b, radius_c,
                                                   alpha_a, alpha_b, alpha_c,offset_a,offset_b,offset_c)
print(dk_param)
IK_joint = DIK(dk_param,planar_probe_res)
FK_end = DFK(dk_param,IK_joint)

suc,est_param = delta_calib(dk_param,IK_joint)

est_dk_param = delta_kinematic_param.init_argv(est_param,dk_param.diag_length)
print('alpha_a',np.rad2deg(est_dk_param.alpha_a))
print('alpha_b',np.rad2deg(est_dk_param.alpha_b))
print('success')

new_points = DFK(est_dk_param, IK_joint)
est_plane_fit_res,est_plane_fit_cost = plane_fit(new_points)
plane_error = plane_fit_error(est_plane_fit_res,new_points)
print('est_plane_fit_error', plane_error)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x_axis = np.linspace(-70,70,7)
y_axis = np.linspace(-70,70,7)
X, Y = np.meshgrid(x_axis, y_axis)
Z = (-plane_fit_res[3]-plane_fit_res[0]*X-plane_fit_res[1]*Y)/plane_fit_res[2]
ax.plot_wireframe(X, Y, Z)
ax.scatter(planar_probe_res[:,0], planar_probe_res[:,1], planar_probe_res[:,2],color='r')
ax.scatter(new_points[:,0], new_points[:,1], new_points[:,2],color='b')
plt.show()
