'''
This is core program to finding kinematic parameter.

'''
from App.core_dfik import *
from scipy import  optimize
import numpy as np
import time
import serial
import serial.tools.list_ports
####INTERFACE####

def serial_connect(serialPort,port,buad,timeout = 5):
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



def ports_descrip(serialPort):
    listPort = [port for port in serial.tools.list_ports.comports(include_links=False)]
    listDescription = [port.description for port in listPort]
    return listDescription


def print_serial_buffer(serialPort):
    while serialPort.inWaiting() > 0:
        readByte = serialPort.readline().replace(b'\xb0', b'\xc2\xb0').replace(b'\r', b'')
        read_str = readByte.decode("utf-8")
        print(read_str)


def send_ok(serialPort,msg,timeout = 30):  # Send and wait retunr 'ok'
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

def send_xyz(serialPort,msg, timeout=30):  # G28
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


def z_probe(serialPort,timeout=30):  # G30, wait for 'Z-probe'
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


def planar_probe(serialPort,probe_height,probe_radius,probe_num):
    half_side = probe_radius*np.sin(np.pi/4.0)
    x_axis = np.linspace(-half_side,half_side,probe_num)
    y_axis = np.linspace(-half_side, half_side, probe_num)
    send_ok(serialPort,b'G0 Z'+str(probe_height).encode("utf-8") + b' F3600\r\n')
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

            send_ok(serialPort,msg)
            for i in range(3):
                probe_res = z_probe(serialPort)
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


def load_epr(serialPort):
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

def load_epr_param(serialPort):
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
    epr_data = load_epr(serialPort)

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



def homing(serialPort):
    send_xyz(serialPort,b'G28\r\n')

def deactivateAutoLevel(serialPort):
    send_ok(serialPort,b'M321\r\n')

def disableDistortionCorr(serialPort):
    send_ok(serialPort,b'M323 S0\r\n')

####INTERFACE####
def probe_point_config(probe_height,probe_radius,probe_num):
    x_axis = np.linspace(-probe_radius,probe_radius,probe_num)
    y_axis = np.linspace(-probe_radius, probe_radius, probe_num)
    points = np.array( [[x,y,probe_height] for x in x_axis for y in y_axis if x**2 + y**2 <= probe_radius**2] )
    return points

def plane_fit_cost(opt_var, points):
    a_x = opt_var[0]
    a_y = opt_var[1]
    z_pos = opt_var[2]
    plane = paz_plane(np.array([0,0,z_pos]),a_x,a_y)
    dist_ssq = 0
    plane_vec = plane[0:3]
    d = plane[3]
    for i in range(points.shape[0]):
        dist_ssq += np.square(np.dot(plane_vec[:],points[i,:]) + d)
    return dist_ssq


def plane_fit(points):
    sol = optimize.minimize(plane_fit_cost, x0=np.array([0,0,0]),args=points)
    print(sol)
    a_x = sol.x[0]
    a_y = sol.x[1]
    z_pos = sol.x[2]
    return z_pos,a_x,a_y


def param_est_cost_2(var,*argv):
    init_param = argv[0]
    joints_pos = argv[1]
    radius = var[0]

    alpha_a = var[1]
    alpha_b = var[2]

    offsetA = var[3]
    offsetB = var[4]
    offsetC = var[5]
    a_x = var[6]
    a_y = var[7]

    diag_len = init_param.diag_len
    param = dk_param(radius,radius,radius,alpha_a,alpha_b,np.pi/2,offsetA,offsetB,offsetC,diag_len)
    end_pos = DFK(param, joints_pos)
    plane = paz_plane(np.array([0, 0, 0]), a_x, a_y)
    dist_ssq = 0
    plane_vec = plane[0:3]
    d = plane[3]
    plane_r_mat = np.transpose(np.matmul(rmat_x(a_x), rmat_y(a_y)))
    rot_end_pos = np.array([np.matmul(plane_r_mat, end_pos[i, :]) for i in range(end_pos.shape[0])])
    for i in range(end_pos.shape[0]):
        dist_ssq += np.square(rot_end_pos[i,2])
    print(dist_ssq)
    return dist_ssq
def param_cost(init_param,joint_pos):
    x0 = np.array([init_param.ra,init_param.aa,
          init_param.ab,init_param.offsetA,init_param.offsetB,init_param.offsetC,0,0])
    return param_est_cost_2(x0,init_param,joint_pos)
def param_est_2(init_param,joint_pos):
    reduce_alphaA, reduce_alphaB, reduce_radius = init_param.reduce()
    x0 = np.array([reduce_radius,reduce_alphaA,
                   reduce_alphaB,init_param.offsetA,init_param.offsetB,init_param.offsetC,0,0])
    bounds = [(None,None) for i in range(8)]

    bounds[0] = (60,150)
    bounds[1] = (np.deg2rad(210-10), np.deg2rad(210+10))
    bounds[2] = (np.deg2rad(330 - 10), np.deg2rad(330 + 10))
    bounds[3] = (-1000, 1000)
    bounds[4] = (-1000, 1000)
    bounds[5] = (-1000, 1000)
    bounds[6] = (np.deg2rad(-2),np.deg2rad(2))
    bounds[7] = (np.deg2rad(-2), np.deg2rad(2))

    method = 'SLSQP'
    sol = optimize.minimize(param_est_cost_2,bounds=bounds,x0=x0,args=(init_param,joint_pos),method=method)

    print(sol)
    var = sol.x
    radius = var[0]

    alpha_a = var[1]
    alpha_b = var[2]

    offsetA = var[3]
    offsetB = var[4]
    offsetC = var[5]
    a_x = var[6]
    a_y = var[7]

    diag_len = init_param.diag_len
    param = dk_param(radius,radius,radius,alpha_a,alpha_b,np.pi/2,offsetA,offsetB,offsetC,diag_len)

    return param,(a_x, a_y),sol

if __name__ == '__main__':
    pass