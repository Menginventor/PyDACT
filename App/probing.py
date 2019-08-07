'''
Run this program first to get eeprom data from printer and do probing.
You need to config serial port and connect to your printer
'''
import serial
from App.pydact_core import *
import numpy as np
from App.core_dfik import *
import yaml
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

serialPort = serial.Serial()


if __name__ == '__main__':
    port = 'COM8'  # you need to config port
    buad = 250000
    print('connecting serial.')
    serial_connect(serialPort,port, buad)
    # Load epr kinematic param
    epr_param = load_epr_param(serialPort)
    epr_kinematic_param = dict_to_dk_param(epr_param)
    yaml.dump(epr_param, open("epr_dkp.yaml", "w"))
    print(epr_kinematic_param)
    #
    homing(serialPort)
    deactivateAutoLevel(serialPort)
    disableDistortionCorr(serialPort)
    #
    probing = True
    if probing:
        height_map = planar_probe(serialPort,probe_height=10, probe_radius=70, probe_num=5)
        np.save('probe_point.npy', height_map)
    else:
        height_map = np.load('probe_point.npy')
    #
    homing(serialPort)



    joint_pos = DIK(epr_kinematic_param, height_map)

    np.save('probe_joint_pos.npy', joint_pos)
    ####
