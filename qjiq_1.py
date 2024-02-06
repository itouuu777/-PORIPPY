import numpy as np
import time
from smbus import SMBus
import math
from datetime import datetime


ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42
i2c = SMBus(1)

def makecontent():
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    content = f"[{current_time}]"
    return content


def bmx_setup():
    
    i2c.write_byte_data(ACCL_ADDR, 0x0F, 0x03)
    i2c.write_byte_data(ACCL_ADDR, 0x10, 0x08)
    i2c.write_byte_data(ACCL_ADDR, 0x11, 0x00)
    time.sleep(0.5)
    
    i2c.write_byte_data(GYRO_ADDR, 0x0F, 0x04)
    i2c.write_byte_data(GYRO_ADDR, 0x10, 0x07)
    i2c.write_byte_data(GYRO_ADDR, 0x11, 0x00)
    time.sleep(0.5)
    
    data = i2c.read_byte_data(MAG_ADDR, 0x4B)
    if(data == 0):
        i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
        time.sleep(0.5)
    i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
    i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
    i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
    i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
    i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
    time.sleep(0.5)


def acc_value():
    data = [0, 0, 0, 0, 0, 0]
    acc_data = [0.0, 0.0, 0.0]
    try:
        for i in range(6):
            data[i] = i2c.read_byte_data(ACCL_ADDR, ACCL_R_ADDR + i) 
        for i in range(3): 
            acc_data[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16 
            if acc_data[i] > 2047: 
                acc_data[i] -= 4096
            acc_data[i] *= 0.0098 
    except IOError as e: 
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return acc_data


def gyro_value():
    data = [0, 0, 0, 0, 0, 0]
    gyro_data = [0.0, 0.0, 0.0]
    try:
        for i in range(6):
            data[i] = i2c.read_byte_data(GYRO_ADDR, GYRO_R_ADDR + i)
        for i in range(3):
            gyro_data[i] = (data[2*i + 1] * 256) + data[2*i]
            if gyro_data[i] > 32767:
                gyro_data[i] -= 65536
            gyro_data[i] *= 0.0038
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return gyro_data


def mag_value():
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    mag_data = [0.0, 0.0, 0.0]
    try:
        for i in range(8):
            data[i] = i2c.read_byte_data(MAG_ADDR, MAG_R_ADDR + i)
        for i in range(3):
            if i != 2:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
                if mag_data[i] > 4095:
                    mag_data[i] -= 8192
            else:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
                if mag_data[i] > 16383:
                    mag_data[i] -= 32768
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return mag_data

def cyakuchi_hantei():
    while True:
        b1=gyro_value()
        time.sleep(0.5)
        b2=gyro_value()
        time.sleep(0.5)
        b3=gyro_value()
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'data'+','+'angular velocity'+','+str(b1[0])+'\n')
            file.write(makecontent()+','+'data'+','+'angular velocity'+','+str(b2[0])+'\n')
            file.write(makecontent()+','+'data'+','+'angular velocity'+','+str(b3[0])+'\n')
        if abs(b1[0])<1.0 and abs(b2[0])<1.0 and abs(b3[0])<1.0:
            cyakuchi=1
            break
        else:
            cyakuchi=0
    return(cyakuchi)

bmx_setup()
cyakuchi=0
while True:
    cyakuchi=cyakuchi_hantei()
    if cyakuchi==1:
        print("gennchaku!")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+"houshutukannryo!"+'\n')
            break
    else:
        print("madayade")
