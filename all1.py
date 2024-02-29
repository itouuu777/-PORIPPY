import serial
import pynmea2
import time
from geopy.distance import geodesic 
from datetime import datetime
import math
import numpy as np
from smbus import SMBus
import time
import RPi.GPIO as GPIO
import wiringpi as pi
from picamera2 import Picamera2
from libcamera import controls
import cv2
from typing import Union, List, Tuple
import pigpio
from pprint import pprint
from collections import OrderedDict
import enum

ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42
i2c = SMBus(1)



AIN1 = 5
AIN2 = 6
PWMA = 13
STBY = 19
BIN1 = 20
BIN2 = 21
PWMB = 12



GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(STBY, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

#youdan
youdan_pin = 18



def makecontent():
    now = datetime.now()
    formatted_date = now.strftime("%Y-%m-%d %H:%M:%S")
    return formatted_date

def housyutsu():
    with Picamera2() as camera:
        camera.resolution = (64, 48)
        camera.start()
        im=camera.capture_array()
        cv2.imshow("camera", im)
        
        img = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        img = np.array(img).flatten()
        mean = img.mean()

        with open("log.csv", "a", encoding='utf-8') as file:
            file.write(makecontent() + ',' + 'data' + ',' + 'shodo' + ',' + str(mean) + '\n')
        
        print(mean)
        
        if mean > 100:
            print("housyutsu")
            return 1
        else:
            print("mada")
            return 0

def housyutsu2():
    while True:
        result1 = housyutsu()
        result2 = housyutsu()
        result3 = housyutsu()

        if result1 == 1 and result2 == 1 and result3 == 1:
            hou = 1
            break
        else:
            hou = 0
            print("fail!")
    
    return hou

def youdann():
    finish=0
    GPIO.setmode(GPIO.BCM)             
    GPIO.setup(youdan_pin, GPIO.OUT)      
    GPIO.output(youdan_pin, GPIO.HIGH)     
    time.sleep(2)                       
    GPIO.output(youdan_pin, GPIO.LOW)     
    GPIO.cleanup()
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'output'+','+'GPIO11_high'+'\n')
    finish=1
    return(finish)

def int_to_binary(n: int, bits: int = 8) -> str:
    return ''.join([str(n >> i & 1 ) for i in reversed(range(0, bits))])


def bytes_to_binary(data: Union[bytearray,bytes]) -> List[str]:
    return [int_to_binary(byte) for byte in data]


def write_register(pi, spi_handler, register_addr: int, data: int):
   
    write_data = (register_addr & 0b01111111) << 8 | data
    write_data = write_data.to_bytes(2, "big")
    cnt, read_data = pi.spi_xfer(spi_handler, write_data)
    #print(f"cnt={cnt}, read_data={bytes_to_binary(read_data)}")


def read_register(pi, spi_handler, register_addr: int, num_bytes: int) -> bytes:
    write_data = (register_addr | 0b10000000) << (8 * num_bytes)
    write_data = write_data.to_bytes(num_bytes + 1, "big")
    cnt, read_data = pi.spi_xfer(spi_handler, write_data)
    if cnt != (num_bytes + 1):
        raise Exception(f"ReadError: cnt={cnt} (expected={num_bytes+1})")
    return read_data[1:]


def read_calibration_data(pi, spi_handler):
    cal_1 = read_register(pi, spi_handler, 0x88, 24)
    cal_2 = read_register(pi, spi_handler, 0xA1, 1)
    cal_3 = read_register(pi, spi_handler, 0xE1, 7)

    cal_data = OrderedDict([
        # --- --- --- 0x88 ~ 0x9F --- --- ---
        ("dig_T1", int.from_bytes(cal_1[0:2]  , byteorder="little", signed=False)),
        ("dig_T2", int.from_bytes(cal_1[2:4]  , byteorder="little", signed=True)),
        ("dig_T3", int.from_bytes(cal_1[4:6]  , byteorder="little", signed=True)),
        ("dig_P1", int.from_bytes(cal_1[6:8]  , byteorder="little", signed=False)),
        ("dig_P2", int.from_bytes(cal_1[8:10] , byteorder="little", signed=True)),
        ("dig_P3", int.from_bytes(cal_1[10:12], byteorder="little", signed=True)),
        ("dig_P4", int.from_bytes(cal_1[12:14], byteorder="little", signed=True)),
        ("dig_P5", int.from_bytes(cal_1[14:16], byteorder="little", signed=True)),
        ("dig_P6", int.from_bytes(cal_1[16:18], byteorder="little", signed=True)),
        ("dig_P7", int.from_bytes(cal_1[18:20], byteorder="little", signed=True)),
        ("dig_P8", int.from_bytes(cal_1[20:22], byteorder="little", signed=True)),
        ("dig_P9", int.from_bytes(cal_1[22:24], byteorder="little", signed=True)),

        # --- --- --- 0xA1 --- --- ---
        ("dig_H1", int.from_bytes(cal_2, byteorder="little", signed=False)),

        # --- --- --- 0xE1 ~ 0xE7 --- --- ---
        ("dig_H2", int.from_bytes(cal_3[0:2], byteorder="little", signed=True)),
        ("dig_H3", int.from_bytes(cal_3[2:3], byteorder="little", signed=False)),
        ("dig_H4", cal_3[3] << 4 | (0b00001111 & cal_3[4])),  
        ("dig_H5", cal_3[5] << 4 | (0b00001111 & (cal_3[4] >> 4))),  
        ("dig_H6", int.from_bytes(cal_3[7:8], byteorder="little", signed=True)),
    ])
    return cal_data


def read_temp(pi, spi_handler, cal_data: OrderedDict) -> Tuple[int, float]:
    temp_register = 0xFA
    read_bytes = read_register(pi, spi_handler, temp_register, 3)
    temp_raw = int.from_bytes(read_bytes, byteorder="big") >> 4
    
    var1 = (((temp_raw >> 3) - (cal_data["dig_T1"] << 1)) * cal_data["dig_T2"]) >> 11
    var2 = (((((temp_raw >> 4) - cal_data["dig_T1"]) * ((temp_raw >> 4) - cal_data["dig_T1"])) >> 12) * (cal_data["dig_T3"])) >> 14
    t_fine = var1 + var2
    temp = ((t_fine * 5 + 128) >> 8) / 100  # DegC
    return (t_fine, temp)


def read_pressure(pi, spi_handler, cal_data: OrderedDict, t_fine: int) -> float:
  
    read_bytes = read_register(pi, spi_handler, 0xF7, 3)
    pressure_raw = int.from_bytes(read_bytes, byteorder="big") >> 4
    
    var1 = t_fine - 128000
    var2 = var1 * var1 * cal_data["dig_P6"]
    var2 = var2 + ((var1 * cal_data["dig_P5"]) << 17)
    var2 = var2 + ((cal_data["dig_P4"]) << 35)
    var1 = ((var1 * var1 * cal_data["dig_P3"]) >> 8) + ((var1 * cal_data["dig_P2"]) << 12)
    var1 = ((1 << 47) + var1) * (cal_data["dig_P1"]) >> 33
    if (var1 == 0):
        return 0  
    p = 1048576 - pressure_raw
    p = (((p << 31) - var2) * 3125) // var1
    var1 = ((cal_data["dig_P9"]) * (p >> 13) * (p >> 13)) >> 25
    var2 = ((cal_data["dig_P8"]) * p) >> 19
    p = ((p + var1 + var2) >> 8) + ((cal_data["dig_P7"]) << 4)
    return p / 256 / 100  # hPa


def read_humidity(pi, spi_handler, cal_data: OrderedDict, t_fine: int) -> float:
    read_bytes = read_register(pi, spi_handler, 0xFD, 2)
    humidity_raw = int.from_bytes(read_bytes, byteorder="big")
    
    v_x1_u32r = t_fine - 76800
    v_x1_u32r = (
        (
            (((humidity_raw << 14) - ((cal_data["dig_H4"]) << 20) - ((cal_data["dig_H5"]) * v_x1_u32r)) + (16384)) >> 15
        ) * (
            ((((((v_x1_u32r * (cal_data["dig_H6"])) >> 10) * (((v_x1_u32r * (cal_data["dig_H3"])) >> 11) + 32768)) >> 10) + 2097152) * (cal_data["dig_H2"]) + 8192) >> 14
        )
    )
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (cal_data["dig_H1"])) >> 4))
    v_x1_u32r = 0 if (v_x1_u32r < 0) else v_x1_u32r
    v_x1_u32r = 419430400 if (v_x1_u32r > 419430400) else v_x1_u32r
    return (v_x1_u32r >> 12) / 1024  


#chakuchi_hantei_bme
def chakuchi_hantei_bme(pi, spi_handler):
    
    config_reg = 0x5F
    t_sb = 0b000    
    filter = 0b101  
    spi3w_en = 0b0  
    reg_data    = (t_sb << 5) | (filter << 2) | spi3w_en
    write_register(pi, spi_handler, config_reg, reg_data)

    max_difference =0.1

    prev_temp = None
    prev_press = None
    prev_hum = None
    consecutive_stable_count = 0
    max_consecutive_stable_count = 10
    
    ctrl_meas_reg = 0xF4
    osrs_t = 0b010  
    osrs_p = 0b101  
    mode = 0b11     
    reg_data = (osrs_t << 5) | (osrs_p << 2) | mode
    write_register(pi, spi_handler, ctrl_meas_reg, reg_data)

    
    ctrl_hum_reg = 0xF2
    osrs_h = 0b001  
    reg_data  = osrs_h
    write_register(pi, spi_handler, ctrl_hum_reg, reg_data)
    
    cal_data = read_calibration_data(pi, spi_handler)

    while True:
        
        t_fine, temp = read_temp(pi, spi_handler, cal_data)
        press = read_pressure(pi, spi_handler, cal_data, t_fine)
        hum = read_humidity(pi, spi_handler, cal_data, t_fine)
        with open("log.csv", "a", encoding='utf-8', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), temp, press, hum, chakuchi_a])
        

        if(
            prev_temp is not None and
            prev_press is not None and
            prev_hum is not None and
            abs(temp - prev_temp) <= max_difference and
            abs(press - prev_press) <= max_difference and
            abs(hum - prev_hum) <= max_difference
            
         ):
            consecutive_stable_count +=1
        else:
            consecutive_stable_count = 0

        if consecutive_stable_count >= max_consecutive_stable_count:
            chakuchi_a=1
            print(f"chakuchi!")
            break
        else:
            chakuchi_a=0
            print("mada")
    return chakuchi_a
        



#mota
def motasetup():
   
    pi.wiringPiSetupGpio()
    pi.pinMode( AIN1, pi.OUTPUT )
    pi.pinMode( AIN2, pi.OUTPUT )
    pi.pinMode( PWMA, pi.OUTPUT )
    pi.pinMode( STBY, pi.OUTPUT )
    pi.pinMode( BIN1, pi.OUTPUT )
    pi.pinMode( BIN2, pi.OUTPUT )
    pi.pinMode( PWMB, pi.OUTPUT )
    
    pi.softPwmCreate( PWMA, 0, 100 )
    pi.softPwmCreate( PWMB, 0, 100 )

    
def forward(n):
   
    
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    
    pi.digitalWrite( STBY, 1 )
    print("moveforward")
    
    pi.softPwmWrite( PWMA, 100 )
    pi.softPwmWrite( PWMB, 100 )
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'output'+','+"forward"+str(n)+"second"+'\n')


def stop():
    print("Stop!!")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.softPwmWrite( PWMB, 0 )

def back(n):
    
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 )
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
        
    pi.digitalWrite( STBY, 1 )
        
    pi.softPwmWrite( PWMA, 100 )
    pi.softPwmWrite( PWMB, 100)
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'output'+','+"back"+str(n)+"second"+'\n')

def stack():
    while True:
        acc = acc_value()
        if acc[2] < 0:
            print("stack! ","acc:",acc[2])
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'data'+','+"acceleration_z"+','+str(acc[2])+'\n')
                file.write(makecontent()+','+'data'+','+"invert"+'\n')
            forward(2)
            stop()
        else:
            break



def backspin_R(n):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 ) 
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    
    pi.digitalWrite( STBY, 1 )
    
    print("backspin_R")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100) 
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'output'+','+"backspin_R"+str(n)+"second"+'\n')

 
def backspin_L(n):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 ) 
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    
    pi.digitalWrite( STBY, 1 )
    
    print("backspin_L")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100)
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'output'+','+"backspin_L"+str(n)+"second"+'\n')

def offset_mota():
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 ) 
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    
    pi.digitalWrite( STBY, 1 )
    
    print("backspin_R")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100) 
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'output'+','+"backspin_R 10s"+'\n')
        



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

    #chakuchi_bmx
def chakuchi_hantei_bmx():
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
            chakuchi_b=1
            break
        else:
            chakuchi_b=0
    return(chakuchi_b)



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





def parse_nmea(sentence):
    try:
        msg = pynmea2.parse(sentence.decode('utf-8'))
        if isinstance(msg, pynmea2.GGA):
            print("Latitude: ", msg.latitude)
            print("Longitude: ", msg.longitude)
            print("Altitude: ", msg.altitude, " meters")
            return msg.longitude, msg.latitude
    except pynmea2.ParseError as e:
        print("NMEA Parse Error: ", e)
    return None, None

def GPS():
    uart = serial.Serial('/dev/serial0', 9600, timeout=3)
    while True:
        sentence = uart.readline()
        longitude, latitude = parse_nmea(sentence)
        if longitude is not None and latitude is not None:
            return longitude, latitude
        time.sleep(1)

def calculate_bearing(lat1, lon1, lat2, lon2):
    delta_lon = lon2 - lon1
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(delta_lon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(delta_lon))
    azimuth = math.atan2(x, y)
    azimuth = math.degrees(azimuth)
    azimuth = (azimuth + 360) % 360  
    return azimuth  

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000  
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) * math.sin(dlat / 2) + \
        math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
        math.sin(dlon / 2) * math.sin(dlon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def makecontent():
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    content = f"[{current_time}]"
    return content

def get_azimuth_distance():
    obj_latitude = 35.72471
    obj_longitude = 140.03850

    p1_longitude, p1_latitude = GPS()
    if p1_longitude is not None and p1_latitude is not None:
        with open("log.csv", "a", encoding='utf-8') as file:
            file.write(makecontent() + ',' + 'data' + ',' + 'Latitude' + ',' + str(p1_latitude) + '\n')
            file.write(makecontent() + ',' + 'data' + ',' + 'Longitude' + ',' + str(p1_longitude) + '\n')

        azimuth = calculate_bearing(p1_latitude, p1_longitude, obj_latitude, obj_longitude)
        distance = calculate_distance(p1_latitude, p1_longitude, obj_latitude, obj_longitude)
        
        if azimuth < 0:
            azimuth += 360
        return azimuth, distance  
if __name__ == "__main__":
    azimuth, distance = get_azimuth_distance()
    print(f"azimuth: {azimuth} degrees")
    print(f"Distance: {distance} meters")

def getoffset():
    n11=0
    n12=0
    n13=0
    n22=0
    n23=0
    n1=0
    n2=0
    n3=0
    print ("offset start")
    offset_mota()
    for i in range (200):
        data=mag_value()
        
        x=data[0]
        y=data[1]
        n11=n11+x**2
        n12=n12+x*y
        n13=n13+x
        n22=n22+y**2
        n23=n23+y
        n1=-(x**3+x*y**2)+n1
        n2=-(x**2*y+y**3)+n2
        n3=-(x**2+y**2)+n3
        time.sleep(0.05)
    stop()
    stack()
    a=np.array([[n11,n12,n13],[n12,n22,n23],[n13,n23,200]])
   
    a_inv=np.linalg.inv(a)
    b=np.array([[n1],[n2],[n3]])
    result=np.dot(a_inv,b)
    result_a=result[0][0]
    result_b=result[1][0]
    offset_x=-1*result_a/2
    offset_y=-1*result_b/2
    offset = [offset_x,offset_y]
    print("offset end")
    print(offset_x,offset_y)
    return(offset)



def get_theta(azimuth,offset):
    theta = 360
    
    mag = mag_value()
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'data'+','+'jiki_x'+','+str(mag[0])+'\n')
        file.write(makecontent()+','+'data'+','+'jiki_y'+','+str(mag[1])+'\n')    
    
    if(mag[0]-offset[0] != 0):
        if(mag[0]-offset[0] < 0 and mag[1] - offset[1] > 0):
            rad1 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg1 = math.degrees(rad1)
            azimuth_sat = deg1 + 180
        elif(mag[0] - offset[0] < 0 and mag[1] - offset[1] < 0):
            rad2 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg2 = math.degrees(rad2)
            azimuth_sat = deg2 +180
        elif(mag[0]-offset[0] > 0 and mag[1] - offset[1] < 0):
            rad3 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg3 = math.degrees(rad3)
            azimuth_sat = deg3 + 360
        else:
            rad4 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg4 = math.degrees(rad4)
            azimuth_sat = deg4

        
        if(abs(azimuth-azimuth_sat) > 180):
            theta = 360 - abs(azimuth-azimuth_sat)
        
        else:
            theta = abs(azimuth-azimuth_sat)

        print('houigaku', azimuth_sat)

      
        muki = 0
        theta2 = azimuth_sat - azimuth
        if(theta2 > 0 and abs(theta2) < 180):
            muki = 0
            print('goal is left')

        elif(theta2 > 0 and abs(theta2) > 180):
            muki = 1
            print('goal is right')

        elif(theta2 < 0 and abs(theta2) < 180):
            muki = 1
            print('goal is right')

        elif(theta2 < 0 and abs(theta2) > 180):
            muki = 0
            print('goal is left')

       
    if(mag[0]-offset[0] == 0):
        print("bunbo ga 0 360")
        
    return theta,muki

def rotate_to_target_direction(theta):
    if(muki == 0):
        backspin_L(theta / 180)
    elif(muki == 1):
        backspin_R(theta / 180)
    print('spin')
                

def GPS_yuudou():
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'sequence'+','+'offset start'+'\n')
    offset = getoffset()
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'data'+','+'offset_x'+str(offset[0])+'\n')
        file.write(makecontent()+','+'data'+','+'offset_y'+str(offset[1])+'\n')
        file.write(makecontent()+','+'sequence'+','+'offset end'+'\n')
    yuudou=0
    while True:
        acc = acc_value()
        mag = mag_value()
        gyro = gyro_value()
        azimuth, distance = get_azimuth_distance()
        theta, muki = get_theta(azimuth, offset)

        print('houi',azimuth)
        print('kyori',distance,'m')
        print('houigakusa',theta)
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'data'+','+"houigakusa"+","+str(theta)+'\n')
            file.write(makecontent()+','+'data'+','+"kyori"+","+str(distance)+'\n')

        rotate_to_target_direction(theta)
        kaihi()

        if(distance >= 2):
            if(theta <= 30):
                print('forward')
                forward(5)
                stop()
                stack()

            else:
                if(muki == 0):
                    backspin_L(theta / 180)
                if(muki == 1):
                    backspin_R(theta / 180)
                print('spin')
               
                stop()
                stack()

        else:
            yuudou=1
            break
    return(yuudou)




def is_purple_object_on_right_half(im):
    img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_purple = np.array([125, 50, 50])  
    upper_purple = np.array([150, 255, 255])  
    
    frame_mask_purple = cv2.inRange(hsv, lower_purple, upper_purple)

    purple_contours, _ = cv2.findContours(frame_mask_purple, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(purple_contours) > 0:
        max_purple_contour = max(purple_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_purple_contour)
        
        img_width = im.shape[1]
        object_center_x = x + w // 2

        right_half_boundary = img_width // 2
        is_on_right_half = object_center_x > right_half_boundary

        return is_on_right_half

    return False

#para_kaihi
def kaihi():
    with Picamera2() as camera:
        try:
            camera.resolution = (640, 480)
            camera.start()
            kaihi = 0
            is_object_on_right = False  

            while True:
                im = camera.capture_array()

                if is_purple_object_on_right_half(im):
                    print("Purple object is on the right half of the image.")
                    back(5)
                    backspin_R(0.2)
                    stop()
                    stack()
                    kaihi = 1
                    is_object_on_right = True
                else:
                    print("Purple object is not on the right half of the image.")
                    if is_object_on_right:
                        break  
                    forward(3)
                    stop()
                    stack()
                    kaihi = 1
        finally:
            cv2.destroyAllWindows()

    return kaihi



def led_goal_function(pin_no):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin_no, GPIO.OUT)

    try:
        while True:
            GPIO.output(pin_no, True)
            time.sleep(0.5)
            GPIO.output(pin_no, False)
            time.sleep(0.5)

            print("goal")

    except KeyboardInterrupt:
        GPIO.cleanup()

#led
pin_no = 24
led_goal_function(pin_no)


if __name__=="__main__":
    with open("log.csv","w",encoding='utf-8')as file:
        file.write(makecontent()+','+'sequence'+','+str("log_start")+'\n')
    bmx_setup()
    motasetup()
    fall=0
    chakuchia=0
    chakuchib=0
    tegusu=0
    para=0
    yuudou=0
    ironinnshiki=0
    
    #houshutu
    while True:
        fall=housyutsu2()
        if fall==1:
            print("housyutsu!")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'sequence'+','+"houshutsu!"+'\n')
            break
        else:
            print("tutunonaka")
    
    #chakuchi
    while True:
        chakuchia=chakuchi_hantei_bme()
        chakuchib=chakuchi_hantei_bmx()

        if chakuchia==1 and chakuchib==1:
            print("chakuchi!")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'sequence'+','+"chakuchi!"+'\n')
            break
        else:
            print("kuucyuu")
    
    #youdan  
    tegusu=youdann()
    if tegusu==1:
        print("youdan!")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"youdan!"+'\n')
    else:
        print("youdann_error")
    
    #GPS_yudou + para_kaihi
    yuudou=GPS_yuudou()
    if para==1:
        print("kaihi_kannryou!")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"kaihi_kannryou!"+'\n')
    else:
        print("kaihi_error")
    if yuudou==1:
        print("near by 5m")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"GPS_ok!"+'\n')
    else:
        print("error_yuudou")

    
    
    if para==1:
        print("kaihi_kannryou!")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"kaihi_kannryou!"+'\n')
    else:
        print("kaihi_error")
    
    #GPS_yudou
    yuudou=GPS_yuudou()
    if yuudou==1:
        print("near by 5m")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"GPS_ok!"+'\n')
    else:
        print("error_yuudou")

    #camera
      #now loading...

    #goooooooal!!!!!!
    led_goal_function()


