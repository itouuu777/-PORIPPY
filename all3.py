import serial
import pynmea2
import time
from geopy.distance import geodesic 
from datetime import datetime
import math
import numpy as np
from smbus import SMBus
import RPi.GPIO as GPIO
import wiringpi as pi
from picamera2 import Picamera2
from libcamera import controls
import cv2
from scipy.optimize import curve_fit
import csv


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

#led
pin_no = 17

def makecontent():
    now = datetime.now()
    formatted_date = now.strftime("%Y-%m-%d %H:%M:%S")
    return formatted_date

def housyutsu():
    with Picamera2() as camera:
        camera.resolution = (64, 48)
        camera.start()
        camera.capture_array()
        
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
    pi.digitalWrite( AIN1, 0 )
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    
    pi.digitalWrite( STBY, 1 )
    print("moveforward")
    
    pi.softPwmWrite( PWMA, 150 )
    pi.softPwmWrite( PWMB, 150 )
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'output'+','+"forward"+str(n)+"second"+'\n')


def forward_S(n):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 )
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    
    pi.digitalWrite( STBY, 1 )
    print("moveforward")
    
    pi.softPwmWrite( PWMA, speed )
    pi.softPwmWrite( PWMB, speed )
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'output'+','+"forward_S"+str(n)+"second"+'\n')

def stop():
    print("Stop!!")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.softPwmWrite( PWMB, 0 )

def back(n):
    
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
        
    pi.digitalWrite( STBY, 1 )
        
    pi.softPwmWrite( PWMA, 100)
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
    pi.digitalWrite( AIN1, 0 ) 
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    
    pi.digitalWrite( STBY, 1 )
    
    print("backspin_R")
    pi.softPwmWrite(PWMA,150)
    pi.softPwmWrite(PWMB,150) 
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
        file.write(makecontent()+','+'output'+','+"backspin_R"+str(n)+"second"+'\n')

 
def backspin_L(n):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 ) 
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    
    pi.digitalWrite( STBY, 1 )
    
    print("backspin_L")
    pi.softPwmWrite(PWMA,150)
    pi.softPwmWrite(PWMB,150)
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
    start_time = time.time()
    while True:
        b1 = gyro_value()
        time.sleep(0.5)
        b2 = gyro_value()
        time.sleep(0.5)
        b3 = gyro_value()
        
        with open("log.csv", "a", encoding='utf-8') as file:
            file.write(makecontent() + ',' + 'data' + ',' + 'angular velocity' + ',' + str(b1[0]) + '\n')
            file.write(makecontent() + ',' + 'data' + ',' + 'angular velocity' + ',' + str(b2[0]) + '\n')
            file.write(makecontent() + ',' + 'data' + ',' + 'angular velocity' + ',' + str(b3[0]) + '\n')
            
        if abs(b1[0]) < 1.0 and abs(b2[0]) < 1.0 and abs(b3[0]) < 1.0:
            chakuchi = 1
            break
        else:
            chakuchi = 0

        elapsed_time = time.time() - start_time
        if elapsed_time >= 20.0:
            print("Timed out. Setting chakuchi to 1.")
            chakuchi = 1
            break
    return(chakuchi)



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



def get_azimuth_distance():
    obj_latitude = 35.72471   #for example
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

def rotate_to_target_direction(theta,muki):
    if(muki == 0):
        backspin_L(theta / 180)
    elif(muki == 1):
        backspin_R(theta / 180)
    print('spin')
                

def GPS_yuudou():
    for speed in range(50, 100):
        forward(0.1,speed)
    forward_S(5, 100)
    stop()
    stack()
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
        if kaihi==1:
            print("succeed")
            
        else:
            backspin_L(1)
            stop()
            for speed in range(50,100):
                forward_S(0.1,speed)
            forward_S(5,100)
            stop()
            acc = acc_value()
            mag = mag_value()
            gyro = gyro_value()
            azimuth, distance = get_azimuth_distance()
            theta, muki = get_theta(azimuth, offset)
            print("kaihi&succeed")
        


        if(distance >= 2):
            if(theta <= 30):
                print('forward')
                for speed in range(50, 100):
                   forward(0.1,speed)
                forward_S(5, 100)
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
                    kaihi = 0
                    break
                else:
                    print("Purple object is not on the right half of the image.")
                    kaihi=1
                    break  
                    
                    
        finally:
            cv2.destroyAllWindows()

    return kaihi





def find_nearest_point(contour, center):
    distances = [np.linalg.norm(np.array(point[0]) - np.array(center)) for point in contour]
    nearest_point_index = np.argmin(distances)
    return tuple(contour[nearest_point_index][0])

def is_point_near_center(point, center, threshold=20):
    center_x, center_y = center
    x, y = point
    return x > 0 and abs(y) < threshold

def cam_chosei():
    with Picamera2() as camera:
        try:
            camera.resolution = (640, 480)
            camera.start()
            
            while True:
                im = camera.capture_array()
                
                img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 211, 153])
                upper_red = np.array([3, 255, 255])
                
                frame_mask_red = cv2.inRange(hsv, lower_red, upper_red)

                red_contours, _ = cv2.findContours(frame_mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if len(red_contours) > 0:
                    max_red_contour = max(red_contours, key=cv2.contourArea)
                    center = tuple(np.array(im.shape[1::-1]) // 2)
                    nearest_point = find_nearest_point(max_red_contour, center)
                    
                    center_x, center_y = center
                    nearest_point = (nearest_point[0] - center_x, nearest_point[1] - center_y)
                    
                    cv2.circle(img, (center_x + nearest_point[0], center_y + nearest_point[1]), 5, (0, 255, 0), -1)
                    cv2.putText(img, f"Nearest Point: {nearest_point}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    chuo=0
                    if is_point_near_center(nearest_point, center):
                        print("Point is near center!")
                        chuo=1
                        break
                    else:
                        backspin_L(0.1)
                        stop()
                        stack()
                        chuo=0
               
                if chuo==1:
                    break

        finally:
            cv2.destroyAllWindows()

        return chuo

def cam_zahyo():
    with Picamera2() as camera:
        try:
            camera.resolution = (640, 480)
            camera.start()
            
            while True:
                im = camera.capture_array()
                
                img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 211, 153])
                upper_red = np.array([3, 255, 255])
                
                frame_mask_red = cv2.inRange(hsv, lower_red, upper_red)

                red_contours, _ = cv2.findContours(frame_mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if len(red_contours) > 0:
                    max_red_contour = max(red_contours, key=cv2.contourArea)
                    center = tuple(np.array(im.shape[1::-1]) // 2)
                    nearest_point = find_nearest_point(max_red_contour, center)
                    
                    center_x, center_y = center
                    nearest_point = (nearest_point[0] - center_x, nearest_point[1] - center_y)
                    
                    cv2.circle(img, (center_x + nearest_point[0], center_y + nearest_point[1]), 5, (0, 255, 0), -1)
                    cv2.putText(img, f"Nearest Point: {nearest_point}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    zahyou=0
                    if nearest_point[0] > 0:
                        
                        cv2.destroyAllWindows()
                        return nearest_point

                
                print(f"Nearest Point: {nearest_point}")
                zahyou=1
                break

        finally:
            cv2.destroyAllWindows()

        return zahyou

def tenth_order_func(x, a, b, c, d, e, f, g, h, i, j, k):
    return a * x**10 + b * x**9 + c * x**8 + d * x**7 + e * x**6 + f * x**5 + g * x**4 + h * x**3 + i * x**2 + j * x + k


def dis(nearest_point):
    filename = 'distance_data.csv'
    camera_data = []
    real_data = []
    diss=0
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row:
                camera_data.append(float(row[0]))
                real_data.append(float(row[1]))
    popt, pcov = curve_fit(tenth_order_func, camera_data, real_data)

    x_value = nearest_point
    y_value = tenth_order_func(x_value, *popt)
    print(y_value)
    
    
    diss=1
    return diss

    
def last():
    with Picamera2() as camera:
        try:
            camera.resolution = (64, 48)
            camera.start()

            while True:
                im = camera.capture_array()
                cv2.imshow("camera", im)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break

        finally:
            cv2.destroyAllWindows()

            img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            lower_red = np.array([0, 110, 70])
            upper_red = np.array([3, 255, 255])
            frame_mask = cv2.inRange(hsv, lower_red, upper_red)
            
            complete = 0

            dst = cv2.bitwise_and(img, img, mask=frame_mask)
            cv2.imwrite('otameshi.jpg', dst)

            img = cv2.imread('otameshi.jpg', 0)
            if img is None:
                print("fall")
                return None 

            ret, img_th = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU)

            hole_area = img_th.size
            red_area = cv2.countNonZero(img_th)
            red_percentage =red_area / hole_area * 100

            print('aka sennyu: ' + str(red_percentage) + '%')

            with open("log.csv", "a", encoding='utf-8') as file:
                file.write(makecontent() + ',' + 'data' + ',' + 'para' + str(red_percentage) + '\n')

            if red_percentage > 30:
                 complete = 1
                 

    return complete



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
    chu=0
    zah=0
    di=0

    #houshutu
    while True:
        fall=housyutsu2()
        if fall==1:
            print("housyutsu!")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'sequence'+','+"houshutsu!"+'\n')
            break
        else:
            print("madadetenai")
    
    #chakuchi
    while True:
        chakuchi=chakuchi_hantei_bmx()

        if chakuchi==1:
            print("chakuchi!")
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'sequence'+','+"chakuchi!"+'\n')
            break
        else:
            print("in the sky")
    
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
        print("near by 3m")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"GPS_ok!"+'\n')
    else:
        print("error_yuudou")

    

    #camera
    chu=cam_chosei()
    if chu==1:
        print("object is centor")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"object is centor!"+'\n')
    else:
        print("chu_error")

    zah=cam_zahyo()
    if zah==1:
        print("zahyou shutoku")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"zahyou!"+'\n')
    else:
        print("where?")
    
    di=dis()
    if di==1:
        print("GOAL!")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"GOAL!"+'\n')
    else:
        print("I'll be back")

    
    finish = last()
    if finish == 1:
        print("Goal!")
        with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'sequence'+','+"object is centor!"+'\n')
    else:
        print("oshii!")
    