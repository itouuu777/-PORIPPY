import time
import RPi.GPIO as GPIO
import wiringpi as pi

AIN1 = 26
AIN2 = 21
PWMA = 19
STBY = 20
BIN1 = 13
BIN2 = 6
PWMB = 5
def motasetup():
    # GPIOピンの設定
    pi.wiringPiSetupGpio()
    pi.pinMode( AIN1, pi.OUTPUT )
    pi.pinMode( AIN2, pi.OUTPUT )
    pi.pinMode( PWMA, pi.OUTPUT )
    pi.pinMode( STBY, pi.OUTPUT )
    pi.pinMode( BIN1, pi.OUTPUT )
    pi.pinMode( BIN2, pi.OUTPUT )
    pi.pinMode( PWMB, pi.OUTPUT )
    # PWM端子に接続したGPIOをPWM出力できるようにする
    pi.softPwmCreate( PWMA, 0, 100 )
    pi.softPwmCreate( PWMB, 0, 100 )

    #モーターを動かすための関数
#真理値とモーターの出力を決めている
#引数はモーターを回す時間
#前進
def forward(n):
    # スタンバイ状態にする
    #逆回転真理値
    #print("StandBy")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    print("moveforward")
    #モーターの出力
    pi.softPwmWrite( PWMA, 100 )
    pi.softPwmWrite( PWMB, 100 )
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"前進"+str(n)+"秒"+'\n')

#ストップ
def stop():
    print("Stop!!")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.softPwmWrite( PWMB, 0 )

    def back(n):
    #距離から比例制御してPWMの価を渡す
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 )
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    # print("movefoward")
    pi.softPwmWrite( PWMA, 100 )
    pi.softPwmWrite( PWMB, 100)
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"後退"+str(n)+"秒"+'\n')

    def stack():
    while True:
        acc = acc_value()
        if acc[2] < 0:
            print("stack! ","acc:",acc[2])
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'データ'+','+"加速度_z"+','+str(acc[2])+'\n')
                file.write(makecontent()+','+'データ'+','+"機体が反転"+'\n')
            forward(2)
            stop()
        else:
            break


#右回転
def backspin_R(n):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 ) 
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    print("backspin_R")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100) 
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"右回転"+str(n)+"秒"+'\n')

#左回転  
def backspin_L(n):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 ) 
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    print("backspin_L")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100)
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"左回転"+str(n)+"秒"+'\n')

    def offset_mota():
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 ) 
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    print("backspin_R")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100) 
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'出力'+','+"右回転10秒"+'\n')
