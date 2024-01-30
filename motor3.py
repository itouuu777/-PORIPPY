import time
import RPi.GPIO as GPIO

AIN1 = 26
AIN2 = 21
PWMA = 19
STBY = 20
BIN1 = 13
BIN2 = 6
PWMB = 5
def motasetup():
    # GPIO�s���̐ݒ�
    pi.wiringPiSetupGpio()
    pi.pinMode( AIN1, pi.OUTPUT )
    pi.pinMode( AIN2, pi.OUTPUT )
    pi.pinMode( PWMA, pi.OUTPUT )
    pi.pinMode( STBY, pi.OUTPUT )
    pi.pinMode( BIN1, pi.OUTPUT )
    pi.pinMode( BIN2, pi.OUTPUT )
    pi.pinMode( PWMB, pi.OUTPUT )
    # PWM�[�q�ɐڑ�����GPIO��PWM�o�͂ł���悤�ɂ���
    pi.softPwmCreate( PWMA, 0, 100 )
    pi.softPwmCreate( PWMB, 0, 100 )

    #���[�^�[�𓮂������߂̊֐�
#�^���l�ƃ��[�^�[�̏o�͂����߂Ă���
#�����̓��[�^�[���񂷎���
#�O�i
def forward(n):
    # �X�^���o�C��Ԃɂ���
    #�t��]�^���l
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
    #���[�^�[�̏o��
    pi.softPwmWrite( PWMA, 100 )
    pi.softPwmWrite( PWMB, 100 )
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'�o��'+','+"�O�i"+str(n)+"�b"+'\n')

#�X�g�b�v
def stop():
    print("Stop!!")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.softPwmWrite( PWMB, 0 )

    def back(n):
    #���������ᐧ�䂵��PWM�̉���n��
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
            file.write(makecontent()+','+'�o��'+','+"���"+str(n)+"�b"+'\n')

    def stack():
    while True:
        acc = acc_value()
        if acc[2] < 0:
            print("stack! ","acc:",acc[2])
            with open("log.csv","a",encoding='utf-8')as file:
                file.write(makecontent()+','+'�f�[�^'+','+"�����x_z"+','+str(acc[2])+'\n')
                file.write(makecontent()+','+'�f�[�^'+','+"�@�̂����]"+'\n')
            forward(2)
            stop()
        else:
            break


#�E��]
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
    #�^���l�ς��邽�߂�setup�֐����ĂԂ̂�Y��Ȃ��悤��
    #theat��10�����炢�܂ŉ�]
    print("backspin_R")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100) 
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'�o��'+','+"�E��]"+str(n)+"�b"+'\n')

#����]  
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
    #�^���l�ς��邽�߂�setup�֐����ĂԂ̂�Y��Ȃ��悤��
    #theat��10�����炢�܂ŉ�]
    print("backspin_L")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100)
    time.sleep(n)
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'�o��'+','+"����]"+str(n)+"�b"+'\n')

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
    #�^���l�ς��邽�߂�setup�֐����ĂԂ̂�Y��Ȃ��悤��
    #theat��10�����炢�܂ŉ�]
    print("backspin_R")
    pi.softPwmWrite(PWMA,100)
    pi.softPwmWrite(PWMB,100) 
    with open("log.csv","a",encoding='utf-8')as file:
            file.write(makecontent()+','+'�o��'+','+"�E��]10�b"+'\n')