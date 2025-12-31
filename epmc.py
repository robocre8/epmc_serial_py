import serial
import time

# Serial Protocol Command IDs -------------
WRITE_SPEED = 10
READ_SPEED = 11
READ_TSPEED = 12
READ_POS = 13
WRITE_PWM = 14
SET_KP = 15
GET_KP = 16
SET_KI = 17
GET_KI = 18
SET_KD = 19
GET_KD = 20
SET_PPR = 21
GET_PPR = 22
SET_CF = 23
GET_CF = 24
SET_RDIR = 25
GET_RDIR = 26
SET_PID_MODE = 27
GET_PID_MODE = 28
SET_CMD_TIMEOUT = 29
GET_CMD_TIMEOUT = 30
SET_I2C_ADDR = 31
GET_I2C_ADDR = 32
SET_MAX_SPEED = 33
GET_MAX_SPEED = 34
RESET = 35
CLEAR = 36
#---------------------------------------------



class EPMC:
    def __init__(self):
        pass

    def connect(self, port, baud=115200, timeOut=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeOut)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
    
    def disconnect(self):
        if self.ser.is_open:
            self.ser.close()

    #------------------------------------------------------------------------
    def send(self, cmd, arg1=0.0, arg2=0.0):
        send_str = str(float(cmd))+" "+str(round(float(arg1),4))+" "+str(round(float(arg2),4))+"\r"
        self.ser.write(send_str.encode())

    def recv(self, cmd, arg1=0):
        try:
            self.send(cmd, arg1)
            data = self.ser.readline().decode().strip().split(' ')
            return True, round(float(data[0]),4), round(float(data[1]),4)
        except:
            # self.ser.reset_input_buffer()
            # self.ser.reset_output_buffer()
            return False, 0.0, 0.0
    
    #---------------------------------------------------------------------
    def writeSpeed(self, v0, v1):
        self.send(WRITE_SPEED, v0, v1)

    def readSpeed(self):
        success, vel0, vel1 = self.recv(READ_SPEED)
        return success, vel0, vel1
    
    def writePWM(self, pwm0, pwm1):
        self.send(WRITE_PWM, pwm0, pwm1)
    
    def readPos(self):
        success, pos0, pos1 = self.recv(READ_POS)
        return success, pos0, pos1
    
    def setCmdTimeout(self, timeout):
        self.send(SET_CMD_TIMEOUT, 0.0, float(timeout))
        
    def getCmdTimeout(self):
        success, timeout, _ = self.recv(GET_CMD_TIMEOUT)
        return success, int(timeout)
    
    def setPidMode(self, mode):
        self.send(SET_PID_MODE, 0.0, mode)
    
    def getPidMode(self):
        success, mode, _ = self.recv(GET_CMD_TIMEOUT)
        return success, int(mode)
    
    def clearDataBuffer(self):
        success, _, _ = self.recv(CLEAR)
        return success
    
    def getMaxSpeed(self, motor_no):
        success, maxVel, _ = self.recv(GET_MAX_SPEED, motor_no)
        return success, round(maxVel, 3)
    #---------------------------------------------------------------------