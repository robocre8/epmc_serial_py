from epmc_serial import EPMCSerialClient, SupportedNumOfMotors
import time

controller = EPMCSerialClient(SupportedNumOfMotors.TWO)

# variable for communication
pos0=0.0; pos1=0.0
vel0=0.0; vel1=0.0

# [4 rev/sec, 2 rev/sec, 1 rev/sec, 0.5 rev/sec]
targetVel = [1.571, 3.142, 6.284, 12.568] 
vel = targetVel[1]
v = 0.0

readTime = None
readTimeInterval = 0.02 # 50Hz

cmdTime = None
cmdTimeInterval = 5.0

# 50Hz comm setup
serial_port = '/dev/ttyACM0'
serial_baudrate = 115200
serial_timeout = 0.018 #value < 0.02 (for 50Hz comm)

controller.connect(serial_port, serial_baudrate, serial_timeout)

success = controller.clearDataBuffer()
controller.writeSpeed(0.0, 0.0)
print('configuration complete')

timeout_ms = 10000
controller.setCmdTimeout(timeout_ms)
success, val0 = controller.getCmdTimeout()
if success: # only update if read was successfull
  timeout_ms = val0
  print("command timeout in ms: ", timeout_ms)
else:
  print("ERROR: could not read motor command timeout")

sendHigh = True

readTime = time.time()
cmdTime = time.time()

while True:
  if time.time() - cmdTime > cmdTimeInterval:
    if sendHigh:
      # print("command high")
      v = vel
      controller.writeSpeed(v, v)
      vel = vel*-1
      sendHigh = False
    else:
      # print("command low")
      v = 0.0
      controller.writeSpeed(v, v)
      sendHigh = True
    
    
    cmdTime = time.time()



  if time.time() - readTime > readTimeInterval:
    try:
      # controller.writeSpeed(v, v)
      success, val = controller.readMotorData()
      if success: # only update if read was successfull
        pos0 = val[0]; pos1 = val[1]
        vel0 = val[2]; vel1 = val[3]

      print(f"motor0_readings: [{pos0}, {vel0}]")
      print(f"motor1_readings: [{pos1}, {vel1}]")
      print("")
    except:
      pass
    
    readTime = time.time()
