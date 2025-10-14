from epmc import EPMC
import time


motorControl = EPMC('/dev/ttyUSB0')

#wait for the EPMC to fully setup
for i in range(4):
  time.sleep(1.0)
  print(f'configuring controller: {i} sec')

motorControl.clearDataBuffer()
motorControl.writeSpeed(0.0, 0.0)
print('configuration complete')

motorControl.setCmdTimeout(5000)
timeout = motorControl.getCmdTimeout()
print("command timeout in ms: ", timeout)

lowTargetVel = -3.142 # in rad/sec
highTargetVel = 3.142 # in rad/sec

prevTime = None
sampleTime = 0.015

ctrlPrevTime = None
ctrlSampleTime = 5.0
sendHigh = True


motorControl.writeSpeed(lowTargetVel, lowTargetVel) # targetA, targetB
sendHigh = True

prevTime = time.time()
ctrlPrevTime = time.time()
while True:
    try:
      start_time = time.time()
      motorControl.writeSpeed(highTargetVel, highTargetVel) # targetA, targetB
      success, motor_data = motorControl.readMotorData()
      if success:
         pos0 = motor_data[0]
         pos1 = motor_data[1]
         v0 = motor_data[2]
         v2 = motor_data[3]
      dt = start_time - prevTime
      prevTime = start_time
      print(dt)
    except:
       pass


