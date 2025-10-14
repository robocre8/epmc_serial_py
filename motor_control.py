from epmc import EPMC
import time


motorControl = EPMC('/dev/ttyUSB0')

#wait for the EPMC to fully setup
for i in range(4):
  time.sleep(1.0)
  print(f'configuring controller: {i+1} sec')

motorControl.clearDataBuffer()
motorControl.writeSpeed(0.0, 0.0)
print('configuration complete')

motorControl.setCmdTimeout(5000)
timeout = motorControl.getCmdTimeout()
print("command timeout in ms: ", timeout)

angPosA = 0.0
angPosB = 0.0
angVelA = 0.0
angVelB = 0.0

lowTargetVel = -10.00 # in rad/sec
highTargetVel = 10.00 # in rad/sec

prevTime = None
sampleTime = 0.015

ctrlPrevTime = None
ctrlSampleTime = 4.0
sendHigh = True


motorControl.writeSpeed(lowTargetVel, lowTargetVel) # targetA, targetB
sendHigh = True

prevTime = time.time()
ctrlPrevTime = time.time()
while True:
  if time.time() - ctrlPrevTime > ctrlSampleTime:
    if sendHigh:
      motorControl.writeSpeed(highTargetVel, highTargetVel) # targetA, targetB
      sendHigh = False
    else:
      motorControl.writeSpeed(lowTargetVel, lowTargetVel) # targetA, targetB
      sendHigh = True
    
    ctrlPrevTime = time.time()



  if time.time() - prevTime > sampleTime:
    try:
      success, motor_data = motorControl.readMotorData()
      if success:
        angPosA = round(motor_data[0], 4)
        angPosB = round(motor_data[1], 4)
        angVelA = round(motor_data[2], 6)
        angVelB = round(motor_data[3], 6)
      print(f"motorA_readings: [{angPosA}, {angVelA}]")
      print(f"motorB_readings: [{angPosB}, {angVelB}]")
      print("")
    except:
      pass
    
    prevTime = time.time()
