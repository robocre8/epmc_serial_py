from epmc import EPMC
import time


motorControl = EPMC('/dev/ttyUSB0')

#wait for the EPMC to fully setup
for i in range(6):
  time.sleep(1.0)
  print(f'configuring controller: {i} sec')
motorControl.sendTargetVel(0.0, 0.0)
print('configuration complete')

angPosA=0.0
angPosB=0.0
angVelA=0.0
angVelB=0.0

lowTargetVel = 3.142 # in rad/sec
highTargetVel = -3.142 # in rad/sec

prevTime = None
sampleTime = 0.02

ctrlPrevTime = None
ctrlSampleTime = 10.0
sendHigh = True


motorControl.sendTargetVel(lowTargetVel, lowTargetVel) # targetA, targetB
sendHigh = True

prevTime = time.time()
ctrlPrevTime = time.time()
while True:
  if time.time() - ctrlPrevTime > ctrlSampleTime:
    if sendHigh:
      motorControl.sendTargetVel(highTargetVel, highTargetVel) # targetA, targetB
      sendHigh = False
    else:
      motorControl.sendTargetVel(lowTargetVel, lowTargetVel) # targetA, targetB
      sendHigh = True
    
    ctrlPrevTime = time.time()



  if time.time() - prevTime > sampleTime:
    try:
      angPosA, angPosB = motorControl.getMotorsPos() # returns angPosA, angPosB
      angVelA, angVelB = motorControl.getMotorsVel() # returns angVelA, angVelB
      print(f"motorA_readings: [{angPosA}, {angVelA}]")
      print(f"motorB_readings: [{angPosB}, {angVelB}]")
      print("")
    except:
      pass
    
    prevTime = time.time()
  # time.sleep(0.01)

