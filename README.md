
## Easy PID Motor Controller (EPMC) Python Library
Python serial interface for the Easy PID Motor Controller (EPMC).

> you can use it in your microcomputer robotics project (e.g Raspberry Pi, PC, etc.)

#

## Install
- you'll need to pip install the pyserial library
  ```shell
    pip3 install epmc-serial   //linux or mac
    pip install epmc-serial  //windows
  ```

#

## Uninstall
- you'll need to pip install the pyserial library
  ```shell
    pip3 uninstall epmc-serial   //linux or mac
    pip uninstall epmc-serial  //windows
  ```

#

## How to Use the Library
- Ensure you have the **`EPMC MODULE`** interfaced with your preferred motors

- Ensure you've already setup the encoder and PID parameters with the [epmc_setup_application](https://github.com/robocre8/epmc_setup_application).

- check the serial port the driver is connected to:
  ```shell
  ls /dev/ttyA*
  ```
  > you should see /dev/ttyACM0 or /dev/ttyACM1 and so on

- use the serial port in your code

- A simple way to get started is simply to try out the example code below


## Basic Library functions and usage

- connect to EPMC module
  > controller = EPMCSerialClient()
  >
  > controller.connect("port_name or port_path")

- clear speed, position, e.t.c data buffer on the EPMC module
  > controller.clearDataBuffer() # returns bool -> success

- send target angular velocity command
  > controller.writeSpeed(motor0_TargetVel, motor1_TargetVel)

- send PWM command
  > controller.writePWM(motor0_PWM, motor1_PWM)

- set motor command timeout
  > controller.setCmdTimeout(timeout_ms)

- get motor command timeout
  > controller.getCmdTimeout() # returns tuple -> (success, motor_command_timeout_ms): bool, float

- read motors angular position and angular velocity
  > controller.readMotorData() # returns tuple -> (success, angPos0, angPos1, angVel0, angVel1): bool, float, float, float, float

- read motors angular position
  > controller.readPos() # returns tuple -> (success, angPos0, angPos1): bool, float, float

- read motors angular velocity
  > controller.readVel() # returns tuple -> (success, angVel0, angVel1): bool, float, float

- read motor maximum commandable angular velocity
  > controller.getMaxVel(motor_no) # returns tuple -> (success, max_vel): bool, float, float
  > maxVel0 or maxVel1 based on the specified motor number

- while these function above help communicate with the already configure EPMC module, more examples of advanced funtions usage for parameter tuning can be found in the [epmc_setup_application](https://github.com/robocre8/epmc_setup_application) source code

#

## example code
```python
from epmc_serial import EPMCSerialClient
import time

controller = EPMCSerialClient()

# variable for communication
pos0=0.0; pos1=0.0; pos2=0.0; pos3=0.0
vel0=0.0; vel1=0.0; vel2=0.0; vel3=0.0

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

#wait for the controller to fully setup
for i in range(4):
  time.sleep(1.0)
  print(f'waiting for the EPMC controller: {i+1} sec')

success = controller.clearDataBuffer()
controller.writeSpeed(v, v)
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
      success, val0, val1, val2, val3 = controller.readMotorData()
      if success: # only update if read was successfull
        pos0 = val0
        pos1 = val1
        vel0 = val2
        vel1 = val3

      print(f"motor0_readings: [{pos0}, {vel0}]")
      print(f"motor1_readings: [{pos1}, {vel1}]")
      print("")
    except:
      pass
    
    readTime = time.time()

```