
## Easy PID Motor Controller (EPMC) Python Library
This library helps communicate with the **`Easy PID Motor Controller Module`** in your PC or microcomputer-based python projects, with the [epmc_setup_application](https://github.com/robocre8/epmc_setup_application).

> you can use it in your microcomputer robotics project (e.g Raspberry Pi, PC, etc.)

A simple way to get started is simply to try out and follow the example code


## Dependencies
- you'll need to pip install the pyserial library
  ```shell
    pip3 install pyserial   //linux or mac
    pip install pyserial   //windows
  ```

## Dependencies
- you'll need to pip install the pyserial library
  > pip3 install pyserial


## How to Use the Library
- Ensure you have the **`EPMC MODULE`** interfaced with your preferred motors, setup the encoder and PID parameters with the **`epmc_setup_application`**.

- Download (by clicking on the green Code button above) or clone the repo into your PC using **`git clone`**
> [!NOTE]  
> you can use this command if you want to clone the repo:
> 
> ```git clone https://github.com/robocre8/epmc_python.git```

- check the serial port the driver is connected to:
  ```shell
  ls /dev/ttyA* # for two motor support
  ```
  OR
  ```shell
  ls /dev/ttyU* # for four motor support
  ```
  > you should see /dev/ttyACM0 or /dev/ttyUSB0 or ... and so on

- A simple way to get started is simply to try out and follow the example code.

- You can add it to your code, and start using it.


## Basic Library functions and usage (Two Motor Support Control)

- connect to EPMC module
  > controller = EPMCSerialClient()
  >
  > _#ensure you set/call **supportedNumOfMotors()** before **connect()** as below:_
  >
  > controller .supportedNumOfMotors(SupportedNumOfMotors.TWO)
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
  > controller.readMotorData() # returns bool, tuple(size=4foats) -> success, (pos0, pos1, vel0, vel1)

- read motors angular position
  > controller.readPos() # returns bool, tuple(size=2floats) -> success, (pos0, pos1)

- read motors angular velocity
  > controller.readVel() # returns bool, tuple(size=2floats) -> success, (vel0, vel1)

- read motor maximum commandable angular velocity
  > controller.getMaxVel(motor_no) # returns tuple -> (success, max_vel): bool, float
  > maxVel0 or maxVel1 based on the specified motor number

- while these function above help communicate with the already configure EPMC module, more examples of advanced funtions usage for parameter tuning can be found in the [epmc_setup_application](https://github.com/robocre8/epmc_setup_application) source code

#

## Basic Library functions and usage (Four Motor Support Control)

- connect to EPMC module
  > controller = EPMCSerialClient()
  >
  > _#ensure you set/call **supportedNumOfMotors()** before **connect()** as below:_
  >
  > controller .supportedNumOfMotors(SupportedNumOfMotors.FOUR)
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
  > controller.readMotorData() # returns bool, tuple(size=8floats) -> success, (pos0, pos1, pos2, pos3, vel0, vel1, vel2, vel3)

- read motors angular position
  > controller.readPos() # returns bool, tuple(size=4floats) -> success, (pos0, pos1, pos2, pos3)

- read motors angular velocity
  > controller.readVel() # returns bool, tuple(size=4floats) -> success, (vel0, vel1, vel2, vel3)

- read motor maximum commandable angular velocity
  > controller.getMaxVel(motor_no) # returns tuple -> (success, max_vel): bool, float
  > maxVel0 or maxVel1 based on the specified motor number

- while these function above help communicate with the already configure EPMC module, more examples of advanced funtions usage for parameter tuning can be found in the [epmc_setup_application](https://github.com/robocre8/epmc_setup_application) source code