
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
- you'll also need the have the CH340 driver installed (especially for windows or mac)


## How to Use the Library
- Ensure you have the **`EPMC MODULE`** interfaced with your preferred motors, setup the encoder and PID parameters with the **`epmc_setup_application`**.

- Download (by clicking on the green Code button above) or clone the repo into your PC using **`git clone`**
> [!NOTE]  
> you can use this command if you want to clone the repo:
> 
> ```git clone https://github.com/robocre8/epmc_python.git```

- check the serial port the driver is connected to:
  ```shell
  ls /dev/ttyU* # for four motor support
  ```
  > you should see /dev/ttyUSB0 or ... and so on

- A simple way to get started is simply to try out and follow the example code.

- You can add it to your code, and start using it.

#

## Basic Library functions and usage

- connect to EPMC module
  > controller = EPMCSerialClient()
  >
  > controller.connect("port_name or port_path")

- clear speed, position, e.t.c data buffer on the EPMC module
  > controller.clearDataBuffer() # returns bool -> success

- send target angular velocity command
  > controller.writeSpeed(motor0_TargetVel, motor1_TargetVel, motor2_TargetVel, motor3_TargetVel)

- send PWM command
  > controller.writePWM(motor0_PWM, motor1_PWM, motor2_PWM, motor3_PWM)

- set motor command timeout
  > controller.setCmdTimeout(timeout_ms)

- get motor command timeout
  > controller.getCmdTimeout() # returns bool, float -> success, motor_command_timeout_ms

- read motors angular position and angular velocity
  > controller.readMotorData() # returns bool, tuple(size=8floats) -> success, (pos0, pos1, pos2, pos3, vel0, vel1, vel2, vel3)

- read motors angular position
  > controller.readPos() # returns bool, tuple(size=4floats) -> success, (pos0, pos1, pos2, pos3)

- read motors angular velocity
  > controller.readVel() # returns bool, tuple(size=4floats) -> success, (vel0, vel1, vel2, vel3)

- read motor maximum commandable angular velocity
  > controller.getMaxVel(motor_no) # returns bool, float -> success, max_vel
  > maxVel0 or maxVel1 based on the specified motor number

- while these function above help communicate with the already configure EPMC module, more examples of advanced funtions usage for parameter tuning can be found in the [epmc_setup_application](https://github.com/robocre8/epmc_setup_application) source code