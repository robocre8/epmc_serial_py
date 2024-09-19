
## Easy PID Motor Controller (EPMC) Python Library
This library helps communicate with the **`Easy PID Motor Controller Module`** (i.e **`L298N EPMC MODULE`** or a **`CUSTOM EPMC INTERFACE BOARD`**) in your PC or microcomputer-based python projects, with the [epmc_setup_application](https://github.com/samuko-things-company/epmc_setup_application).

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
- Ensure you have the **`L298N EPMC MODULE`** or a **`CUSTOM EPMC INTERFACE BOARD`** interfaced with your preferred motors, setup the encoder and PID parameters with the **`epmc_setup_application`**.

- Download (by clicking on the green Code button above) or clone the repo into your PC using **`git clone`**
> [!NOTE]  
> you can use this command if you want to clone the repo:
> 
> ```git clone https://github.com/samuko-things-company/epmc_python.git```

- check the serial port the driver is connected to:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  ```shell
  ls /dev/ttyU*
  ```
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on

- A simple way to get started is simply to try out and follow the example `motor_control.py` code.

- You can copy the **`epmc.py`** file into your python robotics project, import the library as shown in the example `motor_control.py` code, add it to your code, and start using it.


## Basic Library functions and usage

- connect to smc_driver shield module
  > EPMC("port_name or port_path")

- send target angular velocity command
  > .sendTargetVel(motorA_TargetVel, motorB_TargetVel)

- send PWM command
  > .sendPwm(motorA_PWM, motorB_PWM)

- read motors angular position
  > .getMotorsPos() # returns angPosA, angPosB

- read motors angular velocity
  > .getMotorsVel() # returns angVelA, angVelB

- read motorA maximum commandable angular velocity
  > .getMotorAMaxVel() # returns maxVelA

- read motorB maximum commandable angular velocity
  > .getMotorBMaxVel() # returns maxVelB