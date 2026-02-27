# Self - Balancing Robot

This project aims to realize a "self-balancing robot" during the laboratory activity of the course "Modelling and Control of Electric Drives" in University of Padova.

A self-balancing robot is a device on wheels that automatically stabilizes to maintain the initial angle - in most cases, the upright position. 

Additional infos regarding the implementation are written in the [final report](https://github.com/macscEE/selfBalancingRobot_fw/blob/main/report/main_report.pdf)

<img width="4000" height="1848" alt="jamal_01" src="https://github.com/user-attachments/assets/52bbe353-6f55-4fcd-8159-dd5698474beb" />

### Components and Schematics
<img width="1238" height="468" alt="image" src="https://github.com/user-attachments/assets/c5d0675a-832e-4293-bc7f-7208b232f96a" />

| Component  | Description |
| ------------- | ------------- |
| ESP32 | Microcontroller dev board |
| MPU6050  | Inertial unit |
| DRV8833 | DC motor drivers (2 boards needed). Note: add external bulk capacitance to supply the motors during quick transients |
| Battery | In our case, two 21700 cells in series. Can be adapted to other configurations, as long as voltage is between 6V and 8V (alternatively, the battery voltage can be regulated to exactly 6V, but this requires a fast and clean DC/DC converter) |

In particular, the DC motors we used have the following specs:

| Motor Param | Value |
| ------------- | ------------- |
| Nominal Voltage | 6 V |
| Gear Ratio | 120 |
| Torque Constant | 0.336 Vs |
| Armature Resistance | 3.5 Ohm |

The motors are fitted with magnetics encoders, but we do not use them in this project. They could be used to implement position control in addition to angle control.

### Modelling and Control
The system has been modeled as a simple inverse pendulum and linearized around the zero angle.
A complementary filter elaborates the data coming from the accelerometer and gyroscope in the MPU6050 controller to provide a reliable and quick angle estimation. After that, a PID regulator is employed to stabilize the system.

<img width="1836" height="312" alt="image" src="https://github.com/user-attachments/assets/f169db6e-2ba4-40fa-b4fa-bb331caacea5" />

The code is organized in a very simple Finite State Machine:

<img width="1282" height="862" alt="image" src="https://github.com/user-attachments/assets/c0dc59ec-34c9-489f-bcfe-4e83bef0806a" />
