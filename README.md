# pico_quad_copter Flight controller code overview
The flight controller code is responsible for reading the data from the various sensors, calculating the PID outputs, and setting the speed of the motors.

## Sensor input

The first section of the code reads the data from the various sensors on the flight controller. This includes the gyro data, the accelerometer data, and the receiver input data.

The gyro data is used to measure the angular rate of the drone. The accelerometer data is used to measure the acceleration of the drone. The receiver input data is used to control the drone.

## PID controller

The next section of the code calculates the PID outputs for the roll, pitch, and yaw axes. The PID controller is a control loop feedback mechanism that is used to maintain a desired setpoint. In this case, the desired setpoint is the angle of the drone.

The PID controller calculates the PID outputs using the following equation:

```PID output = Kp * error + Ki * integral + Kd * derivative```

where:
* ```Kp``` is the proportional gain
* ```Ki``` is the integral gain
* ```Kd``` is the derivative gain
* ```error``` is the difference between the desired setpoint and the current value
* ```integral``` is the sum of the past errors
* ```derivative``` is the rate of change of the error signal

The PID outputs are then limited to a maximum value to prevent overdriving the motors.

The calculate_pid() function that you provided is used to calculate the PID outputs for the roll, pitch, and yaw axes of a drone. The PID outputs are then used to control the speed of the motors.

The function works as follows:
* For each axis (roll, pitch, and yaw):
  * Calculate the PID error by subtracting the desired angular rate (PID setpoint) from the actual angular rate (gyro input).
  * Calculate the integral term by adding the product of the PID error and the integral gain to the integral memory variable.
  * Calculate the derivative term by subtracting the previous PID error from the current PID error and multiplying by the derivative gain.
  * Calculate the PID output by summing the proportional, integral, and derivative terms.
  * Limit the PID output to the maximum value for that axis.

## Motor control

The last section of the code sets the speed of the motors based on the PID outputs. The speed of the motors is controlled by sending a PWM signal to the electronic speed controllers (ESCs).

The ESCs then convert the PWM signal to a DC voltage, which controls the speed of the motors.

## Working systems

The flight controller code can be divided into the following working systems:
* Sensor input system: This system reads the data from the various sensors on the flight controller.
* PID controller system: This system calculates the PID outputs for the roll, pitch, and yaw axes.
* Motor control system: This system sets the speed of the motors based on the PID outputs.

These three systems work together to control the flight of the drone. The sensor input system provides the data needed by the PID controller system to calculate the PID outputs. The PID controller system calculates the PID outputs needed to maintain the desired angle of the drone. The motor control system then sets the speed of the motors based on the PID outputs, which causes the drone to move in the desired direction.

## Additional notes

* The flight controller code is a complex system, but it can be broken down into these three basic working systems. By understanding how these systems work together, you can better understand how the flight controller controls the flight of the drone.
* The PID gains are important parameters that affect the performance of the flight controller. The PID gains need to be tuned to achieve optimal performance.
* The flight controller code is typically written in C or C++.
