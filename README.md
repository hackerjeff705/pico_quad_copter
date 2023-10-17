# pico_quad_copter

## Flight controller code overview

The flight controller code is responsible for reading the data from the various sensors, calculating the PID outputs, and setting the speed of the motors.

### Sensor input

The first section of the code reads the data from the various sensors on the flight controller. This includes the gyro data, the accelerometer data, and the receiver input data.

The gyro data is used to measure the angular rate of the drone. The accelerometer data is used to measure the acceleration of the drone. The receiver input data is used to control the drone.

### PID controller

The next section of the code calculates the PID outputs for the roll, pitch, and yaw axes. The PID controller is a control loop feedback mechanism that is used to maintain a desired setpoint. In this case, the desired setpoint is the angle of the drone.

The PID controller calculates the PID outputs using the following equation:
