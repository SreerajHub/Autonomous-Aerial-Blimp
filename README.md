# Autonomous-Aerial-Blimp
An autonomous aerial blimp which coordinates with a ground robot in monitoring a construction site using computer vision techniques.


Indoor Blimp is a UAV to probe Indoor environments in a construction site. This device is a simple balloon envelope in addition to a payload carrier with different components. This device is able to send visual data to a mobile base station. This device can also navigate through doors and passages by using sensors for autonomous navigation.<br>

**Goals**
* Capture visual data from its own camera <br>
* Be able to rotate the camera to the needed direction<br>
* Send and receive required data to the mobile base station(HUSKY) using ROS <br>
* Move in indoor environments of a construction site<br>


## 1. Pan and Tilt 
Pan and tilt mechanism were used in order to rotate and move the camera to the needed direction. Pan and tilt mechanism needs two servos. The servos were connected directly to the raspberry pi GPIO ports. This may reduce the speed of the rotation due to low current in the raspberry pi.<br> 
<br>

## 2. Camera<br>
An Arducam Mini camera module is used to capture image and video data. This camera is mounted on the pan and tilt mechanism so the raspberry pi is able to move its field of view to any needed direction.
The camera module has the following characteristics:
* 5 Megapixel camera <br>
* Power: 1.5V Core, 2.6 - 3.0V Analog, 1.7 - 3.0V I/O <br>
* 5 megapixel (2592×1944): 15 fps (and any size scaling down from 5 megapixel) <br>
* 1080p (1920×1080): 30 fps  <br>
* 720p (1280×720): 60 fps <br>
* VGA (640×480): 60 fps <br>
* QVGA (320×240): 120 fps <br>
<br>
<div align="center">
<img src="https://images-na.ssl-images-amazon.com/images/I/61XA4QT5DQL._SL1000_.jpg" width="350">
</div>
<br>



## 3. Motors<br> 
For altitude, yaw, and forward motion, four DC Hubsan motors were used. Two motors face upwards to control altitude and two motors face forward to control movement and yaw. 
The motors are powered by 3.7 volt LiPo batteries to reduce the current draw from the battery and to ensure that the motors do not overdraw from the Raspberry Pi Zero. 
The motors are controlled by two h-bridge boards that allow the motors to be controlled through pulse width modulation (PWM). Below are the characteristics of the motor:
* The motors can draw a peak 1.2 amps<br>
* They can run up to 5 volts<br> 
* Each motor weighs around 4 grams <br>
* Each motor has a thrust capability of 9 grams <br>
* The thrust to weight ratio of the motor is 0.44 <br>
<br>
<div align="center">
<img src="https://images-na.ssl-images-amazon.com/images/I/41W4c9uP6DL.jpg" width="350">
</div>
<br>


## 4. H-Bridge <br>
Sourcing the motors from the Raspberry Pi could draw too much current from either the Raspberry Pi GPIO pins or the battery. Therefore, 
a h-bridge expansion board and a separate power system were used to ensure that the safe use of the motor system. 


## 5. IMU<br>
An inertial measurement unit (IMU) is an electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of accelerometers and gyroscopes, sometimes also magnetometers.
An Adafruit BNO055 was used for the IMU. The characteristics of the breakout board are given below:<br>

* ARM Cortex M0 Processor<br>
* 9 DOF IMU Sensor (3 DOF Accelerometer, 3 DOF Magnetometer, 3 DOF Gyroscope)<br>
* Embedded temperature sensor<br> 
* Can output filtered absolute orientation, angular velocity, acceleration vector, etc. <br>
* 3.3 - 5.0 V Power Input <br>
* I2C and UART communication interfaces <br>

<br>
<div align="center">
<img src="https://cdn-learn.adafruit.com/assets/assets/000/024/666/large1024/sensors_pinout.jpg?1429726694" width="350">
</div>



## 8. Balloon <br>
The balloon envelope should be able to carry more than 200 grams of payload. For this device four 36-inch balloon were used (with pure helium).
It is also possible to heat Mylar sheet to create custom shaped balloon.

## 9. Raspberry Pi Three <br>
A raspberry pi Three was used as the main computation core on the blimp. This board has a built-in Wi-Fi connection. So this board is able to connect to the main router on the mobile base station(HUSKY). This board is also able to receive messages from different components and send them to the main JETSON on the mobile base station in ROS format.<br> 
<br>
<div align="center">
<img src="https://www.raspberrypi.org/blog/raspberry-pi-3-on-sale/" width="350">
</div>
## 10. Batteries<br>
The device uses two different types of batteries. 
1. A battery for raspberry pi Three 
2. batteries for the motors which directly connect to the h-bridges

## ROS Integration

The ROS topics used are:

Camera: /image_indoor_blimp/compressed/ 
	This is a topic used for sending Compressed image from pi camera (RGB, resolution of 410 x 308)


IMU: /indoor_imu/data/
	This is a topic which publishes the orientation (qaternion) and acceleration
    /indoor_imu/raw/: 
	This is a topic which publishes the orientation (qaternion) and acceleration
    /imu/yaw : 
	This is a topic which publishes the Yaw data (Euler angle)


Ultrasonic: /ultrasonic : ultrasonic distance value
	This is a topic which publishes the ultrasonic distance measurements used for obstacle detection


Subscribed Topics:
Key Input: /key
	This is a topic for keyboard output - sends control commands for navigation of blimp
