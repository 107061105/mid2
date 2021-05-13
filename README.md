# mid2

## I. Introduction

In this assignment, we will made a tilt angle detector.

## II. Equipment List
### 1.PC or notebook
### 2.B_L4S5I_IOT01A
### 3.uLCD display

![image](https://github.com/107061105/HW3/blob/master/47276.jpg)

## III. Implement

Complete work flow:

mbed run a RPC loop with two custom functions (operation modes): (1) gesture UI, and (2) tilt angle detection
### 1.
PC/Python use RPC over serial to send a command to call gesture UI mode on mbed, for example,  type "/a/run" to start:

![image](https://github.com/107061105/HW3/blob/master/Screenshot%20from%202021-05-09%2006-09-44.png)

The gesture UI function will start a thread function. In this mode, we let led1 to blink to indicate.
In the thread function, user will use gesture to select from a few threshold angles. For example, gesture "CIRCLE" is 30 degree; "SLOPE" is 45 degree, and "DOWN" is 60 degree.

![image](https://github.com/107061105/HW3/blob/master/Screenshot%20from%202021-05-09%2006-35-02.png)

Before confirmation, current selection will show the selection on uLCD.

![image](https://github.com/107061105/HW3/blob/master/47273.jpg)
After the selection is confirmed with a user button, the selected threshold angle is published through WiFi/MQTT to a broker (run on PC).

![image](https://github.com/107061105/HW3/blob/master/Screenshot%20from%202021-05-09%2006-37-03.png)

After the PC/Python get the published confirmation from the broker, it sends a command to mbed to stop the guest UI mode. Therefore, the mbed is back to RPC loop. Also PC/Python will show the selection on screen.

![image](https://github.com/107061105/HW3/blob/master/47274.jpg)

### 2
PC/Python use RPC over serial to send a command to call tilt angle detection mode on mbed.
Before we start the tilt measurement, we should measure the reference acceleration vector. we let led1 and led2 to blink to show this initialization process for a user to place the mbed on table. We assume this is the stationary position of the mbed and use accelerometer to measure this acceleration vector as the reference (the direction should align with gravity.

![image](https://github.com/107061105/HW3/blob/master/Screenshot%20from%202021-05-09%2006-37-19.png)

The tilt angle function will start a thread function.
After we initialize the gravity vector, we let led3 and led4 blinking to indicate for a user to tilt the mbed. In this mode, we use the accelerometer vectors to detect the tilt angles for each predefined period, e.g., 100ms. A MQTT message will publish if mbed tilts over the selected threshold degree to the stationary position.
For each predefined period (100ms as above), please show the tilt angle on uLCD (so a user can determine how to tilt mbed).

![image](https://github.com/107061105/HW3/blob/master/47275.jpg)

If the tilt angle is over the selected threshold angle, mbed will publish the event and angle through WiFi/MQTT to a broker.
After the PC/Python get a preset number of tilt events, e.g., 10, from the broker, it sends a command to mbed to stop the tilt detection mode. Therefore, the mbed is back to RPC loop. Also PC/Python will show all tilt events on screen.
