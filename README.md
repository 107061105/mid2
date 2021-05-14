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
In the thread function, user will detect gesture.

![image](https://github.com/107061105/mid2/blob/main/47471.jpg)

### 2.
Extract features of the saved accelerator data values

After the PC/Python send the stop command, please send another command to retrieve the saved feature data.
Please plot the following together in two sub figures aligned with the event sequence order in PC/Python:
The classified gesture events.
The extracted features.
