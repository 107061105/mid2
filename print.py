import matplotlib.pyplot as plt
import numpy as np
import serial
import time

serdev = '/dev/ttyACM0'
s = serial.Serial(serdev)
for k in range(0, 10):
    a = s.readline()
    gestureID = int(a)
    if (gestureID == 0):
        print("gestureID = RING:\n\r"
        "          *       \n\r"
        "       *     *    \n\r"
        "     *         *  \n\r"
        "    *           * \n\r"
        "     *         *  \n\r"
        "       *     *    \n\r"
        "          *       \n\r")
    elif (gestureID == 0):
        print("gestureID = SLOPE:\n\r"
        "        *        \n\r"
        "       *         \n\r"
        "      *          \n\r"
        "     *           \n\r"
        "    *            \n\r"
        "   *             \n\r"
        "  *              \n\r"
        " * * * * * * * * \n\r")
    elif (gestureID == 0):
        print("gestureID = DOWN:\n\r"
        "        *        \n\r"
        "        *        \n\r"
        "        *        \n\r"
        "        *        \n\r"
        "        *        \n\r"
        "      * * *      \n\r"
        "       ***       \n\r"
        "        *        \n\r")
    print("\n\nSequence Number = ", k, "\n")
    a = s.readline()
    num = int(a)
    y = np.arange(0,num, 1)
    for x in range(0, num):
        line=s.readline() # Read an echo string from B_L4S5I_IOT01A terminated with '\n'
        y[x] = float(line)
    t = np.arange(0,num, 1)
    fig, ax = plt.subplots()
    ax[0].plot(t,y)
    ax[0].set_xlabel('Time')
    ax[0].set_ylabel('Amplitude')
    plt.show()
s.close()