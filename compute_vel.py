# To be loaded on the raspberry - must communicate with the Arduino

import rospy
import serial
import time
import threading
import math

from geometry_msgs.msg import Twist

startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False
oldRPS = "+0.0 +0.0 +0.0 +0.0" # to keep track of previous velocities
RPS = "" # current velocities to be sent only if different from the old ones

def rec():
    arduinoReply = recvLikeArduino()
    if not (arduinoReply == 'XXX'):
        print("Time %s  Reply %s" % (time.time(), arduinoReply))


def setupSerial(baudRate, serialPortName):
    global serialPort
    serialPort = serial.Serial(port=serialPortName, baudrate=baudRate, timeout=0, rtscts=True)
    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))
    waitForArduino()


def sendToArduino(stringToSend):

    # this adds the start and end-markers before sending
    global startMarker, endMarker, serialPort

    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)
    serialPort.write(stringWithMarkers.encode('utf-8'))  # encode needed for Python3
    print(stringToSend)


def recvLikeArduino():

    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") # decode needed for Python3

        if dataStarted:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True

    if messageComplete:
        messageComplete = False
        return dataBuf
    else:
        return "XXX"


def waitForArduino():

    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded

    print("Waiting for Arduino to reset")

    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'):
            print(msg)


def callbackVelocities(vel):
	global RPS
	global oldRPS

	if (vel.angular.z != 0):
		wheelBL = vel.angular.z * (+1.0)
		wheelFL = vel.angular.z * (+1.0)
		wheelFR = vel.angular.z * (-1.0)
		wheelBR = vel.angular.z * (-1.0)
	else:
		wheelBL = vel.linear.x * (-1.0) + vel.linear.y * (+1.0)
		wheelFL = vel.linear.x * (+1.0) + vel.linear.y * (+1.0)
		wheelFR = vel.linear.x * (-1.0) + vel.linear.y * (+1.0)
		wheelBR = vel.linear.x * (+1.0) + vel.linear.y * (+1.0)
	
    # check the signs of the velocities, so that when it is positive, add the +
    if(wheelBL >= 0):
		wheelBLstr = "+" + str(wheelBL)
	else:
		wheelBLstr = str(wheelBL)
	if(wheelFL >= 0):
		wheelFLstr = "+" + str(wheelFL)
	else:
		wheelFLstr = str(wheelFL)
	if(wheelFR >= 0):
		wheelFRstr = "+" + str(wheelFR)
	else:
		wheelFRstr = str(wheelFR)
	if(wheelBR >= 0):
		wheelBRstr = "+" + str(wheelBR)
	else:
		wheelBRstr = str(wheelBR)

	RPS = wheelBLstr + " " + wheelFLstr + " "+ wheelFRstr + " " + wheelBRstr
	print(RPS)

	if(RPS != oldRPS):
        # send the velocities to the arduino only if different from the previous ones
		sendToArduino(RPS)
		oldRPS = RPS

setupSerial(115200, "/dev/ttyACM0")
rospy.init_node('compute_vel')
# subscribe to the topic where the robot velocities are published
subscriber = rospy.Subscriber("/cmd_vel", Twist, callbackVelocities)
   
rospy.spin()
   
