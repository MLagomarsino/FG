
import serial
import time
import threading

from geometry_msgs.msg import Point

startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False


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


def callback():

    x = Point.x
    z = Point.z

    if(x > 30):
        sendToArduino("+0.5 +0.5 -0.5 -0.5")
    elif(x < -30):
        sendToArduino("-0.5 -0.5 +0.5 +0.5")
    else:
        if(z > 20):
            sendToArduino("+0.5 +0.5 +0.5 +0.5")
        else:
            sendToArduino("+0.0")

setupSerial(115200, "/dev/ttyACM0")

subscriber = rospy.Subscriber("/ball_coord", Point, callback,  queue_size = 1)

try:
    rospy.spin()
except KeyboardInterrupt:
    print "Shutting down ROS Image feature detector module"
    
for n in range(0, 1000000):
    rec()