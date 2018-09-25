#!/usr/bin/python

import serial
import sys
import rospy
from std_msgs.msg import String
import numpy as np
from math import sqrt

portname = "/dev/ttyACM0"
s = serial.Serial(portname)

def getLowBits(val):
    return val & 0x7F

def getHighBits(val):
    return (val >> 7) & 0x7F

def getPos(serialPort, channel):
    global s
    functionBytes = chr(0x90)
    serialPort.write(functionBytes + chr(channel))
    currPosLSB = ord(s.read())
    currPosMSB = ord(s.read())
    pos = (currPosMSB << 8) + currPosLSB
    return int(pos)

def moveToTargetThroughSer(serialPort, channel, target):
    print "moving to target:", target
    functionBytes = chr(0x84)
    convTarget = target*4
    serialPort.write(functionBytes + chr(channel) + chr(getLowBits(convTarget)) + chr(getHighBits(convTarget)))

def callback(data):
    global s
    #minTarget = 992*4
    #maxTarget = 2000*4
    #mid = 1500*4
    payload = data.data
    vals = payload.split(" ")
    (xdiff, ydiff) = (int(vals[0]), int(vals[1]))
    print "xdiff:", xdiff, "ydiff:", ydiff
    currX = getPos(s, 0)
    currY = getPos(s, 2)
    newX = currX if xdiff == 0 else currX / 4 + (xdiff / abs(xdiff)) * 3
    newY = currY if ydiff == 0 else currY / 4 + -1 * (ydiff / abs(ydiff)) * 3
    print "calcd newx:", newX, "calcd newy:", newY
    moveToTargetThroughSer(s, 0, newX)
    moveToTargetThroughSer(s, 2, newY)

def main():

    global s
    moveToTargetThroughSer(s, 0, 1500)
    moveToTargetThroughSer(s, 2, 1500)

    # Initialise ROS node
    rospy.init_node('servo')
    rospy.Subscriber('/processing/servo_commands', String, callback)

    #s.write(chr(0x89) + chr(0) + chr(getLowBits(125)) + chr(getHighBits(125)))
    #s.write(chr(0x89) + chr(2) + chr(getLowBits(125)) + chr(getHighBits(125)))

    #while not rospy.is_shutdown():
    #    k = cv2.waitKey(5) & 0xFF
    #    rospy.sleep(0.05)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
