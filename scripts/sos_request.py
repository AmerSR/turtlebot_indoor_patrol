#! /usr/bin/env python

import rospy
import sys
from std_msgs.msg import UInt8

# Publish the number of the SOS shop on topic /SOS_request
def SOSrequest(shopNum):
    pub = rospy.Publisher('SOS_request', UInt8, queue_size=10)
    rospy.init_node('SOS_request', anonymous=True)
    pub.publish(shopNum)

# Error message incase the user entered any invalid number
def Invalid():
  print("\nInvalid shop number, please choose a number between 1 and 12\n")

def main():
    if (len(sys.argv) < 2):
        Invalid()
        return

    shopNum = float(sys.argv[1])

    if (not shopNum.is_integer() or shopNum < 1 or shopNum > 12):
        Invalid()
        return
# If the number is valid, publish it on topic /SOS_request
    SOSrequest(shopNum)


if __name__ == '__main__':
    main()
