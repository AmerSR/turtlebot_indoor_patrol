#! /usr/bin/env python

import rospy
import sys
from std_msgs.msg import UInt8
from collections import deque
from explore import WaypointNav

class Patrol():
    def __init__(self):
        rospy.init_node('patrolling_Mall', anonymous=False)

        # The patrolling scenario depends on fixed waypoints navigation
        # Start by defining all target waypoints the robot could need
        self.exploring_waypoints = [[6, 3, 90],[7, 9, 0],[18, 8, -90],[18, 3, 180]]
        self.stores_waypoints =    [[ 2,  2, 180], [ 2,  7, 180], [ 8.0, 11.0,  90], 
                                    [16, 11.0,  90], [22.0, 8.0, 0], [22.0, 5.0, 0], 
                                    [22.0, 3, 0], [18.0, 1.0, -90], [12,  1.0, -90], 
                                    [14.0, 6.0, 180], [10.0, 7.0, 0], [10.0, 5.0,0]]

        # Initialize an empty double-ended queue for shops
        self.emergemcy_list = deque()
        # A boolean operator for any emergency request received
        self.no_emergency_request = True
        # A variable referring to the index of the stored exploring waypoints 
        self.explore_point_num = None

        self.nav = WaypointNav()

        # Subscribe to topic /SOS_request that publishes emergency calls
        rospy.Subscriber("SOS_request", UInt8, self.SOSCallback)

        self.start()


    def start(self):
        self.NearestExploringPoint()

        while not rospy.is_shutdown():
            # If any valid number is found in the emergemcy list, go there immediately          
            if (len(self.emergemcy_list) > 0):
                # Fitch the first shop number in the emergency queue
                shop_num = self.emergemcy_list.popleft()
                print("Emergency in shop number " + str(shop_num))

                # Fitch the waypoint of the corresponding shop
                way_p = self.stores_waypoints[shop_num-1]
                # Pass the pose to the navigation class
                self.nav.Navigate(way_p[0], way_p[1], way_p[2])

                self.NearestExploringPoint()

            # If no SOS, patrol normally
            else:
                self.no_emergency_request = True

                way_p = self.exploring_waypoints[ self.explore_point_num ]
                self.nav.Navigate(way_p[0], way_p[1], way_p[2])

                # Set to the next waypoint
                self.explore_point_num = (self.explore_point_num + 1) % len(self.exploring_waypoints)

    # Always assign the nearest exploration point as the target
    def NearestExploringPoint(self):
        index = 0
        minimun_d = None

        for i in range(len(self.exploring_waypoints)):
            way_p = self.exploring_waypoints[i]
            distance = self.nav.ComputeDistance(way_p)

            if (minimun_d == None or minimun_d > distance):
                minimun_d = distance
                index = i

        self.explore_point_num = index

    # Any SOS request will intrupt the normal exploration behaviour and be prioritized
    def SOSCallback(self, msg):
        
        shop_n = msg.data
        if (shop_n < 1 or shop_n > 12):
            return

        # Stack the SOS requests (first come first serve)
        self.emergemcy_list.append(shop_n)
        print("SOS from shop number " + str(shop_n) + " is received!")

        # Cancel the explorint routine and deal with the emergency first
        if (self.no_emergency_request):
            self.no_emergency_request = False
            self.nav.cancelGoal()

if __name__=='__main__':
    try:
        Patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("Halt.")
