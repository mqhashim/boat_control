#!/usr/bin/python3

# importing rospy 
import rospy

# importing messages
from std_msgs.msg import String
from std_msgs.msg import Int8

import time

class WaypointHandler:

    GOING = 0
    PAUSED = 1
    DONE = 2
    CANCELLED = 3
    OFF = 4
    UNKNOWN = 5


    def __init__(self):

        # Init ros node
        rospy.init_node('waypoint_handler',anonymous = False)
        self.ns = rospy.get_namespace()

        rospy.on_shutdown(self.shutdown)

        # Set up rate
        rate = rospy.get_param('~rate',1)
        self.rate = rospy.Rate(rate)

        # string for each waypoint
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.send_next = False
        self.last_waypoint_ended = 0
        self.moving_to_waypoint = False

        # publish to waypoint
        self.waypoint_pub = rospy.Publisher(self.ns+'waypoint',String,queue_size=1)
        
        # subscribe to waypoint_list
        self.waypoint_list_sub = rospy.Subscriber(self.ns+'waypoint_list',String,self.waypoint_list_callback)
        
        # subscribe to waypoint_state
        self.waypoint_list_sub = rospy.Subscriber(self.ns+'waypoint_state',Int8,self.waypoint_state_callback)

    def is_waypoint(self,s):
        try:
            lat,lng = s.split(',')
            float(lat)
            float(lng)
            return True
        except ValueError:
            return False

    def waypoint_list_callback(self,msg):
        data = msg.data
        waypoints = data.split('\n')
        waypoints = list(filter(lambda x: self.is_waypoint(x),waypoints))
        self.current_waypoints = waypoints
        self.current_waypoint_index = 0
        self.send_next = True
        self.last_waypoint_ended = 0

    def waypoint_state_callback(self,msg):
        state = msg.data
        if (state == self.GOING or state == self.PAUSED):
            # do nothing
            return
        elif (state == self.DONE):
            if (self.moving_to_waypoint):
                # Done with current waypoint, move on to next

                # TODO: add anything that needs to be done when a waypoint
                # is reached here
                print('DONE :D')

                self.moving_to_waypoint = False
                self.current_waypoint_index += 1
                self.send_next = True
                self.last_waypoint_ended = time.time()
        else :
            # cancel everything
            self.current_waypoints = []
            self.current_waypoint_index = 0
            self.send_next = False
            self.last_waypoint_ended = 0
            self.moving_to_waypoint = False
        
        return

    def loop(self):
        while not rospy.is_shutdown():
            if self.send_next and (abs(time.time() - self.last_waypoint_ended)>4.9) and (0<=self.current_waypoint_index<len(self.current_waypoints)):
                # send next waypoint :D
                self.send_next = False
                self.moving_to_waypoint = True
                msg = String()
                msg.data = self.current_waypoints[self.current_waypoint_index]
                self.waypoint_pub.publish(msg)
    
            self.rate.sleep()

    def shutdown(self):
        return 5


        

if __name__ == '__main__':
    waypoint_handler = WaypointHandler()
    waypoint_handler.loop()