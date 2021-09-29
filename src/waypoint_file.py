#!/usr/bin/python3

# importing rospy 
import rospy

# importing messages
from std_msgs.msg import String

class WaypointFileReader:

    def __init__(self):

        # Init ros node
        rospy.init_node('waypoint_file_reader',anonymous = False)
        self.ns = rospy.get_namespace()

        filename = rospy.get_param('~waypoints_file','waypoints.txt')

        f = open(filename,'r')

        msg = String()
        msg.data = f.read()

        print(msg)

        f.close()

        # publish to waypoint_list
        self.waypoint_list_pub = rospy.Publisher(self.ns+'waypoint_list',String,queue_size=1)
        rospy.sleep(1)
        self.waypoint_list_pub.publish(msg)

        rospy.sleep(5)

        rospy.signal_shutdown('done :D')



  


        

if __name__ == '__main__':
    waypoint_handler = WaypointFileReader()