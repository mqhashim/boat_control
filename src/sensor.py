#!/usr/bin/python3

# importing rospy 
import rospy

# importing messages
from std_msgs.msg import String

import time

class Sensor:
    def __init__(self):

        # Init ros node
        rospy.init_node('sensor',anonymous = False)
        self.ns = rospy.get_namespace()

        rospy.on_shutdown(self.shutdown)

        # Set up rate
        rate = rospy.get_param('~rate',1)
        self.rate = rospy.Rate(rate)

        
        # subscribe to sensor
        self.sensor_sub = rospy.Subscriber(self.ns+'sensor',String,self.sensor_callback)
        
        # get map file location
        self.sensor_file_location = '/tmp/'+self.ns+'sensor_data.csv'
        self.data_file = open(self.sensor_file_location,'w')
        self.data_file.write('timestamp,channel,type,value,lat,lng\n')

    def loop(self):
        while not rospy.is_shutdown():
            if self.map != None:
                self.map.save(self.map_location)
                if not self.map_launched:
                    self.map_launched =True
                    self.browser = webdriver.Firefox()
                    self.browser.get('file://'+os.path.realpath(self.map_location))
                else:
                    self.browser.refresh()
            self.rate.sleep()

                    


    def sensor_callback(self,data):
        #data should look like:
        #channel,type,value,lat,lng
        timestamp = time.time()
        line = str(timestamp) + ',' + data+'\n'
        self.data_file.write(line)
    
    def shutdown(self):
        self.data_file.close()


            

    def distance(self,l1,l2):
        lat_diff = l1[0] - l2[0]
        lng_diff = l1[1] - l1[1]
        dist = (lat_diff**2 + lng_diff**2)**0.5
        return dist

if __name__ == '__main__':
    map = Map()
    map.loop()