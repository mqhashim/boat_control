#!/usr/bin/python3

# Node for mapping the boat
import rospy

from geographic_msgs.msg import GeoPose

import folium

import webbrowser
from selenium import webdriver
import os

class Map:
    def __init__(self):

        # Init ros node
        rospy.init_node('map',anonymous = False)
        self.ns = rospy.get_namespace()

        rospy.on_shutdown(self.shutdown)


        # Set up rate
        # update map every 2 seconds by default
        rate = rospy.get_param('~rate',1/5)
        self.rate = rospy.Rate(rate)

        # subscribe to position
        self.geo_sub = rospy.Subscriber(self.ns+'geo_pose',GeoPose,self.geo_pose_callback)

        # get map file location
        self.map_location = '/tmp/'+self.ns+'map.html'

        # set up map object
        self.map = None
        self.map_launched = False
        self.browser = None

        # set up locations
        self.location = None
        self.locations = []

    def loop(self):
        while not rospy.is_shutdown():
            if self.location != None:
                if (self.map == None):
                    # first location
                    self.map = folium.Map(location = self.location,zoom_start=12)
                    folium.Marker(self.location,popup='Start').add_to(self.map)
                else :
                    last_point =self.locations[-1]
                    dist = self.distance(point,last_point)
                    if (dist>0.00000001):
                        # new point is far enough
                        folium.Marker(point).add_to(self.map)
                        folium.PolyLine(locations=[point,last_point],line_opacity=0.5).add_to(self.map)
                self.locations.append(self.location) 
            if self.map != None:
                self.map.save(self.map_location)
                if not self.map_launched:
                    self.map_launched =True
                    self.browser = webdriver.Firefox()
                    self.browser.get('file://'+os.path.realpath(self.map_location))
                else:
                    self.browser.refresh()
            
            self.rate.sleep()

    def shutdown(self):
        #placeholder
        x = 5



    def geo_pose_callback(self,geo_pose):
        lat = geo_pose.position.latitude
        lng = geo_pose.position.longitude
        self.location  = [lat,lng]
        

              

            

    def distance(self,l1,l2):
        lat_diff = l1[0] - l2[0]
        lng_diff = l1[1] - l1[1]
        dist = (lat_diff**2 + lng_diff**2)**0.5
        return dist

if __name__ == '__main__':
    map = Map()
    map.loop()