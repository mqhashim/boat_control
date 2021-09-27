#!/usr/bin/python3

# importing rospy 
import rospy

# importing messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose

# import struct to change floats into bytes
import struct

# import server for handling communication with boat/phone
import server

import geodesy.utm as utm

class ControlBoat():
    def __init__(self):
        #initiate the ros node
        rospy.init_node('MoveBoat',anonymous = False)
        rospy.loginfo('CTRL+C to stop the boat')

        self.ns = rospy.get_namespace()
        print('MoveBoat namespace:',self.ns)

         # get the robots ip and port
         # TODO: setup UDP hole punching (POW!!!)
        self.boat_ip = rospy.get_param('~boat_ip','192.168.86.115')
        self.boat_port = rospy.get_param('~boat_port',11411)

        # get port for communcation with boat/phone
        self.port = rospy.get_param('~port',12345)

        # Set up server for communication with boat
        self.boat_server = server.UDPServer(self.port,self.boat_ip,self.boat_port)

        # which function to call when ctrl+c is pressed
        # in this case we want to stop the boat before closing
        rospy.on_shutdown(self.shutdown)

        # subscribe to vel_cmd
        self.vel_sub = rospy.Subscriber(self.ns+'cmd_vel',Twist,callback = self.vel_callback,queue_size=1)
        self.waypoint_sub = rospy.Subscriber(self.ns+'waypoint',GeoPose,callback = self.waypoint_callback,queue_size=1)

        # setup pose and geoPose publishers
        self.geo_pub = rospy.Publisher(self.ns+'geo_pose',GeoPose,queue_size=1)
        self.pose_pub = rospy.Publisher(self.ns+'pose',Pose,queue_size=1)
        self.sensor_pub = rospy.Publisher(self.ns+'sensor',String,queue_size=1)

        #initialize velocity
        self.velocity = Twist()

        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

        self.register_pose_listener()
        self.register_sensor()

        # set up the velocity publish rate in Hz
        rate = rospy.get_param('~rate',50)
        self.rate = rospy.Rate(rate)

        

    def control_boat(self):
        while not rospy.is_shutdown():
            # create packet
            self.boat_server.get_packet()
            self.boat_server.handle_timeouts()
            # sleep
            # self.rate.sleep()

    def vel_callback(self,msg):
        vel_data = self.twist_to_bytes(msg)
        ticket_number = -1
        self.boat_server.send_command(ticket_number,server.Commands.CMD_SET_VELOCITY,vel_data,None)
        self.velocity = msg

    def waypoint_callback(self,msg):
        print(5) 

    def register_sensor(self):
        def callback(data):
            sensor_data = self.sensor_from_bytes(data)
            ack_ticket = self.int_from_bytes(data[-8:])
            #self.boat_server.ack_sensor_data(ack_ticket)
            # TODO: Add sensor logging HERE
            string = str(sensor_data['channel'])+','+str(sensor_data['type'])+','+str(sensor_data['val'])
            string += ','+str(sensor_data['lat'])+','+str(sensor_data['lng'])
            self.sensor_pub.publish(string)
            print(sensor_data)

        self.boat_server.register_listener(server.Commands.CMD_REGISTER_SENSOR_LISTENER,callback)

    def register_pose_listener(self):
        def callback(data):
            pose,geo_pose = self.pose_from_bytes(data)
            self.geo_pub.publish(geo_pose)
            self.pose_pub.publish(pose)
        self.boat_server.register_listener(server.Commands.CMD_REGISTER_POSE_LISTENER,callback)

    def get_pose(self,callback):
        ticket_number = self.boat_server.get_ticket()
        command = server.Commands.CMD_GET_POSE
        def callback2(data):
            pose = self.pose_from_bytes(data)
            return callback(pose)
        
        self.boat_server.send_command(ticket_number,command,b'',callback2)


    # transforms a python float (C double) into a sequence of bytes
    # big endian
    def double_to_bytes(self,d):
        return bytes(struct.pack('>d',d))
    def double_from_bytes(self,b):
        return struct.unpack('>d',b)[0]
    
    def int_to_bytes(self,i,l):
        return int.to_bytes(i,l,'big',signed=True)
    def int_from_bytes(self,d):
        return int.from_bytes(d,'big',signed = True)


    def twist_to_bytes(self,t):
        x = self.double_to_bytes(t.linear.x)
        y = self.double_to_bytes(t.linear.y)
        z = self.double_to_bytes(t.linear.z)

        rx = self.double_to_bytes(t.angular.x)
        ry = self.double_to_bytes(t.angular.y)
        rz = self.double_to_bytes(t.angular.z)

        return x+y+z+rx+ry+rz

    def twist_from_bytes(self,data):
        if len(data) != 8*6 : 
            return None
        res = Twist()
        res.linear.x = self.double_from_bytes(data[0:8])
        res.linear.y = self.double_from_bytes(data[8:16])
        res.linear.z = self.double_from_bytes(data[16:24])

        res.angular.x = self.double_from_bytes(data[24:32])
        res.angular.y = self.double_from_bytes(data[32:40])
        res.angular.z = self.double_from_bytes(data[40:48])
        return res
    
    def waypoints_to_bytes(self,waypoints):
        res = b''
        num_of_waypoints = self.int_to_bytes(len(waypoints),4)
        res += num_of_waypoints
        for i in range(len(waypoints)):
            # waypoints should be a lat,lng tuple list
            lat,lng = waypoints[i]
            res += self.double_to_bytes(lat)
            res += self.double_to_bytes(lng)
        return res
        

    # TODO: check if needed/ check format
    def waypoints_from_bytes(self,data):
        return None
    
    def sensor_from_bytes(self,data):
        res = {}
        res['channel'] = self.int_from_bytes(data[0:4])
        res['type'] = data[4]
        data = data[5:]
        res['val'] = self.double_from_bytes(data[0:8])
        res['lat'] = self.double_from_bytes(data[8:16])
        res['lng'] = self.double_from_bytes(data[16:24])
        # res['ack'] = self.int_from_bytes(data[24:32])
        return res

    def sensor_from_bytes_temp(self,data):
        res = {}
        res['channel'] = self.int_from_bytes(data[0:4])
        res['type'] = data[4]
        data = data[5:]
        number_of_vals = self.int_from_bytes(data[0:4])
        vals = []
        data = data[4:]
        for i in range(number_of_vals):
            vals.append(self.double_from_bytes(data[8*i:8*(i+1)]))
        res['vals'] = vals
        return res
    
    def pose_from_bytes(self,data):
        easting = self.double_from_bytes(data[0:8])
        northing = self.double_from_bytes(data[8:16])
        alt = self.double_from_bytes(data[16:24])

        rw = self.double_from_bytes(data[24:32])
        rx = self.double_from_bytes(data[32:40])
        ry = self.double_from_bytes(data[40:48])
        rz = self.double_from_bytes(data[48:56])

        zone = data[56]
        N = not not data[57]

        point =  utm.UTMPoint(easting,northing,alt,zone,None)
        geo_point = point.toMsg()
        lat = geo_point.latitude
        lng = geo_point.longitude
        
        pose = Pose()
        pose.position.x = easting
        pose.position.y = northing
        pose.position.z = alt
        pose.orientation.w = rw
        pose.orientation.x = rx
        pose.orientation.y = ry
        pose.orientation.z = rz

        geo_pose = GeoPose()
        geo_pose.position.latitude = lat
        geo_pose.position.longitude = lng
        geo_pose.position.altitude = alt
        geo_pose.orientation.w = rw
        geo_pose.orientation.x = rx
        geo_pose.orientation.y = ry
        geo_pose.orientation.z = rz


        return (pose,geo_pose)
    
    def crumb_from_bytes(self,data):
        return None

    #TODO: fix
    def shutdown(self):
        rospy.loginfo('Stop Boat')
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

        vel_data = self.twist_to_bytes(self.velocity)

        self.boat_server.send_command(-1,server.Commands.CMD_SET_VELOCITY,vel_data,None)
        
        rospy.sleep(5)
        self.boat_server.shutdown()
        
if __name__ == '__main__' :
    # try:
    boat_controller = ControlBoat()
    boat_controller.control_boat()
    # except Exception as e:
    #     print(e)
    #     rospy.loginfo('Boat control node terminated')

        
