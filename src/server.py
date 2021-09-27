#!/usr/bin/python3
# File containt server code for interacting with the phone application

import socket
import time

import random




class Commands:
    CMD_ACKNOWLEDGE = "OK"
    CMD_REGISTER = "HI"

    CMD_REGISTER_POSE_LISTENER = "RPL"
    CMD_SEND_POSE = "_P"
    CMD_SET_POSE = "SP"
    CMD_GET_POSE = "GP"

    CMD_REGISTER_SENSOR_LISTENER = "RSL"
    CMD_SEND_SENSOR = "_S"
    CMD_ACK_SENSORDATA = "ASD"
    
    CMD_REGISTER_VELOCITY_LISTENER = "RVL"
    CMD_SEND_VELOCITY = "_V"
    CMD_SET_VELOCITY = "SV"
    CMD_GET_VELOCITY = "GV"
    
    CMD_REGISTER_WAYPOINT_LISTENER = "RWL"
    CMD_SEND_WAYPOINT = "_W"
    CMD_START_WAYPOINTS = "STW"
    CMD_STOP_WAYPOINTS = "SPW"
    CMD_GET_WAYPOINTS = "GW"
    CMD_GET_WAYPOINT_STATUS = "GWS"
    CMD_GET_WAYPOINTS_INDEX = "GWI"
    
    # For now it seems like the phone doesn't send crumb data
    CMD_REGISTER_CRUMB_LISTENER = "RCL"
    CMD_SEND_CRUMB = "_B"
    CMD_ACK_CRUMB = "AC"


class Ticket:
    def __init__(self,ticket_id,command,packet,callback,server):
        # Since a ticket is only generated with a non -1 ticket number
        # Always expect an ack, always try to timeout
        
        #setting up the timeout things
        self.last_reply = time.time()
        self.max_time = 0.5
        self.timeout_counter = 0

        self.server = server
        self.ticket_id = ticket_id
        self.command = command
        self.packet = packet
        self.callback = callback
        self.got_ack = False
        #only wait for data if there is a callback
        if (callback != None):
            self.got_data = False
        else :
            self.got_data = True
    
    def handle_ack(self):
        self.got_ack = True
        self.last_reply = time.time()
        return self.got_data

    def handle_data(self,data):
        self.got_data = True
        self.last_reply = time.time()
        if (self.callback != None):
            self.callback(data)
        return self.got_ack

    def is_timeout(self):
        #returns true if the timeout function should be called
        return (not self.got_ack) and (abs(time.time() - self.last_reply)>self.max_time)
    
    def timeout(self):
        if (self.is_timeout()):
            self.timeout_counter += 1
            if (self.timeout_counter >=5):
                return True
            self.last_reply = time.time()
            packet = self.packet
            self.server.send_packet(packet)        
            return False
        else :
            return False


# Handle Listeners (and reregistering them after timeout)
# By default, the phone app stops sending messages to listeners after 5 seconds
class Listener:
    def __init__(self,server,register_command):
        self.callbacks = []

        self.server = server
        self.register_command = register_command

        self.last_reply = time.time()
        self.timeout = 3
        self.timeout_counter = 0


    def register_listener(self,callback):
        if (callback != None):
            self.callbacks.append(callback)
    
    def unregister_all(self):
        self.callbacks = []
    
    def check_timeout(self):
        if (self.is_connected()) and (abs(time.time()-self.last_reply)>self.timeout):
            self.last_reply = time.time()
            self.server.register_listener(self.register_command,None)
        return 

    def is_connected(self):
        return len(self.callbacks) >0

    def handle_data(self,data):
        self.last_reply = time.time()
        self.timeout_counter = 0
        for i in range(len(self.callbacks)):
            callback = self.callbacks[i]
            if callback != None:
                callback(data)
        
    
class UDPServer:

    NO_TICKET = -1

    #TODO: handle listener timeout

    listener_commands = [
        Commands.CMD_SEND_CRUMB,
        Commands.CMD_SEND_POSE , 
        Commands.CMD_SEND_SENSOR  ,
        Commands.CMD_SEND_VELOCITY , 
        Commands.CMD_SEND_WAYPOINT  
    ]

    def __init__(self,server_port,dest_ip,dest_port):
        # ticket dictionary
        # key : int/ticket number
        # value : Ticket object
        self.tickets = {}
        # first ticket is a random value between 1000 and 10000
        self.ticket_counter = random.randint(1000,10000)

        # Listeners: callback function that gets called when an appropriate
        # packet arrives
        self.pose_listeners = Listener(self,Commands.CMD_REGISTER_POSE_LISTENER)
        self.sensor_listeners = Listener(self,Commands.CMD_REGISTER_SENSOR_LISTENER)
        self.crumb_listeners = Listener(self,Commands.CMD_REGISTER_CRUMB_LISTENER)
        self.velocity_listeners = Listener(self,Commands.CMD_REGISTER_VELOCITY_LISTENER)
        self.waypoint_listeners = Listener(self,Commands.CMD_REGISTER_WAYPOINT_LISTENER)

        self.log = open('log.txt','w')

        # Create the socket
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        # Bind the socket so we can recieve packets
        self.sock.bind(('',server_port))        
        # Set socket timeout (so it doesn't hang waiting for a packet)
        self.sock.settimeout(0.1)

        # Set up destination ip and port
        self.dest_ip = dest_ip
        self.dest_port = dest_port

    
    def send_packet(self,packet):
        if not self.log.closed:
            self.log.write('Sent: ')
            self.log.write(str(packet))
            self.log.write('\n')
        self.sock.sendto(packet,(self.dest_ip,self.dest_port))

    def send_ack(self,ticket_number):
        self.ack_listener(Commands.CMD_ACKNOWLEDGE)

    def ack_listener(self,command):
        ticket_bytes = self.ticket_to_bytes(-1)
        command_bytes = self.command_to_bytes(command)
        packet = ticket_bytes + command_bytes
        self.send_packet(packet)
    
    def ack_sensor_data(self,sensor_ack_code):
        ticket_bytes = self.ticket_to_bytes(-1)
        sensor_ack_bytes = self.ticket_to_bytes(sensor_ack_code)
        command_bytes = self.command_to_bytes(Commands.CMD_ACK_SENSORDATA)
        packet = ticket_bytes + command_bytes + sensor_ack_bytes
        self.send_packet(packet)
    
    # Used for sending commands (not registering listeners)
    def send_command(self,ticket_number,command,data,callback):
        ticket_bytes = self.ticket_to_bytes(ticket_number)
        command_bytes = self.command_to_bytes(command)
        packet = ticket_bytes+command_bytes+data
        if ticket_number != -1 :
            # expecting a reply
            ticket = Ticket(ticket_number,command,packet,callback,self)
            self.tickets[ticket_number] = ticket
        self.send_packet(packet)

    # Used for registering listeners
    def register_listener(self,command,callback):

        valid_commands = [
            Commands.CMD_REGISTER_CRUMB_LISTENER,
            Commands.CMD_REGISTER_POSE_LISTENER,
            Commands.CMD_REGISTER_SENSOR_LISTENER,
            Commands.CMD_REGISTER_VELOCITY_LISTENER,
            Commands.CMD_REGISTER_WAYPOINT_LISTENER,
        ]
        if (command not in valid_commands):
            return 

        # Add callback to list of callbacks for the appropriate topic
        # All callbacks in the list are called when data arrives
        if (command == valid_commands[0]):
            listener = self.crumb_listeners
        if (command == valid_commands[1]):
            listener = self.pose_listeners
        if (command == valid_commands[2]):
            listener = self.sensor_listeners
        if (command == valid_commands[3]):
            listener = self.velocity_listeners
        if (command == valid_commands[4]):
            listener = self.waypoint_listeners

        listener.register_listener(callback)

        # Ack for registering a listener uses ticket number, future packets use -1
        ticket_number = self.get_ticket()
        ticket_bytes = self.ticket_to_bytes(ticket_number)
        command_bytes = self.command_to_bytes(command)
        packet = ticket_bytes + command_bytes

        # Generate a ticket to ensure the register listener command was recieved
        ticket = Ticket(ticket_number,command,packet,None,self)
        self.tickets[ticket_number] = ticket
        
        self.send_packet(packet)

    
    def ticket_to_bytes(self,t):
        return int.to_bytes(t,8,'big',signed=True)
    
    def ticket_from_bytes(self,b):
        return (int.from_bytes(b[:8],'big',signed=True) , b[8:])

    def command_to_bytes(self,command):
        l = int.to_bytes(len(command),2,'big',signed=True)
        c = bytes(command,'utf')
        return l+c

    def command_from_bytes(self,b):
        length = int.from_bytes(b[:2],'big',signed=True)
        b = b[2:]
        res = str(b[:length],'utf')
        return (res , b[length:])
    
    def get_ticket(self):
        res = self.ticket_counter
        self.ticket_counter += 1
        return res

    def handle_listeners(self,command,data):
        l = []
        if (command == self.listener_commands[0]):
            l = self.crumb_listeners
            # ack_command = Commands.CMD_ACK_CRUMB
        if (command == self.listener_commands[1]):
            l = self.pose_listeners
            # ack_command = Commands.CMD_ACK_
        if (command == self.listener_commands[2]):
            l = self.sensor_listeners
            # ack_command = Commands.CMD_ACK_CRUMB
        if (command == self.listener_commands[3]):
            l = self.velocity_listeners
            # ack_command = Commands.CMD_ACK_CRUMB
        if (command == self.listener_commands[4]):
            l = self.waypoint_listeners
            # ack_command = Commands.CMD_ACK_CRUMB
        
        l.handle_data(data)
        return

    def handle_timeouts(self):
        # Check if any ticket timed out
        keys = list(self.tickets.keys())
        for ticket_number in keys:
            ticket = self.tickets[ticket_number]
            # ticket.timeout checks if too much time
            # has passed since the last reply and 
            # resends a command automatically
            if (ticket.timeout()):
                # if ticket.timeout returns true,
                # the ticket should be removed
                self.tickets.pop(ticket_number)

        # for each listener, check if the replies timed out
        # if they did, the server will reregister the listener automatically
        self.pose_listeners.check_timeout()
        self.sensor_listeners.check_timeout()
        self.crumb_listeners.check_timeout()
        self.velocity_listeners.check_timeout()
        self.waypoint_listeners.check_timeout()
        
        return 

    def get_packet(self):
        try : 
            packet , addr = self.sock.recvfrom(512)
        except :
            return

        if (packet == ''):
            return
        if not self.log.closed :
            self.log.write('Recieved: ')
            self.log.write(str(packet))
            self.log.write('\n')

        ticket_number , data_no_ticket = self.ticket_from_bytes(packet)
        command, data = self.command_from_bytes(data_no_ticket)

        if (ticket_number != -1):
            # ticket, either an ack or data
            # if data, send ack to phone
            if (ticket_number in self.tickets):
                ticket = self.tickets[ticket_number]
                if command == Commands.CMD_ACKNOWLEDGE :
                    res = ticket.handle_ack()
                else:
                    res = ticket.handle_data(data)
                    self.send_ack(ticket_number)
                if (res):
                    self.tickets.pop(ticket_number)
        else :
            # no ticket
            # probably sensor
            if command in self.listener_commands :
                self.handle_listeners(command,data)

        return

    def shutdown(self):
        self.log.close()

