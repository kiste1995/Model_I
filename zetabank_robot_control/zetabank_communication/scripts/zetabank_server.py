#!/usr/bin/env python

'''
	Zeta robot protocol between main_board and controller_board.
	Server Program : running on controller_board
	(using Python 2.7)

	History:
		20181007 Geonhee Lee - Redefine the protocol
        20180730 Geonhee LEE - Reconstruction with the ROS platform
		20180715 kyuhsim - new protocol implementation
		20180529 kyuhsim - created.
'''
import rospy

import sys
import socket
from SocketServer import ThreadingTCPServer, StreamRequestHandler
#import logging

import json

from threading import Timer
import threading
import time

from zetabank_msgs.msg import CommInfo
from zetabank_msgs.srv import CmdInfo

### configurations that can be changed
gHost = 'localhost'
gPort = 9006
gRepeatTime = 1				# send status packet in every 2 sec., will be 1 sec.

### global variables
gSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 	# create socket (SOCK_STREAM means TCP socket) 

gConn = gSock				# init for only type matching. it will be changed after connection

gCache = {
	'status' : 'stopped'	# moving status : stopped, moving, suspended
}

gQueue = []					# list as move queue

gSendPacket = {
	'response' : '($echo)',
	'battery' : '50',
	'bumper' : 'off', 
	'sonar' : '1000100',
	'led' : ['on','off','off','off'], 
	'lidar' : '', 
	'position' : { 'x' : '0.10', 'y' : '0.20', 'theta' : '1.141592' },
	'velocity' : { 'v':'0', 'w':'0' }
	}

gRecvPacket = { 
	'command' : None,
	'speed' : None,
	'goal' : { 'x' : '0', 'y' : '0', 'theta' : '0' },
	'setled' : ['on','off','off','off']
	}

class RepeatTimer():
	def __init__(self, seconds, target):
		self._should_continue = False
		self.is_running = False
		self.seconds = seconds
		self.target = target
		self.thread = None

	def _handle_target(self):
		self.is_running = True
		self.target()
		self.is_running = False
		self._start_timer()

	def _start_timer(self):
		if self._should_continue: 			# Code could have been running when cancel was called.
			self.thread = Timer(self.seconds, self._handle_target)
			self.thread.start()

	def start(self):
		if not self._should_continue and not self.is_running:
			self._should_continue = True
			self._start_timer()
		else:
			print("* Timer already started or running, please wait if you're restarting.")

	def cancel(self):
		if self.thread is not None:
			self._should_continue = False 	# Just in case thread is running and cancel fails.
			self.thread.cancel()
		else:
			print("* Timer never started or failed to initialize.")


# get controller status and update gSendPacket and send_packet()

def repeated_processing():
	send_packet()
    
def send_packet():
	global gConn
	print ('>>>>> Sending Packet >>>>> %d' % threading.current_thread().ident)
	pkt = json.dumps( gSendPacket )				# serialize dict to json string
	gConn.send( pkt )

def subscribe_status():
	rospy.Subscriber("/zetabot/status", CommInfo, status_callback) # It receives the status data through topic

def status_callback(CommInfo):
	print ('<<<<< Callback ROS Topic <<<<<  %d' % threading.current_thread().ident)
	#rospy.loginfo(rospy.get_caller_id() + " %s", CommInfo.battery)
	collect_status(CommInfo)			# collect controller status to gSendPacket
	# Print the parameter of ROS
	#rospy.loginfo(CommInfo)

# robot's status came from publisher applies for packet	
def collect_status(CommInfo):
	gSendPacket['battery'] = CommInfo.battery			#get_battery_status()
	if gSendPacket['battery'] == '':
		gSendPacket['battery'] = None
	gSendPacket['bumper'] = CommInfo.bumper				#get_bumber_status()
	gSendPacket['sonar'] = CommInfo.sonar				#get_sonar_status()
	gSendPacket['led'] = CommInfo.led					#get_led_status()
	gSendPacket['lidar'] = CommInfo.lidar				#get_lidar_status()
	get_pose(CommInfo)									#get_position_status()
	get_velocity(CommInfo)								#get_velocity_status()
	gSendPacket['response'] = CommInfo.response			#{ 'status':'-' }

### Receive the Status from movement	
def get_pose(CommInfo):
	value = gSendPacket['position']		# dict
	if (value == '-'):					# string
		value = { 'x':'0', 'y':'0', 'theta':'0' }	# dict
	value['x'] = "%f" % CommInfo.pose_x
	value['y'] = "%f" % CommInfo.pose_y
	value['theta'] = "%f" % CommInfo.pose_theta
	return value	

def get_velocity(CommInfo):
	value = gSendPacket['velocity']		# dict
	if (value == '-'):					# string
		value = { 'v':'0', 'w':'0' }	# dict
	value['v'] = "%f" % CommInfo.trans_velocity
	value['w'] = "%f" % CommInfo.ang_velocity
	return value	

### <-- simulations
'''
	def get_battery_status():
		value = gSendPacket['battery']
		if (value == '-'):
			value = '100'
		i = int (value)				# string to integer
		i = (i - 1) % 100			# just decrement
		return "%s" % i				# return string of int
	def get_bumber_status():
		value = gSendPacket['bumper']
		if (value == '-'):
			value = 'off'
		elif (value == 'on'):		# just toggle
			value = 'off'
		elif (value == 'off'):
			value = 'on'
		return value
	def get_sonar_status():
		value = gSendPacket['sonar']
		if (value == '-'):
			value = '0'
		i = int (value, 16)			# sonar value is hexa (2 char)
		i = (i + 1) % 128			# just increment
		value = "%s" % hex(i)		# to hexa string
		value = value[2:]			# remove '0x'
		if (i < 0x10):
			return '0' + value		# if 1 char, add '0'
		else:
			return value
	def get_led_status():
		value = gSendPacket['led']
		if (value == '-'):
			value = '0'
		i = int (value)				# string to integer
		i = (i + 1) % 7				# just increment
		return "%s" % i
	def get_lidar_status():
		value = gSendPacket['lidar']
		if (value == '-'):
			value = '0'
		f = float(value)			# float
		f += 0.001					# just increment
		return "%f" % f				# return string of float
	def get_position_status():
		value = gSendPacket['position']		# dict
		if (value == '-'):					# string
			value = { 'x':'0', 'y':'0', 'theta':'0' }	# dict
		# process only 'x' value for simulation
		x = value['x']
		ix = int (x)
		ix = (ix + 1) % 1024
		value['x'] = "%d" % ix
		return value
	def get_velocity_status():
		value = gSendPacket['velocity']
		if (value == '-'):
			value = { 'v':'0', 'w':'0' }
		v = value['v']
		fv = float (v)
		fv = fv + 0.01				# just increment
		value['v'] = "%f" % fv
		w = value['w']
		fw = float (w)
		fw = fw + 0.001				# just increment
		value['w'] = "%f" % fw
		return value
	def get_obstacle_status():
		value = gSendPacket['obstacle']
		if (value == '-'):
			value = 'off'
		elif (value == 'on'):		# just toggle
			value = 'off'
		elif (value == 'off'):
			value = 'on'
		return value
	def get_pathchange_status():
		value = gSendPacket['pathchange']
		if (value == '-'):
			value = 'off'
		elif (value == 'on'):		# just toggle
			value = 'off'
		elif (value == 'off'):
			value = 'on'
		return value
	### --> simulations
'''
class RequestHandler( StreamRequestHandler ):
	def handle( self ):
		global gConn
		print ('- Connection from : '), (self.client_address)
		conn = self.request
		gConn = conn								# make send_packet() can use conn
		rt = RepeatTimer(gRepeatTime, repeated_processing)	# repeat every 2 sec., send gSendPacket. repeate time will be 1 sec. later.
		rt.start()

		while True:
			try:
				conn.settimeout(None)
				msg = conn.recv(1024)
						
			except Exception, e:
				#logger.error('Failed to upload to ftp: '+ str(e))
				print ('* conn.recv made exception. connection is broken.\n- Waiting another connection...')
				print e
				rt.cancel()
				conn.close()
				break

			if not msg:
				StopRepeatTimer()
				rt.cancel()
				conn.close()
				print ('! Disconnected from : ', self.client_address)
				break
			print ('<<<<<< [Recieve msg from Client] <<<<< \n'), (msg)
			process_message( conn, msg )			# conn.send( msg )		

def StopRepeatTimer():
	print ''			

# process message from client of TCP/IP communication
def process_message( conn, msg ):
	print ("TCP/IP process_message")		
	try:
		pkt = json.loads( msg )		# should return type of 'dict'
	except:
		print ("! json.loads fails")
		pkt = ""

	if ( type(pkt) == type({}) ):	# if packet type is dict
		update_gRecvPacket( pkt )	# copy first
		client_gRecvPacket()		# publish received packet with command through topic

	else:							# recved packet cannot be decoded as json string
		conn.send( '! Unknown Command' )

# update gRecvPacket using the json message from client of TCP/IP communication
def update_gRecvPacket( json_string ):
	gRecvPacket['command'] = json_string['command']
	gRecvPacket['speed'] = json_string['speed']
	gRecvPacket['setled'] = json_string['setled']
	gRecvPacket['goal'] = json_string['goal']


# deliver rgRecvPacket to the action manger node through ROS service
def client_gRecvPacket():
	rospy.wait_for_service('/zetabot/command')
	goal = gRecvPacket[ 'goal' ]		# dict

	print (gRecvPacket['setled'])
	
	for i in range(0,4):
		if gRecvPacket['setled'][0] == 'on':
			gLedStr = "1"
		else:
			gLedStr = "0"

		if gRecvPacket['setled'][1] == 'on':
			gLedStr += "1"
		else:
			gLedStr += "0"

		if gRecvPacket['setled'][2] == 'on':
			gLedStr += "1"
		else:
			gLedStr += "0"

		if gRecvPacket['setled'][3] == 'on':
			gLedStr += "1"
		else:
			gLedStr += "0"

	print (gLedStr)
	
	try:
		#Receive ros service call
		req_cmd_info = rospy.ServiceProxy('/zetabot/command', CmdInfo)
		response = req_cmd_info(gRecvPacket['command'], gRecvPacket[ 'speed' ],float(goal['x']), float(goal['y']),float(goal['theta']), gLedStr)
		#print response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	print ("\t<<<<<< command: "), (gRecvPacket['command'])
	print ("\t<<<<<< speed: "), (gRecvPacket[ 'speed' ])
	print ("\t<<<<<< setled: "), (gRecvPacket[ 'setled' ])
       

if __name__ == '__main__':

	rospy.init_node('zetabank_server')
	subscribe_status()
    # Load the parameter values from Xmlserver
	gPort = rospy.get_param('port')
	gHost = rospy.get_param('ip')

    # Start threading
	print "- Listening on ip, port ", (gHost), (gPort)

    # Declare the Class SocketServer.ThreadTCPServer
	server = ThreadingTCPServer((gHost, gPort), RequestHandler)


    # serve_forever(poll_interval=0.5)
    # Handle requests until an explicit shutdown() request. 
    # Poll for shutdown every poll_interval seconds. 
    # Ignores the timeout attribute. 
    # If you need to do periodic tasks, do them in another thread.
	server.serve_forever()


	thread = threading.Thread(target = rospy.spin())
	thread.start()
	
# End of File
        
