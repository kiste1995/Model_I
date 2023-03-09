'''
	Zeta robot protocol between main_board and controller_board.
	Server Program : running on controller_board
	(using Python 2.7)

	History:
		20181007 Geonhee Lee - Redefine the protocol
		20180715 kyuhsim - new protocol implementation
		20180529 kyuhsim - created.
'''


import sys
import socket
from SocketServer import ThreadingTCPServer, StreamRequestHandler

import json

from threading import Timer
import time


### configurations that can be changed
gPort = 9006
gRepeatTime = 2				# send status packet in every 2 sec., will be 1 sec.

### global variables
gSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 	# create socket (SOCK_STREAM means TCP socket) 

gConn = gSock				# init for only type matching. it will be changed after connection

gCache = {
	'status' : 'stopped'	# moving status : stopped, moving, suspended
}

gQueue = []					# list as move queue

gSendPacket = {
	'response' : 'None',
	'battery' : '50',
	'bumper' : 'off', 
	'sonar' : '3f',
	'led' : '0', 
	'lidar' : '0.05', 
	'position' : { 'x' : '100', 'y' : '200', 'theta' : '3.141592' },
	'velocity' : 'None',
	}

gRecvPacket = { 
	'command' : 'None',
	'speed' : 'None',
	'goal' : { 'x' : '0', 'y' : '0', 'theta' : 'None' },
	'setled' : 'None'
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

# process received packet
def process_gRecvPacket():
	print ("\t\t<-- command: "), (gRecvPacket['command'])
	print ("\t\t<-- speed: "), (gRecvPacket[ 'speed' ])
	print ("\t\t<-- goal: "), (gRecvPacket[ 'goal' ])
	print ("\t\t<-- setled: "), (gRecvPacket[ 'setled' ])

def update_gRecvPacket( json_string ):
	gRecvPacket['command'] = json_string['command']
	gRecvPacket['speed'] = json_string['speed']
	gRecvPacket['goal'] = json_string['goal']
	gRecvPacket['setled'] = json_string['setled']

# process message from client
def process_message( conn, msg ):
	try:
		pkt = json.loads( msg )		# should return type of 'dict'
	except:
		print ("! json.loads fails")
		pkt = ""

	if ( type(pkt) == type({}) ):	# if packet type is dict
		update_gRecvPacket( pkt )	# copy first
		process_gRecvPacket()		# process received packet
	else:							# recved packet cannot be decoded as json string
		conn.send( '! Unknown Command' )

def send_packet():
	global gConn
	print ('\t---> Sending Packet')
	pkt = json.dumps( gSendPacket )				# serialize dict to json string
	gConn.send( pkt )


# get controller status and update gSendPacket and send_packet()
### <-- simulations
def get_battery_status():
	value = gSendPacket['battery']
	if (value == 'None'):
		value = '100'
	i = int (value)				# string to integer
	i = (i - 1) % 100			# just decrement
	return "%s" % i				# return string of int
def get_bumber_status():
	value = gSendPacket['bumper']
	if (value == 'None'):
		value = 'off'
	elif (value == 'on'):		# just toggle
		value = 'off'
	elif (value == 'off'):
		value = 'on'
	return value
def get_sonar_status():
	value = gSendPacket['sonar']
	if (value == 'None'):
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
	if (value == 'None'):
		value = '0'
	i = int (value)				# string to integer
	i = (i + 1) % 7				# just increment
	return "%s" % i
def get_lidar_status():
	value = gSendPacket['lidar']
	if (value == 'None'):
		value = '0'
	f = float(value)			# float
	f += 0.001					# just increment
	return "%f" % f				# return string of float
def get_position_status():
	value = gSendPacket['position']		# dict
	if (value == 'None'):					# string
		value = { 'x':'0', 'y':'0', 'theta':'0' }	# dict
	# process only 'x' value for simulation
	x = value['x']
	ix = int (x)
	ix = (ix + 1) % 1024
	value['x'] = "%d" % ix
	return value
def get_velocity_status():
	value = gSendPacket['velocity']
	if (value == 'None'):
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
### --> simulations

def collect_status():
	gSendPacket['battery'] = get_battery_status()
	gSendPacket['bumper'] = get_bumber_status()
	gSendPacket['sonar'] = get_sonar_status()
	gSendPacket['led'] = get_led_status()
	gSendPacket['lidar'] = get_lidar_status()
	gSendPacket['position'] = get_position_status()
	gSendPacket['velocity'] = get_velocity_status()
	gSendPacket['response'] = { 'status':'None' }
	
def repeated_processing():
	collect_status()			# collect controller status to gSendPacket
	send_packet()

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
				msg = conn.recv(1024)
			except:
				print ('* conn.recv made exception. connection is broken.\n- Waiting another connection...')
				rt.cancel()
				conn.close()
				break

			if not msg:
				StopRepeatTimer()
				rt.cancel()
				conn.close()
				print ('! Disconnected from : ', self.client_address)
				break
			print ('\t\t<--- [Client] '), (msg)
			process_message( conn, msg )			# conn.send( msg )

if __name__ == '__main__':

	if (len(sys.argv) != 2 ):
		print ("- Usage : python {} <port>".format( sys.argv[0] ))
		print ("* Exiting...")
	else:
		gPort = int( sys.argv[1] )			# to integer
		server = ThreadingTCPServer(('', gPort), RequestHandler)
		print ('- Listening on port '), (gPort)
		server.serve_forever()


# End of File
