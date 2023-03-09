#!/usr/bin/env python
'''
	Zeta robot protocol between main_board and controller_board.
	Client Program : running on main_board
	(using Python 2.7) 
	History:
		20181007 Geonhee Lee - Redefine the protocol
		20180813 Geonhee Lee - Change velocity type
		20180727 kyuhsim - UI version with Tkinter
		20180715 kyuhsim - new protocol implementation
		20180529 kyuhsim - created.
'''

import sys
import socket
from threading import Thread

import json

from Tkinter import *

### configurations that can be changed
gHost = "192.168.0.55"	# "localhost"			# "192.168.0.123"
gPort = 9006


# for UI
class App:
	def __init__(self, master):

		frame = Frame(master)
		frame.configure(width=400, height=300)
		frame.place(x=0, y=0, width=400, height=300)
		frame.pack()

		
		self.Buttons = [None] * 9

		self.Buttons[0] = Button(frame, text="teleop_x", command=self.teleop_x)
		self.Buttons[0].pack(anchor=NW)
		self.Buttons[1] = Button(frame, text="teleop_theta", command=self.teleop_theta)
		self.Buttons[1].pack(anchor=NW)
		self.Buttons[2] = Button(frame, text="stop", command=self.do_stop)
		self.Buttons[2].pack(anchor=NW)
		self.Buttons[3] = Button(frame, text="slow", command=self.slow)
		self.Buttons[3].pack(anchor=SE)
		self.Buttons[4] = Button(frame, text="normal", command=self.normal)
		self.Buttons[4].pack(anchor=SE)
		self.Buttons[5] = Button(frame, text="led", command=self.led)
		self.Buttons[5].pack(anchor=SE)
		self.Buttons[6] = Button(frame, text="moveto_left_top", command=self.moveto_left_top)
		self.Buttons[6].pack(anchor=NW)
		self.Buttons[7] = Button(frame, text="moveto_bottom", command=self.moveto_bottom)
		self.Buttons[7].pack(anchor=SE)
		self.Buttons[8] = Button(frame, text="moveto_right_top", command=self.moveto_right_top)
		self.Buttons[8].pack(anchor=SE)

		'''
		self.Buttons[0].place(x=10,  y=10, width=100, height=30)
		self.Buttons[1].place(x=10,  y=40, width=100, height=30)
		self.Buttons[2].place(x=10,  y=70, width=100, height=30)
		self.Buttons[3].place(x=10, y=100, width=100, height=30)
		self.Buttons[4].place(x=10, y=130, width=100, height=30)
		self.Buttons[5].place(x=10, y=160, width=100, height=30)
		self.Buttons[6].place(x=10, y=190, width=100, height=30)
		self.Buttons[7].place(x=10, y=220, width=100, height=30)
		self.Buttons[8].place(x=10, y=250, width=100, height=30)
		'''
		
	def teleop_x(self):
		print ("    --> sending teleop_x command")
		gSendPacket['command'] = 'teleop'
		gSendPacket['goal'] = { 'x' : '0.1', 'y' : '0', 'theta' : '0'}
		gSendPacket['setled'] = ['off','off','off','off']
		send_packet()
	def teleop_theta(self):
		print ("    --> sending teleop_theta command")
		gSendPacket['command'] = 'teleop'
		gSendPacket['goal'] = { 'x' : '0', 'y' : '0', 'theta' : '0.1'}
		gSendPacket['setled'] = ['off','off','off','off']
		send_packet()
	def do_stop(self):
		print ("    --> sending stop command")
		gSendPacket['command'] = 'stop'
		gSendPacket['goal'] = { 'x' : '0', 'y' : '0', 'theta' : '0'}
		gSendPacket['setled'] = ['off','off','off','off']
		send_packet()
	def moveto_left_top(self):
		print ("    --> sending moveto command")
		gSendPacket['command'] = 'moveto'
		gSendPacket['goal'] = {'x':'500', 'y':'730', 'theta':'0.85'}
		gSendPacket['setled'] = ['on','off','off','off']
		send_packet()
	def slow(self):
		print ("    --> sending slow command")
		gSendPacket['command'] = 'moveto'
		gSendPacket['goal'] = {'x':'1000', 'y':'680', 'theta':'0.85'}
		gSendPacket['speed'] = 'slow'
		send_packet()
	def led(self):
		print ("    --> sending led command")	# '0' ~ '6' : 0 : none, 1: normal, 2: obstacle, 3: batt<=10%, 4:batt<=20%, 5:batt<=60%, 6:speech
		gSendPacket['command'] = 'led'
		gSendPacket['goal'] = {'x':'1500', 'y':'735', 'theta':'0.85'}
		gSendPacket['speed'] = 'fast'
		gSendPacket['setled'] = ['on','on','off','off']
		send_packet()
	def normal(self):
		print ("    --> sending normal command")
		gSendPacket['command'] = 'initialize'
		gSendPacket['goal'] = { 'x' : '800', 'y' : '730', 'theta' : '0'}
		gSendPacket['speed'] = 'normal'
		gSendPacket['setled'] = ['on','off','on','off']
		send_packet()
	def moveto_bottom(self):
		print ("    --> sending emergencymove command")
		gSendPacket['command'] = 'moveto'
		gSendPacket['goal'] = {'x':'800', 'y':'1500', 'theta':'3.141592'}
		gSendPacket['setled'] = ['on','off','off','on']
		send_packet()
	def moveto_right_top(self):
		print ("    --> sending nextmove command")
		gSendPacket['command'] = 'moveto'
		gSendPacket['goal'] = {'x':'1100', 'y':'600', 'theta':'1.070796'}
		gSendPacket['setled'] = ['on','on','on','on']
		send_packet()


### global variables
gSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 	# create socket (SOCK_STREAM means TCP socket) 
gCache = { 
		'ServerIP' : 'localhost', 
		'ServerPort' : 9006,
		'ServerStatus' : 'Unknown', 
		'ClientStatus' : 'Begin',
		'MsgNum' : 0,
	}

gSendPacket = { 
	'command' : 'None',
	'speed' : 'normal',
	'goal' : { 'x' : '50', 'y' : '100', 'theta' : '3.141592' },
	'setled' : ['on','off','off','off'], 
	}

gRecvPacket = {
	#	'response' : { 'status' : '-', 'theta' : '-' },
	'response' : 'None',
	'battery' : 'None',
	'bumper' : 'None', 
	'sonar' : 'None',
	'led' : ['on','off','off','off'], 
	'lidar' : 'None', 
	'position' : { 'x' : '0', 'y' : '0', 'theta' : 'None' },
	'velocity' : { 'v' : '0', 'w' : '0'}
	}

# process received packet
def process_gRecvPacket():
	'''
	print ("\t\t<-- response: "), (gRecvPacket['response'])
	print ("\t\t<-- battery: "), (gRecvPacket[ 'battery' ])
	print ("\t\t<-- bumper: "), (gRecvPacket[ 'bumper' ])
	print ("\t\t<-- sonar: "), (gRecvPacket[ 'sonar' ])
	print ("\t\t<-- led: "), (gRecvPacket[ 'led' ])
	print ("\t\t<-- lidar: "), (gRecvPacket[ 'lidar' ])
	print ("\t\t<-- position: "), (gRecvPacket[ 'position' ])
	print ("\t\t<-- velocity: "), (gRecvPacket[ 'velocity' ])
	'''
	#if (gRecvPacket['response'] != '-'):
	#	print ("\t\t<-- response.status: "), (gRecvPacket['response']['status'])
	#  
	#if (gRecvPacket['response']['status'] == 'obstaclealert'):
	#	print ("\t\t<-- response.theta: "), (gRecvPacket['response']['theta'])


def update_gRecvPacket( json_string ):
	gRecvPacket['response'] = json_string['response']
	gRecvPacket['battery'] = json_string['battery']
	gRecvPacket['bumper'] = json_string['bumper']
	gRecvPacket['sonar'] = json_string['sonar']
	gRecvPacket['led'] = json_string['led']
	gRecvPacket['lidar'] = json_string['lidar']
	gRecvPacket['position'] = json_string['position']
	gRecvPacket['velocity'] = json_string['velocity']

def recv_msg(sock):
	MsgRecv = ''
	while True:
		try:
			MsgRecv = gSock.recv(1024)
			if not MsgRecv:
				break
			print ("\t\t<--- [Server] {}".format( MsgRecv ))
			pkt = json.loads( MsgRecv )				# should return type of 'dict'
			if ( type(pkt) == type({}) ):			# if packet type is dict
				update_gRecvPacket( pkt )			# copy first
				process_gRecvPacket()
			else:									# recved packet cannot be decoded as json string
				print ("* json.loads fails")
		except:
			print ("* recv() exception")


def connect_socket(host, port):
	gSock.connect((host, port))						# connect()
	print ("- Connected to " + host + ", " + str(port))

	gThrd = Thread( target=recv_msg, args=(gSock,))		# thread to process received packets
	gThrd.daemon = True
	gThrd.start()

	gCache[ 'ServerStatus' ] = 'Connected'


def send_packet():
	global gSock
	print ('\t---> Sending Packet')
	pkt = json.dumps( gSendPacket )					# serialize dict to json string
	gSock.sendall( pkt )



if __name__ == '__main__':
	if ( len(sys.argv) != 3 ):
		print ("- Usage : python {} <target_host_ip> <port>".format( sys.argv[0] ))
		print ("* Exiting...")
	else:
		gHost = sys.argv[1]
		gPort = int( sys.argv[2] )					# to integer
		print ("- Protocol Client : \n")

		connect_socket(gHost, gPort)				# connect to server

		root = Tk()
		root.configure(width=400, height=300)
		app = App(root)
		root.mainloop()

		gSock.close()
		print ("- Program exiting normally")

# End of File