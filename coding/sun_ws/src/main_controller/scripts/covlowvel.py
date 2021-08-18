#!/usr/bin/python3

import serial
from json import loads

__ser = None
__inited = False
__hex = '0123456789abcdef'

def init(sercode, sudopass=''):
	global __ser, __inited
	try:
		__ser = serial.Serial(sercode, baudrate=921600)
	except Exception:
		from os import system
		system('echo %s | sudo -S chmod 777 %s'%(sudopass, sercode))
		__ser = serial.Serial(sercode, baudrate=921600)
		del system
	__inited = True

def setServo(servo1, servo2, servo3):
	if not __inited:
		raise Exception('Call init first!')

	if servo1==60 or servo1==62 or servo1==10 or servo1==13:
		servo1 += 1
	elif servo1==0:
		servo1 = 1

	if servo2==60 or servo2==62 or servo2==10 or servo2==13:
		servo2 += 1
	elif servo2==0:
		servo2 = 1

	if servo3==60 or servo3==62 or servo3==10 or servo3==13:
		servo3 += 1
	elif servo3==0:
		servo3 = 1

	__ser.write(eval('b"<V\\x%s\\x%s\\x%s>"'%(
		__hex[servo1//16]+__hex[servo1%16],
		__hex[servo2//16]+__hex[servo2%16],
		__hex[servo3//16]+__hex[servo3%16],
	)))
	return loads(__ser.readline().decode())

def getButton():
	if not __inited:
		raise Exception('Call init first!')
		
	__ser.write(b'<S>')
	return loads(__ser.readline().decode())
	