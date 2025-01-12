#!/usr/bin/env python
import time

import sys

import compy.conn as conn
import compy.command as cmd
# Automatic Serial Detection
import serial
import serial.tools
import serial.tools.list_ports
import sys
import time
#Xbox Libraries
from xbox360controller import Xbox360Controller

Stm32_Con = None
LRPM,RRPM = 0,0

def FindStm32():
	print(f'\033[32mSearching for STM32 ...\033[0m')
	for port in serial.tools.list_ports.comports():
		try:
			print("Trying... to Connect on "+port.name)
			con1 = conn.SerialConnection(port.device, timeout = 2)
			con1.send(cmd.Echo(bytes("echo", encoding="utf-8")))
			data = con1.receive().bytes
			if data == b'echo':
				print('\033[32m'+"STM32 found on "+'\033[35m'+port.name+'\033[0m')
				return con1
		except Exception as e:
			print(f"Except: {e}")
			print("STM32 is Not Connected on "+port.name)
	return None

def DiffDrive(VL,WZ):
	# Constants
	L = 0.55 # Distance between the wheels
	R = 0.1778 # Radius of the wheel
	
	# m/s
	LRPM = (VL - ((WZ*L)/2))
	RRPM = (VL + ((WZ*L)/2))

	#RPM
	LRPM = (LRPM*60)/(2*3.14159*R)
	RRPM = (RRPM*60)/(2*3.14159*R)

	return LRPM,RRPM

def xbox_listener():
	global LRPM,RRPM,Stm32_Con
	try:
		with Xbox360Controller(0, axis_threshold=0.2) as controller:
			while True:
				if(controller.button_b.is_pressed == True):
					#print(DiffDrive(0,0))
					Stm32_Con.send(cmd.ChangeRpmR(0))
					Stm32_Con.send(cmd.ChangeRpmL(0))
					continue
				throttle_y = round(controller.axis_l.y,2)
				throttle_x = round(controller.axis_r.x,2)
				LRPM,RRPM = DiffDrive(-1.6*throttle_y,1.6*throttle_x)
				# To Prevent from Reciving Noice from Xbox Controller
				if (LRPM < 5 and LRPM > 0) or (LRPM > -5 and LRPM < 0):
					LRPM = 0
				if (RRPM < 5 and RRPM > 0) or (RRPM > -5 and RRPM < 0):
					RRPM = 0
				#print(LRPM,RRPM)

				Stm32_Con.send(cmd.ChangeRpmR(LRPM))
				Stm32_Con.send(cmd.ChangeRpmL(RRPM))
				
	except KeyboardInterrupt:
		Stm32_Con.send(cmd.ChangeRpmR(0))
		Stm32_Con.send(cmd.ChangeRpmL(0))
		time.sleep(1)

def ConnectStm32():
	global Stm32_Con
	Stm32_Con = FindStm32()
	if(Stm32_Con == None):
		print(f'\033[31mSTM32 Not Connected to Computer\033[0m')
	
if __name__ == '__main__':
	ConnectStm32()
	xbox_listener()