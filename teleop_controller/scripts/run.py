#!/usr/bin/env python

#import rospy
#from geometry_msgs.msg import Twist
import compy.conn as conn
import compy.command as cmd
# Automatic Serial Detection
import serial
import serial.tools
import serial.tools.list_ports
import sys
import random
import time
#Xbox Libraries
import signal
from xbox360controller import Xbox360Controller

Stm32_Con = None

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

def on_axis_moved(axis):
	if(axis.name == 'axis_l'):
		print(round(axis.y,2)*-1)
	else:
		print(round(axis.x,2))


# def callback(data):
# 	#rospy.loginfo(data.linear.x,data.angular.z)
# 	#print(DiffDrive(data.linear.x,data.angular.z))
# 	LRPM,RRPM = DiffDrive(data.linear.x,data.angular.z)
# 	print(LRPM,RRPM)
# 	Stm32_Con.send(cmd.ChangeRpmR(RRPM))
# 	Stm32_Con.send(cmd.ChangeRpmL(LRPM))

#def listener():
#	rospy.init_node('STM32_Interfacer', anonymous=True)
#	rospy.Subscriber("/cmd_vel", Twist, callback)
#	rospy.spin()
def xbox_listener(Stm32_Con):
	try:
		with Xbox360Controller(0, axis_threshold=0.2) as controller:
 			# Button A events
			#controller.button_a.when_pressed = on_button_pressed
			#controller.button_a.when_released = on_button_released
			# Left and right axis move event
			#controller.axis_l.when_moved = on_axis_moved
			#controller.axis_r.when_moved = on_axis_moved
			#signal.pause()
			while True:
				if(controller.button_b.is_pressed == True):
					print(DiffDrive(0,0))
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
				print(LRPM,RRPM)

				Stm32_Con.send(cmd.ChangeRpmR(LRPM))
				Stm32_Con.send(cmd.ChangeRpmL(RRPM))
				
	except KeyboardInterrupt:
		Stm32_Con.send(cmd.ChangeRpmR(0))
		Stm32_Con.send(cmd.ChangeRpmL(0))
		time.sleep(1)
if __name__ == '__main__':
	#listener()
	Stm32_Con = FindStm32()
	if(Stm32_Con == None):
		print(f'\033[31mSTM32 Not Connected to Computer\033[0m')
	
	xbox_listener(Stm32_Con)
	#while True:
	#	a = random.random()*60
	#	b = random.random()*60
	#	Stm32_Con.send(cmd.ChangeRpmR(a))
	#	Stm32_Con.send(cmd.ChangeRpmL(b))
	#	print(a,b)
	#	time.sleep(0.0001)


	#Stm32_Con.send(cmd.ChangeRpmR(30))
	#Stm32_Con.send(cmd.ChangeRpmL(40))

	#listener()
	
    # while True:
    #     data = input(">> ")

    #     if data.startswith("r"):
    #         rpm = float(data.lstrip("r "))
    #         conn1.send(cmd.ChangeRpmR(rpm))
    #     elif data.startswith("l"):
    #         rpm = float(data.lstrip("l "))
    #         conn1.send(cmd.ChangeRpmL(rpm))
    #     elif data.startswith("e"):
    #         conn1.send(cmd.Echo(bytes("echo", encoding="utf-8")))
    #         print("Received: ", conn1.receive().bytes)