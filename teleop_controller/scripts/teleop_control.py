#!/usr/bin/python3

import sys

# Velocity Controller
import time

import rospy

# Automatic Serial Detection
import serial
import serial.tools
import serial.tools.list_ports
from geometry_msgs.msg import Twist

import compy.command as cmd
import compy.conn as conn

Stm32_Con = None
pub = None
LRPM, RRPM = 0, 0
LinearVC, AngularVC = None, None


class VelocityController:
    def __init__(self, Max_Accelration=0.3):
        self.Max_Accelration = Max_Accelration
        self.LastSet = 0
        self.LastVel = 0
        self.LastTime = time.perf_counter()

    def Command(self, Velocity):
        self.LastSet = Velocity

    def Control(self):
        delta_t = time.perf_counter() - self.LastTime
        maxSpeed = self.Max_Accelration * delta_t
        if abs(self.LastSet - self.LastVel) > maxSpeed:
            # self.LastVel = self.LastVel + maxSpeed if (self.LastSet > 0 or (self.LastVel < 0 and self.LastSet == 0)) else self.LastVel - maxSpeed
            self.LastVel = (
                self.LastVel + maxSpeed
                if self.LastSet > self.LastVel
                else self.LastVel - maxSpeed
            )
            self.LastTime = time.perf_counter()
            return self.LastVel
        else:
            self.LastVel = self.LastSet
            self.LastTime = time.perf_counter()
            return self.LastSet


# Finding STM32 on all Ports
def FindStm32():
    print(
        f"\033[32mSearching for STM32 ...\033[0m"
        + "in "
        + str(len(serial.tools.list_ports.comports()))
    )
    for port in serial.tools.list_ports.comports():
        try:
            print("Trying... to Connect on " + port.name)
            con1 = conn.SerialConnection(port.device, timeout=2)
            con1.send(cmd.Echo(bytes("echo", encoding="utf-8")))
            data = con1.receive().bytes
            if data == b"echo":
                print(
                    "\033[32m" + "STM32 found on " + "\033[35m" + port.name + "\033[0m"
                )
                return con1
        except Exception as e:
            print(f"Except: {e}")
            print("STM32 is Not Connected on " + port.name)
    print("No Ports are Available", serial.tools.list_ports.comports())
    return None


# Connection to STM32 and Assign to Global Variable
def ConnectStm32():
    global Stm32_Con
    Stm32_Con = FindStm32()
    if Stm32_Con is None:
        print(f"\033[31mSTM32 Not Connected to Computer\033[0m")
        sys.exit()


# Diff Drive Module
def DiffDrive(VL, WZ):
    # Constants
    L = 0.77  # Distance between the wheels in m
    R = 0.1778  # Radius of the wheel in m

    # m/s
    LRPM = VL - ((WZ * L) / 2)
    RRPM = VL + ((WZ * L) / 2)

    # RPM
    LRPM = (LRPM * 60) / (2 * 3.14159 * R)
    RRPM = (RRPM * 60) / (2 * 3.14159 * R)

    return LRPM, RRPM


def CommandRPM(LRPM, RRPM):
    global Stm32_Con
    # print(LRPM,RRPM)
    if Stm32_Con is None:
        print("Cannot publish RPM values to STM32: No connection")
        return
    try:
        Stm32_Con.send(cmd.ChangeRpmR(RRPM))
        Stm32_Con.send(cmd.ChangeRpmL(LRPM))
    except Exception as e:
        print(f"Exception while sending command to STM32: {e}")


# Callback to /cmd_vel Subscriber
def callback(data):
    global LRPM, RRPM, LinearVC, AngularVC
    # rospy.loginfo(data.linear.x,data.angular.z)
    # print(DiffDrive(data.linear.x,data.angular.z))

    # LRPM,RRPM = DiffDrive(data.linear.x,data.angular.z)
    # CommandRPM(LRPM,RRPM)
    # print(LRPM,RRPM)
    LinearVC.Command(data.linear.x)
    AngularVC.Command(data.angular.z)


# Ros Publisher and init (Only for PID Tuning)


def ros_talker():
    global LRPM, RRPM, LinearVC, AngularVC
    rospy.init_node("Teleop Control", anonymous=True)
    print("Initialized Node.....")
    LinearVC, AngularVC = VelocityController(0.6), VelocityController(1.0)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    print("Subscribed to /cmd_vel")
    print("Spinning............")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        LRPM, RRPM = DiffDrive(LinearVC.Control(), AngularVC.Control())
        CommandRPM(LRPM, RRPM)
        print(LRPM, RRPM)
        rate.sleep()
    # print("Spinning............")
    # while not rospy.is_shutdown():
    # 	try:
    # 		rospy.spin()
    # 	except Exception as e:
    # 		print(f"Exception in talker: {e}")


if __name__ == "__main__":
    ConnectStm32()
    try:
        ros_talker()
    except Exception as e:
        print(e)
