#!/usr/bin/python3

import random

# Xbox Libraries
import signal
import sys
import threading
import time

import rospy

# Automatic Serial Detection
import serial
import serial.tools
import serial.tools.list_ports
from geometry_msgs.msg import Pose, Twist
from xbox360controller import Xbox360Controller

import compy.command as cmd
import compy.conn as conn

Stm32_Con = None
pub = None
LRPM, RRPM = 0, 0
RLRPM, RRRPM = 0, 0


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
    L = 0.55  # Distance between the wheels
    R = 0.1778  # Radius of the wheel

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


# Xbox Values Reader
def xbox_listener():
    global LRPM, RRPM, Stm32_Con
    try:
        with Xbox360Controller(0, axis_threshold=0.2) as controller:
            while True:
                if controller.button_b.is_pressed == True:
                    print(DiffDrive(0, 0))
                    CommandRPM(0, 0)
                    continue
                throttle_y = round(controller.axis_l.y, 2)
                throttle_x = round(controller.axis_r.x, 2)
                LRPM, RRPM = DiffDrive(-1.6 * throttle_y, 1.6 * throttle_x)
                # To Prevent from Reciving Noice from Xbox Controller
                if abs(LRPM) < 5 or abs(RRPM) < 5:
                    LRPM = 0
                    RRPM = 0
                print(LRPM, RRPM)
                CommandRPM(LRPM, RRPM)
                # Remove this if Not PID Tuning
                # Publish_Recv()
                time.sleep(0.01)
                # Stm32_Con.send(cmd.ChangeRpmR(LRPM))
                # Stm32_Con.send(cmd.ChangeRpmL(RRPM))

    except KeyboardInterrupt:
        CommandRPM(0, 0)
        print("Connection Closed")


# Callback to /cmd_vel Subscriber
def callback(data):
    global LRPM, RRPM
    # rospy.loginfo(data.linear.x,data.angular.z)
    # print(DiffDrive(data.linear.x,data.angular.z))
    LRPM, RRPM = DiffDrive(data.linear.x, data.angular.z)
    CommandRPM(LRPM, RRPM)
    # print(LRPM,RRPM)
    # Stm32_Con.send(cmd.ChangeRpmR(RRPM))
    # Stm32_Con.send(cmd.ChangeRpmL(LRPM))


# Ros Publisher and init (Only for PID Tuning)


def talker():
    global LRPM, RRPM, RLRPM, RRRPM
    pub = rospy.Publisher("Recv", Pose, queue_size=10)
    rospy.init_node("DkPy", anonymous=True)
    rate = rospy.Rate(100)  # 100hz
    rospy.Subscriber("/cmd_vel", Twist, callback)
    while not rospy.is_shutdown():
        try:
            poe = Pose()
            # Left - Position
            poe.position.x = LRPM
            poe.position.y = RLRPM
            # Right - Orientation
            poe.orientation.x = RRPM
            poe.orientation.y = RRRPM
            # Publish to /Recv Topic
            pub.publish(poe)
        except Exception as e:
            print(f"Exception in talker: {e}")


def background_task_rec():
    global RRRPM, RLRPM, Stm32_Con
    while True:
        try:
            data = Stm32_Con.serial.read_until(b"\n")
            # print(data)
            RLRPM, RRRPM = map(float, data.decode("utf-8").split(","))
            RRRPM = RRRPM * (60 / 23283)
            RLRPM = RLRPM * (60 / 23283)
            # print(RRRPM,RLRPM)
        except Exception as e:
            print(f"Exception in background_task_rec: {e}")


def Send_Kp():
    global Stm32_Con
    while True:
        try:
            ini = input(">>")
            if ini.startswith("ki"):
                ki = float(ini.lstrip("ki "))
                Stm32_Con.send(cmd.ChangeKi(ki))
            elif ini.startswith("kp"):
                kp = float(ini.lstrip("kp "))
                Stm32_Con.send(cmd.ChangeKp(kp))
            elif ini.startswith("exit"):
                sys.exit()
        except ValueError:
            print("Invalid input format. Use 'ki <value>' or 'kp <value>'")
        except Exception as e:
            print(f"Exception in Send_Kp: {e}")


def Pid_control():
    global pub
    pub = rospy.Publisher("Recv", Pose, queue_size=10)
    rospy.init_node("DkPy", anonymous=True)


def Publish_Recv():
    global LRPM, RRPM, RLRPM, RRRPM, pub
    try:
        poe = Pose()
        # Left - Position
        poe.position.x = LRPM
        poe.position.y = RLRPM
        # Right - Orientation
        poe.orientation.x = RRPM
        poe.orientation.y = RRRPM
        # Publish to /Recv Topic
        pub.publish(poe)
    except Exception as e:
        print(f"Exception in talker: {e}")


if __name__ == "__main__":
    # Xbox Control
    ConnectStm32()
    # Pid_control()
    # threading.Thread(target=Send_Kp,daemon=True).start()
    # threading.Thread(target=background_task_rec,daemon=True).start()
    # xbox_listener()

    # threading.Thread(target=xbox_listener,daemon=True).start()
    # threading.Thread(target=Send_Kp,daemon=True).start()
    # threading.Thread(target=background_task_rec,daemon=True).start()
    # xbox_listener()

    # Connect to STM32
    # ConnectStm32()
    # threading.Thread(target=xbox_listener,daemon=True).start()
    # threading.Thread(target=Send_Kp,daemon=True).start()
    # xbox_listener()
    try:
        talker()
    except Exception as e:
        print(e)

