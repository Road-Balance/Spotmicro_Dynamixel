from os import system, name 
import sys, os
sys.path.append("..")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import matplotlib.animation as animation
import numpy as np
import time
import math
import datetime as dt
import keyboard
import random
import board
import busio
import adafruit_bno055


import Kinematics.kinematics as kn
import spotmicroai
from SD_servo_controller_test import Dynamixel_Controllers

from multiprocessing import Process
from Common.multiprocess_kb import KeyInterrupt
from Kinematics.kinematicMotion import KinematicMotion, TrottingGait

from dynamixel_sdk import *

rtime=time.time()

# BNO055 ( IMU )
i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)
sensor = adafruit_bno055.BNO055_I2C(i2c_bus0)

# Dynamixel address
ADDR_MX_TORQUE_ENABLE      = 24               
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

PROTOCOL_VERSION            = 1.0

BAUDRATE                    = 1000000            
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                
TORQUE_DISABLE              = 0

# setting port and protocol version
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


DXL_controller = Dynamixel_Controllers()

# setting the DXL_Motor
DXL_controller.DynamixelSetting()



def reset():
    global rtime
    rtime=time.time()    

robot=spotmicroai.Robot(False,False,reset)
# controller = servo_controller.Controllers()

# TODO: Needs refactoring
speed1=240
speed2=170
speed3=300

speed1=322
speed2=237
speed3=436

spurWidth=robot.W/2+50
stepLength=0
stepHeight=72

# Initial End point X Value for Front legs 
iXf=120

walk=False

def resetPose():
    
    # joystick?
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz, joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

# define our clear function 
def consoleClear(): 
  
    # for windows 
    if name == 'nt': 
        _ = system('cls') 
  
    # for mac and linux(here, os.name is 'posix') 
    else: 
        _ = system('clear') 

Lp = np.array([[iXf, -170, spurWidth, 1], [iXf, -170, -spurWidth, 1],
[-120, -170, spurWidth, 1], [-120, -170, -spurWidth, 1]])

# Lp = np.array([[120, -140, 90, 1],[120, -140, -90, 1],[-120, -140, 90, 1],[-120, -140, -90, 1]])

motion=KinematicMotion(Lp) 
resetPose()

trotting=TrottingGait()

def main(id, command_status):
    jointAngles = []
    while True:
        xr = 0.0
        yr = 0.0

        # Reset when robot pose become strange
        # robot.resetBody()
    
        ir=xr/(math.pi/180)
        
        d=time.time()-rtime

        # robot height
        height = 30 #40
 
        # calculate robot step command from keyboard inputs
        result_dict = command_status.get()
        print(result_dict)
        command_status.put(result_dict)

        # wait 3 seconds to start
        if result_dict['StartStepping']:
            currentLp = trotting.positions(d-3, result_dict)
            robot.feetPosition(currentLp)
        else:
            robot.feetPosition(Lp)
        # roll=-xr

        # robot rotation 
        # robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
        roll=0 
        pitch = 0
        yaw = 0
        # pitch = -((sensor.euler[1]*math.pi)/180)
        # yaw = -((sensor.euler[0]*math.pi)/180)
        # roll = -((sensor.euler[2]*math.pi)/180)
        # robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
        robot.bodyRotation((roll,yaw,pitch))
        # bodyX=50+yr*10
        # robot.bodyPosition((bodyX, 40+height, -ir))
        robot.bodyPosition((0, 0, 0))


        # Get current Angles for each motor
        LaDian = robot.getAngle()
        print(LaDian)
        
        # First Step doesn't contains jointAngles
        if len(LaDian):
     

            # ladian to degree
            thetas = DXL_controller.LadianToAngles(LaDian)

            # adjust robot servo
            Goal_Degree = DXL_controller.AngleToServo(DXLMotor_N)

            # degree to dynamixel value
            Goal_Position_Value = DXL_controller.DegreeToDXLValue(Goal_Degree)


            # Real Actuators
            DXL_controller.WriteMotor(DXLMotor_N)
            print(" **** Goal Degree **** ")
            print(Goal_Degree)
        
            
            
            # # Plot Robot Pose into Matplotlib for Debugging
            # TODO: Matplotplib animation
            # kn.initFK(jointAngles)
            # kn.plotKinematics()

        robot.step()
        consoleClear()


if __name__ == "__main__":
    try:
        # motor num
        DXLMotor_N = 12

        # motor id from LF to RB
        DXL_ID = []
        DXL_ID = [ i + 1 for i in range(DXLMotor_N)]
        
        # torque enable
        DXL_controller.EnableTorque(DXLMotor_N)


        
        # Keyboard input Process
        KeyInputs = KeyInterrupt()
        KeyProcess = Process(target=KeyInputs.keyInterrupt, args=(1, KeyInputs.key_status, KeyInputs.command_status))
        KeyProcess.start()

        # Main Process 
        main(2, KeyInputs.command_status)
        

        # torque disable
        DXL_controller.DisableTorque(DXLMotor_N)
        DXL_controller.closeport()


        print("terminate KeyBoard Input process")
        if KeyProcess.is_alive():
            KeyProcess.terminate()
    except Exception as e:
        print(e)
    finally:
        print("Done... :)")
