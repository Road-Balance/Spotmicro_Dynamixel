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
# import SD_servo_controller_test

from multiprocessing import Process
from Common.multiprocess_kb import KeyInterrupt
from Kinematics.kinematicMotion import KinematicMotion, TrottingGait

from dynamixel_sdk import *

rtime=time.time()

i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)
sensor = adafruit_bno055.BNO055_I2C(i2c_bus0)

ADDR_MX_TORQUE_ENABLE      = 24               
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

PROTOCOL_VERSION            = 1.0

BAUDRATE                    = 1000000            
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                
TORQUE_DISABLE              = 0



# DXL_goal_POSITION_VALUE = [ 512 for i in range(DXL_Motor)]

# DXL_goal_deg = []
# DXL_present_POSITION_VALUE = []
# DXL_goal_deg = [ int(i * 0.29) for i in DXL_goal_POSITION_VALUE ]

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

DXL_controller = Dynamixel_Controllers()
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
# a = 150
# b = 130         
# c = 60


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
        # roll=0 
        pitch = 0
        yaw = 0
        # pitch = -((sensor.euler[1]*math.pi)/180)
        # yaw = -((sensor.euler[0]*math.pi)/180)
        roll = -((sensor.euler[2]*math.pi)/180)
        robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
        robot.bodyRotation((roll,yaw,pitch))
        # bodyX=50+yr*10
        # robot.bodyPosition((bodyX, 40+height, -ir))
        robot.bodyPosition((0, 0, 0))

        print(sensor.euler[1])
        print((sensor.euler[1]*math.pi)/180)

        # Get current Angles for each motor
        LaDian = robot.getAngle()
        print(LaDian)
        
        # First Step doesn't contains jointAngles
        if len(LaDian):
            # Real Actuators
            # controller.servoRotate(jointAngles)

            thetas = DXL_controller.LadianToAngles(LaDian)
            Goal_Degree = DXL_controller.AngleToServo(DXLMotor_N)
            Goal_Position_Value = DXL_controller.DegreeToDXLValue(Goal_Degree)

            # controller.angleToServo(jointAngles)
            # DXL_goal_deg = controller.servoDynamixel_angle()
            # DXL_goal_POSITION_VALUE = [ int(i / 0.29) for i in DXL_goal_deg ]

            DXL_controller.WriteMotor(DXLMotor_N)
            print(" **** Goal Degree **** ")
            print(Goal_Degree)
            # DXL_controller.ReadMotor(DXLMotor_N)
            # for i in range(DXL_Motor):
            #     dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i + 1, ADDR_MX_GOAL_POSITION, DXL_goal_POSITION_VALUE[i])
            #     if dxl_comm_result != COMM_SUCCESS:
            #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            #     elif dxl_error != 0:
            #         print("%s" % packetHandler.getRxPacketError(dxl_error))
            # print(" *** goal degree *** ")
            # print(DXL_goal_deg)

            # for i in range(DXL_Motor):
            #     dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i + 1, ADDR_MX_PRESENT_POSITION)
            #     if dxl_comm_result != COMM_SUCCESS:
            #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            #     elif dxl_error != 0:
            #         print("%s" % packetHandler.getRxPacketError(dxl_error))
            #     DXL_present_POSITION_VALUE.append(dxl_present_position)
            # print(" *** present positon value *** ") 
            # print(DXL_present_POSITION_VALUE)   
            # DXL_present_deg = [ int(i * 0.29) for i in DXL_goal_POSITION_VALUE ]
            # print(" *** present degree *** ")
            # print(DXL_present_deg)

            # for i in range(DXL_Motor):
            #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            #     if dxl_comm_result != COMM_SUCCESS:
            #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            #     elif dxl_error != 0:
            #         print("%s" % packetHandler.getRxPacketError(dxl_error))

            # portHandler.closePort()          
            
            # # Plot Robot Pose into Matplotlib for Debugging
            # TODO: Matplotplib animation
            # kn.initFK(jointAngles)
            # kn.plotKinematics()

        robot.step()
        consoleClear()


if __name__ == "__main__":
    try:
        DXLMotor_N = 12

        DXL_ID = []
        DXL_ID = [ i + 1 for i in range(DXLMotor_N)]
        
        # if portHandler.openPort():
        #     print("Succeeded to open the port")
        # else:
        #     print("Failed to open the port")
        #     print("Press any key to terminate...") 
        #     getch()
        #     quit()
    
        # if portHandler.setBaudRate(BAUDRATE):
        #     print("Succeeded to change the baudrate")
        # else:
        #     print("Failed to change the baudrate")
        #     print("Press any key to terminate...")
        #     getch()
        #     quit()
        DXL_controller.EnableTorque(DXLMotor_N)

        # for i in range(DXL_Motor):
        #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        #     if dxl_comm_result != COMM_SUCCESS:
        #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        #     elif dxl_error != 0:
        #         print("%s" % packetHandler.getRxPacketError(dxl_error))
        #     else:
        #         print("Dynamixel#%d has been successfully connected" % i)
        
        # Keyboard input Process
        KeyInputs = KeyInterrupt()
        KeyProcess = Process(target=KeyInputs.keyInterrupt, args=(1, KeyInputs.key_status, KeyInputs.command_status))
        KeyProcess.start()

        # Main Process 
        main(2, KeyInputs.command_status)
        

        DXL_controller.DisableTorque(DXLMotor_N)
        DXL_controller.closeport()
        # for i in range(DXL_Motor):
        #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        #     if dxl_comm_result != COMM_SUCCESS:
        #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        #     elif dxl_error != 0:
        #         print("%s" % packetHandler.getRxPacketError(dxl_error))

        # portHandler.closePort()

        print("terminate KeyBoard Input process")
        if KeyProcess.is_alive():
            KeyProcess.terminate()
    except Exception as e:
        print(e)
    finally:
        print("Done... :)")
