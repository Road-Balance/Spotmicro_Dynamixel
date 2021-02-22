import sys
sys.path.append("..")

import Kinematics.kinematics as kn
import numpy as np
from adafruit_servokit import ServoKit
import board
import busio
import time

from dynamixel_sdk import *

ADDR_MX_TORQUE_ENABLE      = 24               
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

PROTOCOL_VERSION            = 1.0

DXL_Motor = 12

DXL_ID = []
DXL_ID = [ i + 1 for i in range(DXL_Motor)]

BAUDRATE                    = 1000000            
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                
TORQUE_DISABLE              = 0

DXL_goal_POSITION_VALUE = [ 512 for i in range(DXL_Motor)]

DXL_goal_deg = []
DXL_present_POSITION_VALUE = []
DXL_goal_deg = [ int(i * 0.29) for i in DXL_goal_POSITION_VALUE ]

portHandler = PortHandler(DEVICENAME)

packetHandler = PacketHandler(PROTOCOL_VERSION)

class Controllers:
    def __init__(self):

        # print("Initializing Servos")
        # self._i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        # print("Initializing ServoKit")
        # self._kit = ServoKit(channels=16, i2c=self._i2c_bus0, address=0x40)
        # self._kit2 = ServoKit(channels=16, i2c=self._i2c_bus0, address=0x41)
        # print("Done initializing")

        #1 by 12 array
        self.MOTOR_LEG_FRONT = 0
        self.MOTOR_LEG_BACK = 6
        self.MOTOR_LEG_LEFT = 0 
        self.MOTOR_LEG_RIGHT = 3
        self.MOTOR_LEG_SHOULDER = 2
        self.MOTOR_LEG_UPPER = 1
        self.MOTOR_LEG_LOWER = 0

        #4 by 3 matrix
        self.SIM_LEG_FRONT = 0
        self.SIM_LEG_BACK = 2
        self.SIM_LEG_LEFT = 0
        self.SIM_LEG_RIGHT = 1
        self.SIM_LEG_THETA1 = 0
        self.SIM_LEG_THETA2 = 1
        self.SIM_LEG_THETA3 = 2

        # [0]~[2] : 왼쪽 앞 다리 // [3]~[5] : 오른쪽 앞 다리 // [6]~[8] : 왼쪽 뒷 다리 // [9]~[11] : 오른쪽 뒷 다리
        # centered position perpendicular to the ground
        self._servo_offsets = [170, 85, 90, 1, 95, 90, 172, 90, 90, 1, 90, 95]
        #self._servo_offsets = [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]

        self._val_list = np.zeros(12) #[ x for x in range(12) ]

        # All Angles for Leg 3 * 4 = 12 length
        self._thetas = []

    def getDegreeAngles(self, La):
        # radian to degree
        La *= 180/np.pi
        La = [ [ int(x) for x in y ] for y in La ]

        self._thetas = La

    # Angle mapping from radian to servo angles
    def angleToServo(self, La):

        self.getDegreeAngles(La)

        #FL Lower
        self._val_list[0] = self._servo_offsets[0] - self._thetas[0][2]
        #FL Upper
        self._val_list[1] = self._servo_offsets[1] - self._thetas[0][1]    
        #FL Shoulder
        self._val_list[2] = self._servo_offsets[2] + self._thetas[0][0]

        #FR Lower
        self._val_list[3] = self._servo_offsets[3] + self._thetas[1][2]
        #FR Upper
        self._val_list[4] = self._servo_offsets[4] + self._thetas[1][1]    
        #FR Shoulder
        self._val_list[5] = self._servo_offsets[5] - self._thetas[1][0]

        #BL Lower
        self._val_list[6] = self._servo_offsets[6] - self._thetas[2][2]
        #BL Upper
        self._val_list[7] = self._servo_offsets[7] - self._thetas[2][1]    
        #BL Shoulder, Formula flipped from the front
        self._val_list[8] = self._servo_offsets[8] - self._thetas[2][0]

        #BR Lower. 
        self._val_list[9] = self._servo_offsets[9] + self._thetas[3][2]
        #BR Upper
        self._val_list[10] = self._servo_offsets[10] + self._thetas[3][1]    
        #BR Shoulder, Formula flipped from the front
        self._val_list[11] = self._servo_offsets[11] + self._thetas[3][0]     

    def getServoAngles(self):
        return self._val_list

    def servoRotate(self, thetas):
        self.angleToServo(thetas)
        #self.angleToServo(np.zeros((4,3)))
        for x in range(len(self._val_list)):
            
            if x>=0 and x<12:
                self._val_list[x] = (self._val_list[x]-26.36)*(1980/1500)
                #print(self._val_list[x], end=' ')
                #if x%3 == 2: print()
                print(self._val_list[x])

                if (self._val_list[x] > 180):
                    print("Over 180!!")
                    self._val_list[x] = 179
                    continue
                if (self._val_list[x] <= 0):
                    print("Under 0!!")
                    self._val_list[x] = 1
                    continue
                if x < 6:
                    self._kit.servo[x].angle = self._val_list[x]
                else:
                    self._kit2.servo[x].angle = self._val_list[x]


if __name__=="__main__":
    
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
    
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
    
    for i in range(DXL_Motor):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % i)

    legEndpoints=np.array([[100,-100,87.5,1],[100,-100,-87.5,1],[-100,-100,87.5,1],[-100,-100,-87.5,1]])
    thetas = kn.initIK(legEndpoints) #radians
    
    controller = Controllers()

    controller.angleToServo(thetas)
    controller.servoDynamixel_angle()

    while 1:
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break

        for i in range(DXL_Motor):
            xl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i + 1, ADDR_MX_GOAL_POSITION, DXL_goal_POSITION_VALUE[i])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("goal degree : %03d" % DXL_goal_deg)

        for i in range(DXL_Motor):
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i + 1, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            DXL_present_POSITION_VALUE.append(dxl_present_position)
        print("present positon value : %04d" % DXL_present_POSITION_VALUE )   
        DXL_present_deg = [ int(i * 0.29) for i in DXL_goal_POSITION_VALUE ]
        print("present degree : %03d" % DXL_present_deg)

    for i in range(DXL_Motor):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    portHandler.closePort()
        
    # legEndpoints=np.array([[100,-100,87.5,1],[100,-100,-87.5,1],[-100,-100,87.5,1],[-100,-100,-87.5,1]])
    # thetas = kn.initIK(legEndpoints) #radians
    
    # controller = Controllers()

    # # Get radian thetas, transform to integer servo angles
    # # then, rotate servos
    # controller.servoRotate(thetas)

    # # Get AngleValues for Debugging
    # svAngle = controller.getServoAngles()
    # print(svAngle)

    # # #plot at the end
    # kn.plotKinematics()