#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address (AX-12A setting)
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# set the number of dynamixel motor
DXL_Motor = 3

# Dynamixel motor 'AX-12A' unique number 
# DXL_ID                      = 14
# DXL1_ID                     = 10                
# DXL2_ID                     = 11   
# DXL_ID = [1 ,2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
DXL_ID = []
DXL_ID = [ i + 1 for i in range(DXL_Motor)] 


# baudrate and port set
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller

# Value for using of torque
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
            
# set position value of dynamixel motor                  
# DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
# DXL_MAXIMUM_POSITION_VALUE1  = 512        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
# DXL_goal_POSITION_VALUE = [ 512 for i in range(DXL_Motor)]
DXL_goal_POSITION_VALUE = [582, 292, 310] # 1 positon value = 0.29 deg / max position value is 1023
# DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# set positon value to degree
DXL_goal_deg = []
DXL_present_POSITION_VALUE = []
DXL_goal_deg = [ int(i * 0.29) for i in DXL_goal_POSITION_VALUE ]
# DXL_goal_deg = [ (int)i for i in DXL_goal_deg ]



# index = 1
# dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))
# else:
#     print("Dynamixel has been successfully connected")

# Enable Dynamixel Torque
for i in range(DXL_Motor):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % i)

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Write goal position
    for i in range(DXL_Motor):
        xl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i + 1, ADDR_MX_GOAL_POSITION, DXL_goal_POSITION_VALUE[i])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    print(" *** goal degree *** ")
    print(DXL_goal_deg)
        
    # # Write goal position
    # dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, DXL_MAXIMUM_POSITION_VALUE1)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    # while 1:
    # Read present position
    for i in range(DXL_Motor):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i + 1, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        DXL_present_POSITION_VALUE.append(dxl_present_position)
    print(" *** present positon value *** ") 
    print(DXL_present_POSITION_VALUE)   
    DXL_present_deg = [ int(i * 0.29) for i in DXL_goal_POSITION_VALUE ]
    print(" *** present degree *** ")
    print(DXL_present_deg)
        
    # Read present position
    # while 1:
    #     dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position[index], dxl_present_position))
    #     if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
    #             break

    # # Change goal position
    # if index == 0:
    #     index = 1
    # else:
    #     index = 0

# Disable Dynamixel Torque
for i in range(DXL_Motor):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()

