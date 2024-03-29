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
# *********     Bulk Read Example      *********
#
#
# Available Dynamixel model on this example : MX or X series set to Protocol 1.0
# This example is tested with two Dynamixel MX-28, and an USB2DYNAMIXEL
# Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os

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

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_MOVING              = 46

# Data Byte Length
LEN_MX_GOAL_POSITION        = 2
LEN_MX_PRESENT_POSITION     = 2
LEN_MX_MOVING               = 1

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3 
DXL4_ID                     = 4 
DXL5_ID                     = 5 
DXL6_ID                     = 6               
BAUDRATE                    = 1000000            # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023           # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]        # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

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


# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)

# Enable Dynamixel#4 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL4_ID)

# Enable Dynamixel#5 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL5_ID)

# Enable Dynamixel#6 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL6_ID)

# Add parameter storage for Dynamixel#1 present position
dxl_addparam_result = groupBulkRead.addParam(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL1_ID)
    quit()

# Add parameter storage for Dynamixel#2 moving value
dxl_addparam_result = groupBulkRead.addParam(DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL2_ID)
    quit()

# Add parameter storage for Dynamixel#3 moving value
dxl_addparam_result = groupBulkRead.addParam(DXL3_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL3_ID)
    quit()

# Add parameter storage for Dynamixel#4 moving value
dxl_addparam_result = groupBulkRead.addParam(DXL4_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL4_ID)
    quit()

# Add parameter storage for Dynamixel#5 moving value
dxl_addparam_result = groupBulkRead.addParam(DXL5_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL5_ID)
    quit()

# Add parameter storage for Dynamixel#6 moving value
dxl_addparam_result = groupBulkRead.addParam(DXL6_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL6_ID)
    quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Write Dynamixel#1 goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write Dynamixel#2 goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write Dynamixel#3 goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write Dynamixel#4 goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write Dynamixel#5 goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL5_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    # Write Dynamixel#6 goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL6_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Bulkread present position and moving status
        dxl_comm_result = groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupbulkread data of Dynamixel#1 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL1_ID)
            quit()

        # Check if groupbulkread data of Dynamixel#2 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL2_ID)
            quit()

         # Check if groupbulkread data of Dynamixel#3 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL3_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL3_ID)
            quit()

         # Check if groupbulkread data of Dynamixel#4 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL4_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL4_ID)
            quit()

         # Check if groupbulkread data of Dynamixel#5 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL5_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL5_ID)
            quit()

         # Check if groupbulkread data of Dynamixel#6 is available
        dxl_getdata_result = groupBulkRead.isAvailable(DXL6_ID, ADDR_MX_MOVING, LEN_MX_MOVING)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % DXL6_ID)
            quit()

        # Get Dynamixel#1 present position value
        dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)

        # Get Dynamixel#2 moving value
        dxl2_moving_value = groupBulkRead.getData(DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING)

        print("[ID:%03d] Present Position : %d \t [ID:%03d] Is Moving: %d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_moving_value))

        if not (abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD):
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Clear bulkread parameter storage
groupBulkRead.clearParam()

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
