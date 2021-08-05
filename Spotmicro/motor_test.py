import sys, os
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
from Kinematics import kinematics as kn 
import numpy as np
# import servo_controller
import time

from dynamixel_sdk import *
import time
import matplotlib.pyplot as plt

ADDR_MX_TORQUE_ENABLE      = 24               
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38

PROTOCOL_VERSION            = 1.0

BAUDRATE                    = 1000000            
DEVICENAME                  = '/dev/ttyUSB0'
# MAXTORQUE
TORQUE_ENABLE               = 1                
TORQUE_DISABLE              = 0

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# set the number of dynamixel motor
DXL_Motor = 12

DXL_ID = []
DXL_ID = [ i + 1 for i in range(DXL_Motor)] 
i = 2   
DXL_goal_POSITION_VALUE = [512 for i in range(DXL_Motor)]

# Open port
# portHandler.
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

# for i in range(DXL_Motor):
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % i)

while 1:
    speed = int(input("speed : (0 ~ 1024) : "))
    position_value = int(input(" position value(0 ~ 1024) : "))
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
        # Write goal position
    # for i in range(DXL_Motor):
    secs = time.time()
    print(secs)
    xl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i, ADDR_MX_MOVING_SPEED, speed)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    xl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i, ADDR_MX_GOAL_POSITION, position_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    # print(" *** goal degree *** ")
    # print(DXL_goal_deg)
        # for i in range(DXL_Motor):
    dxl_present_speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i , ADDR_MX_PRESENT_SPEED)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    print(" present speed ")
    print(dxl_present_speed)
    while 1 :
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i , ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        
        print(dxl_present_position)
        if dxl_present_position == position_value:
            secs1 = time.time()
            print(secs1)
            break
    print(dxl_present_position)


# for i in range(DXL_Motor):
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))


portHandler.closePort()