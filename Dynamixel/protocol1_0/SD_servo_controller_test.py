import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from Kinematics import kinematics as kn 
import numpy as np
import servo_controller
import time

from dynamixel_sdk import *

# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

ADDR_MX_TORQUE_ENABLE      = 24               
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

PROTOCOL_VERSION            = 1.0

BAUDRATE                    = 1000000            
DEVICENAME                  = 'COM4'

TORQUE_ENABLE               = 1                
TORQUE_DISABLE              = 0

# DXL_g_deg = []
# DXL_present_deg = []
# DXL_present_POSITION_VALUE = []

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


class Dynamixel_Controllers:
    def __init__(self):
        
        # self.ADDR_MX_TORQUE_ENABLE      = 24               
        # self.ADDR_MX_GOAL_POSITION      = 30
        # self.ADDR_MX_PRESENT_POSITION   = 36

        # self.PROTOCOL_VERSION            = 1.0

        # self.BAUDRATE                    = 1000000            
        # self.DEVICENAME                  = '/dev/ttyUSB0'

        # self.TORQUE_ENABLE               = 1                
        # self.TORQUE_DISABLE              = 0

        self.DXL_goal_deg = [0 for i in range(12)]
        self.DXL_present_deg = []
        self.DXL_present_POSITION_VALUE = []
        
        self._servo_offsets = [150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150]
        
    
    def DynamixelSetting(self):

        self.openPort_()
    
    def packetHandler_(self):
        
        packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        self.portHandler_()

    
    def portHandler_(self):
        
        portHandler = PortHandler(self.DEVICENAME)
        
        self.openPort_()

    
    def openPort_(self):
        
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...") 
            getch()
            quit()
        
        self.setBaudRate_()        

    
    def setBaudRate_(self):

        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    
    def EnableTorque(self, ET):
        
        for i in range(ET):
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % i)

    def DisableTorque(self, DT):

        for i in range(DT):
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    def WriteMotor(self, WM):

        for i in range(WM):
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i + 1, ADDR_MX_GOAL_POSITION, self.DXL_goal_POSITION_VALUE[i])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
               

    def ReadMotor(self, RM):

        for i in range(RM):
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i + 1, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            DXL_present_POSITION_VALUE.append(dxl_present_position)
        print(" **** Read Present DXL Positon Value **** ") 
        print(DXL_present_POSITION_VALUE)
        DXL_present_deg = [ int(i * 0.29) for i in DXL_present_POSITION_VALUE ]
        print(" **** Read Present DXL Degree **** ")
        print(DXL_present_deg)
        DXL_present_POSITION_VALUE.clear()
        DXL_present_deg.clear()
                 

    
    def LadianToAngles(self, La):
        # radian to degree
        La *= 180/np.pi
        La = [ [ int(x) for x in y ] for y in La ]

        self._thetas = La
        return self._thetas

    
    def AngleToServo(self, num):

        # for i in range(num):
        #     self.DXL_goal_deg[i] = self._servo_offsets[i] - self._thetas[int(i/3)][int(i%3)] 

                   #FL Lower
        self.DXL_goal_deg[0] = self._servo_offsets[0] - self._thetas[0][0]
        #FL Upper
        self.DXL_goal_deg[1] = self._servo_offsets[1] - self._thetas[0][1]    
        #FL Shoulder
        self.DXL_goal_deg[2] = self._servo_offsets[2] - self._thetas[0][2]

        #FR Lower
        self.DXL_goal_deg[3] = self._servo_offsets[3] + self._thetas[1][0]
        #FR Upper
        self.DXL_goal_deg[4] = self._servo_offsets[4] + self._thetas[1][1]    
        #FR Shoulder
        self.DXL_goal_deg[5] = self._servo_offsets[5] + self._thetas[1][2]

        #BL Lower
        self.DXL_goal_deg[6] = self._servo_offsets[6] - self._thetas[2][0]
        #BL Upper
        self.DXL_goal_deg[7] = self._servo_offsets[7] - self._thetas[2][1]    
        #BL Shoulder, Formula flipped from the front
        self.DXL_goal_deg[8] = self._servo_offsets[8] - self._thetas[2][2]

        #BR Lower. 
        self.DXL_goal_deg[9] = self._servo_offsets[9] + self._thetas[3][0]
        #BR Upper
        self.DXL_goal_deg[10] = self._servo_offsets[10] + self._thetas[3][1]    
        #BR Shoulder, Formula flipped from the front
        self.DXL_goal_deg[11] = self._servo_offsets[11] + self._thetas[3][2] 

        return self.DXL_goal_deg
    
    def DegreeToDXLValue(self, Dg):
        
        self.DXL_goal_POSITION_VALUE = [ int(i / 0.29) for i in Dg ]
        return self.DXL_goal_POSITION_VALUE
    
    def DXLValueToDegree(self, VL):

        self.DXL_present_deg = [ int(i * 0.29) for i in VL ]
        return self.DXL_present_deg

    def closeport(self):

        self.portHandler.closePort()

if __name__=="__main__":
    
    # setting the DXL_Motor
    DXL_controller = Dynamixel_Controllers()
    DXL_controller.DynamixelSetting()

    # caculate inverse kinematics
    legEndpoints=np.array([[100,-100,87.5,1],[100,-100,-87.5,1],[-100,-100,87.5,1],[-100,-100,-87.5,1]])
    LaDian = kn.initIK(legEndpoints) #radians
    
    # set numbers of motor and ID
    DXLMotor_N = 4
    DXL_ID = [ i + 1 for i in range(DXLMotor_N)]
    # DXL_ID = []

    # calculate DXL_goal_position_value
    thetas = DXL_controller.LadianToAngles(LaDian)
    Goal_Degree = DXL_controller.AngleToServo(DXLMotor_N)
    Goal_Position_Value = DXL_controller.DegreeToDXLValue(Goal_Degree)

    DXL_controller.EnableTorque(DXLMotor_N)

    # write and read DXL_servo
    DXL_controller.WriteMotor(DXLMotor_N)
    print(" **** Goal Degree **** ")
    print(Goal_Degree)
    DXL_controller.ReadMotor(DXLMotor_N)

    # close the DXL_servo
    DXL_controller.DisableTorque(DXLMotor_N)
    DXL_controller.closeport()
        
   