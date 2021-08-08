#!/usr/bin/env python
"""
DESCRIPTION:

SUBSCRIBERS:
"""

import numpy as np
import copy
import sys
import os
import time
import board
import adafruit_bno055
import busio

sys.path.append("../../")

from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.GaitGenerator.Bezier import BezierGait
from spot_bullet.src.ars_lib.ars import ARSAgent, Normalizer, Policy
from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv

# AGENT PARAMS
CD_SCALE = 0.05
SLV_SCALE = 0.05
RESIDUALS_SCALE = 0.015
Z_SCALE = 0.035
# Filter actions
alpha = 0.7
# Added this to avoid filtering residuals
# -1 for all
actions_to_filter = 14


# 가속도 센서 Initializing
i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)
sensor = adafruit_bno055.BNO055_I2C(i2c_bus0)


class SpotCommander:
    def __init__(self, Agent=True, contacts=False):

        self.Agent = Agent
        self.agent_num = 999

        # FIXED
        self.BaseStepVelocity = 0.001
        self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
        # Stock, use Bumpers to change
        self.BaseSwingPeriod = 0.2
        self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)
        self.SwingPeriod_LIMITS = [0.1, 0.3]
        # Stock, use arrow pads to change
        self.BaseClearanceHeight = 0.035
        self.BasePenetrationDepth = 0.003
        self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
        self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
        self.ClearanceHeight_LIMITS = [0.0, 0.04]
        self.PenetrationDepth_LIMITS = [0.0, 0.02]

        self.enable_contact = contacts

        # Contacts: FL, FR, BL, BR
        self.contacts = [0, 0, 0, 0]

        # IMU: R, P, Ax, Ay, Az, Gx, Gy, Gz
        self.imu = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # See spot_params.yaml in config
        self.spot = SpotModel(
            shoulder_length=0.055,
            elbow_length=0.10652,
            wrist_length=0.145,
            hip_x=0.23,
            hip_y=0.075,
            foot_x=0.23,
            foot_y=0.185,
            height=0.20,
            com_offset=0.0,
        )

        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        # See spot_params.yaml in config
        self.bzg = BezierGait(dt=0.001, Tswing=0.2)

        if self.Agent:
            self.load_spot(contacts, agent_num=self.agent_num)

        self.angles = []
        self.time = time.time()

    def load_spot(self, contacts, state_dim=12, action_dim=14, agent_num=0):
        self.policy = Policy(state_dim=state_dim, action_dim=action_dim)
        self.normalizer = Normalizer(state_dim=state_dim)
        env = spotBezierEnv(
            render=False, on_rack=False, height_field=False, draw_foot_path=False
        )
        agent = ARSAgent(self.normalizer, self.policy, env)
        my_path = os.path.abspath(os.path.dirname(__file__))
        if contacts:
            models_path = os.path.join(my_path, "spot_bullet/models/contact")
        else:
            models_path = os.path.join(my_path, "spot_bullet/models/no_contact")

        print("MODEL PATH: {}".format(my_path))
        file_name = "spot_ars_"
        if os.path.exists(models_path + "/" + file_name + str(agent_num) + "_policy"):
            print("Loading Existing agent: {}".format(agent_num))
            agent.load(models_path + "/" + file_name + str(agent_num))
            agent.policy.episode_steps = np.inf
            self.policy = agent.policy

        self.action = np.zeros(action_dim)
        self.old_act = self.action[:actions_to_filter]

    def getAngle(self):
        return self.angles

    def imu_cb(self, imu):
        """Reads the IMU

        Args: imu
        """
        # Update imu
        self.imu = [
            imu.roll,
            imu.pitch,
            np.radians(imu.gyro_x),
            np.radians(imu.gyro_y),
            np.radians(imu.gyro_z),
            imu.acc_x,
            imu.acc_y,
            imu.acc_z - 9.81,
        ]

    def move(self):
        """Turn joystick inputs into commands"""

        self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
        self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)

        print(sensor.acceleration)
        print(sensor.gyro)
        print(sensor.euler)

        StepLength = 0.2
        LateralFraction = 0.0
        YawRate = 0.0

        pos = np.array([0.0, 0.0, 0.0])
        orn = np.array([0.0, 0.0, 0.0])

        # OPTIONAL: Agent
        if self.Agent:
            phases = copy.deepcopy(self.bzg.Phases)
            # Total 12
            state = []
            # r, p, gz, gy, gz, ax, ay, az (8)
            state.extend(self.imu)
            # FL, FR, BL, BR (4)
            state.extend(phases)
            # FL, FR, BL, BR (4)
            if self.enable_contact:
                state.extend(self.contacts)
            self.normalizer.observe(state)
            # Don't normalize contacts
            state[:-4] = self.normalizer.normalize(state)[:-4]
            self.action = self.policy.evaluate(state, None, None)
            self.action = np.tanh(self.action)
            # EXP FILTER
            self.action[:actions_to_filter] = (
                alpha * self.old_act + (1.0 - alpha) * self.action[:actions_to_filter]
            )
            self.old_act = self.action[:actions_to_filter]

            self.ClearanceHeight += self.action[0] * CD_SCALE

        # Time
        dt = time.time() - self.time
        self.time = time.time()

        # Update Step Period
        self.bzg.Tswing = self.SwingPeriod

        # CLIP
        self.ClearanceHeight = np.clip(
            self.ClearanceHeight,
            self.ClearanceHeight_LIMITS[0],
            self.ClearanceHeight_LIMITS[1],
        )
        self.PenetrationDepth = np.clip(
            self.PenetrationDepth,
            self.PenetrationDepth_LIMITS[0],
            self.PenetrationDepth_LIMITS[1],
        )

        self.T_bf = self.bzg.GenerateTrajectory(
            StepLength,
            LateralFraction,
            YawRate,
            self.StepVelocity,
            self.T_bf0,
            self.T_bf,
            self.ClearanceHeight,
            self.PenetrationDepth,
            self.contacts,
            dt,
        )

        T_bf_copy = copy.deepcopy(self.T_bf)
        # OPTIONAL: Agent
        if self.Agent:
            self.action[2:] *= RESIDUALS_SCALE
            # Add DELTA to XYZ Foot Poses
            T_bf_copy["FL"][:3, 3] += self.action[2:5]
            T_bf_copy["FR"][:3, 3] += self.action[5:8]
            T_bf_copy["BL"][:3, 3] += self.action[8:11]
            T_bf_copy["BR"][:3, 3] += self.action[11:14]
            pos[2] += abs(self.action[1]) * Z_SCALE

        joint_angles = self.spot.IK(orn, pos, T_bf_copy)

        self.angles = joint_angles

        # ja_msg.fls = np.degrees(joint_angles[0][0])  # fls
        # ja_msg.fle = np.degrees(joint_angles[0][1])  # fle
        # ja_msg.flw = np.degrees(joint_angles[0][2])  # flw

        # ja_msg.frs = np.degrees(joint_angles[1][0])  # frs
        # ja_msg.fre = np.degrees(joint_angles[1][1])  # fre
        # ja_msg.frw = np.degrees(joint_angles[1][2])  # frw

        # ja_msg.bls = np.degrees(joint_angles[2][0])  # bls
        # ja_msg.ble = np.degrees(joint_angles[2][1])  # ble
        # ja_msg.blw = np.degrees(joint_angles[2][2])  # blw

        # ja_msg.brs = np.degrees(joint_angles[3][0])  # brs
        # ja_msg.bre = np.degrees(joint_angles[3][1])  # bre
        # ja_msg.brw = np.degrees(joint_angles[3][2])  # brw
