import pybullet as p
import time
import pybullet_data

# ROBOT_URDF = "./urdf/spotmicroai_no_mesh.urdf"
ROBOT_URDF = "./urdf/spotmicroai_sample.urdf"

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.setTimeStep(1/200)

planeID = p.loadURDF("plane.urdf")

cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 3.14])

robotID = p.loadURDF(fileName=ROBOT_URDF, basePosition=cubeStartPos, baseOrientation=cubeStartOrientation, useFixedBase=True)

while True:
    p.stepSimulation()