import pybullet as p
import time
import pybullet_data
import numpy as np
np.set_printoptions(precision=6, suppress=True)
import math
from modern_robotics import *
import modern_robotics as mr
import numpy as np
from mr_urdf_loader import loadURDF
def pos_orn_to_T(pos,orn):
	T = np.eye(4)
	T[0:3,3] = np.array(pos).T
	T[0:3,0:3] = np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
	return T
	


urdf_name = "indy7/indy7.urdf"

## Modern Robotics setup
MR=loadURDF(urdf_name)
M  = MR["M"]
Slist  = MR["Slist"]
Mlist  = MR["Mlist"]
Glist  = MR["Glist"]
Blist  = MR["Blist"]
actuated_joints_num = MR["actuated_joints_num"]
## pybullet setup
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

robotID = p.loadURDF(urdf_name, [0, 0, 0],[0, 0, 0, 1],useFixedBase=1)
numJoints = p.getNumJoints(robotID)
p.resetBasePositionAndOrientation(robotID, [0, 0, 0], [0, 0, 0, 1])
for i in range(0,numJoints):
	p.setJointMotorControl2(robotID, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
	
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)
timeStep = 1/240.0;
p.setTimeStep(timeStep)

q = [0,0,0,0,0,0]
q_dot = [0,0,0,0,0,0]
q_ddot = [0,0,0,0,0,0]
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]


while p.isConnected():
	# get Joint Staes
	jointStates = np.array(p.getJointStates(robotID,[1,2,3,4,5,6]))
	# get Link Staes	
	linkState = np.array(p.getLinkState(robotID,7,1,1))
	q = np.array(jointStates[:,0])
	q_dot = np.array(jointStates[:,1])
	
	# Pybullet Forward Kinematics 
	pb_Tsb = pos_orn_to_T(linkState[0],linkState[1])
	# Modern Robotics Forward Kinematics 	
	mr_Tsb = mr.FKinSpace(M,Slist,q)
	mr_Tsb = mr.FKinBody(M,Blist,q)
	
	
	# Pybullet Jacobian
	#pb_J =  p.calculateJacobian(robotID,4,[0,0,0],[q[0],q[1],q[2]],[q_dot[0],q_dot[1],q_dot[2]],[0,0,0])
	# Modern Robotics Jacobian	
	mr_Jb= mr.JacobianBody(Blist, q)
	mr_Js= mr.JacobianSpace(Slist, q)	
	#mr_Ja = AnalyticJacobianBody(M,Blist, q)	# pb_j = mr_Ja	
	
	#pybullet InverseDynamics		
	print(q)
	pb_ID= np.array(p.calculateInverseDynamics(robotID,[q[0],q[1],q[2],q[3],q[4],q[5] ],[q_dot[0],q_dot[1],q_dot[2],q_dot[3],q_dot[4],q_dot[5] ] ,[0,0,0,0,0,0] ))
	#modern_robotics InverseDynamics
	mr_ID =mr.InverseDynamics(q, q_dot, q_ddot, g, Ftip, Mlist,  Glist, Slist)
	
	print("=============pb_Tsb=============")
	print(pb_Tsb)
	print("=============mr_Tsb=============")
	print(mr_Tsb)
	print("=============pb_ID=============")
	print(pb_ID)
	print("=============mr_ID=============")
	print(mr_ID)
	
	#set torques
	for i in range(0,actuated_joints_num):		
		p.setJointMotorControl2(robotID, i+1, p.TORQUE_CONTROL,force=mr_ID[i])
		#p.setJointMotorControl2(robotID, i+1, p.TORQUE_CONTROL,force=pb_ID[i])
				
	p.stepSimulation()
	time.sleep(timeStep)
