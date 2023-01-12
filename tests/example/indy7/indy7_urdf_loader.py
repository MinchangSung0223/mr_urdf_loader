import numpy as np
np.set_printoptions(precision=6, suppress=True)
from mr_urdf_loader import loadURDF
from modern_robotics import *
urdf_name = "indy7/indy7.urdf"
MR=loadURDF(urdf_name)
M  = np.array(MR["M"])
Slist  = np.array(MR["Slist"])
Mlist  = np.array(MR["Mlist"])
Glist  = np.array(MR["Glist"])
Blist  = np.array(MR["Blist"])
actuated_joints_num = MR["actuated_joints_num"]

thetalist = np.array([0,0,np.pi/2.0])
dthetalist = np.array([0,0,0.1])
ddthetalist = np.array([0,0,0])
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]


#print(M)
print(Mlist)
print(Glist)
print("FKinSpace\n", FKinSpace(M,Slist,thetalist))
print("FKinBody\n", FKinBody(M,Blist,thetalist))

print("JacobianSpace\n", JacobianSpace(Slist,thetalist))
print("JacobianBody\n", JacobianBody(Blist,thetalist))

print("InverseDynamics\n" ,InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist,  Glist, Slist))


