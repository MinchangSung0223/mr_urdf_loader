from mr_urdf_loader import loadURDF
from modern_robotics import *
urdf_name = "3DoF.urdf"
MR=loadURDF(urdf_name)
M  = MR["M"]
Slist  = MR["Slist"]
Mlist  = MR["Mlist"]
Glist  = MR["Glist"]
Blist  = MR["Blist"]
actuated_joints_num = MR["actuated_joints_num"]

thetalist = np.array([0,0,np.pi/2.0])
dthetalist = np.array([0,0,0.1])
ddthetalist = np.array([0,0,0])
g = np.array([0,0,-9.8])
Ftip = [0,0,0,0,0,0]


print("FKinSpace\n", FKinSpace(M,Slist,thetalist))
print("FKinBody\n", FKinBody(M,Blist,thetalist))

print("JacobianSpace\n", JacobianSpace(Slist,thetalist))
print("JacobianBody\n", JacobianBody(Blist,thetalist))

print("InverseDynamics\n" ,InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist,  Glist, Slist))


