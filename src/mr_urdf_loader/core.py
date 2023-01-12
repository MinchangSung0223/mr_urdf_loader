from __future__ import annotations
from urdfpy import URDF
import numpy as np
import modern_robotics as mr
def TransformToXYZ(T):
	return np.array(T[0:3,3])
def getJoint(robot,link_name):
	ret_link=[]
	for link in robot.links:
		if link.name == link_name:
			ret_link=link
			break;
	return ret_link
def w_p_to_Slist(w,p,ROBOT_DOF):
    Slist = []
    for i in range(0,ROBOT_DOF):
      w_ = w[i];
      p_ = p[i];      
      v_ = -np.cross(w_,p_)
      Slist.append([w_[0],w_[1],w_[2],v_[0],v_[1],v_[2]])
    return np.transpose(Slist)
def AnalyticJacobianBody(M,Blist, thetalist):
    """Computes the Analytic Jacobian for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param thetalist: A list of joint coordinates
    :return: The body Jacobian corresponding to the inputs (6xn real
             numbers)

    Example Input:
        Blist = np.array([[0, 0, 1,   0, 0.2, 0.2],
                          [1, 0, 0,   2,   0,   3],
                          [0, 1, 0,   0,   2,   1],
                          [1, 0, 0, 0.2, 0.3, 0.4]]).T
        thetalist = np.array([0.2, 1.1, 0.1, 1.2])
    Output:
        np.array([[-0.04528405, 0.99500417,           0,   1]
                  [ 0.74359313, 0.09304865,  0.36235775,   0]
                  [-0.66709716, 0.03617541, -0.93203909,   0]
                  [ 2.32586047,    1.66809,  0.56410831, 0.2]
                  [-1.44321167, 2.94561275,  1.43306521, 0.3]
                  [-2.06639565, 1.82881722, -1.58868628, 0.4]])
    """
    Jb =  JacobianBody(Blist, thetalist)
    Tsb = FKinBody(M,Blist, thetalist)
    Rsb= Tsb[0:3,0:3]
    r = so3ToVec(Rsb)
    norm_r = np.sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2])
    omega_r = VecToso3(r)
    A = np.eye(3) - (1-np.cos(norm_r))/(norm_r*norm_r) * omega_r+ (norm_r - np.sin(norm_r)) / (norm_r**3) * omega_r @ omega_r
    A_ = np.eye(6)
    A_[0:3,0:3] = Rsb
    A_[3:6,3:6] = Rsb
    Ja = A_ @ Jb
    
    

    return Ja
def loadURDF(urdf_name):
	print(urdf_name)
	robot = URDF.load(urdf_name)
	LinkNum = len(robot.links)
	JointNum = len(robot.actuated_joints)

	p_ = []	
	w_= []
	M_list = [np.eye(4)]

	Glist =[]
	com_p_=[]
	for joint in robot.actuated_joints:
		child_link = getJoint(robot,joint.child)
		p_.append(TransformToXYZ(robot.link_fk()[child_link]))
		G = np.eye(6)
		G[0:3,0:3] = child_link.inertial.inertia
		G[3:6,3:6] = child_link.inertial.mass*np.eye(3)
		Glist.append(G)

		child_M = robot.link_fk()[child_link] 
		child_M_R, child_M_p =mr.TransToRp(child_M)
		child_w = np.array(child_M_R @ np.array(joint.axis).T)
		w_.append( child_w ) 
		child_M[0:3,0:3] = np.eye(3)
		CoM_M =  child_M@ child_link.inertial.origin

		M_list.append(CoM_M)
	eef_link = getJoint(robot,robot.end_links[0].name)
	M = robot.link_fk()[eef_link]	
		
	M_list.append(M)
	Slist = w_p_to_Slist(w_,p_,JointNum)	
	Blist = mr.Adjoint(mr.TransInv(M))@ Slist
	Mlist = []
	for i in range(1,len(M_list)):
		M_i_1 = M_list[i-1]
		M_i = M_list[i]	
		Mlist.append(mr.TransInv(M_i_1) @ M_i)
	Mlist = np.array(Mlist)	
	Glist = np.array(Glist)
	
	return {"M":M,"Slist" : Slist,"Blist": Blist,"Mlist":Mlist,"Glist":Glist,"actuated_joints_num":len(robot.actuated_joints)}
	
	
