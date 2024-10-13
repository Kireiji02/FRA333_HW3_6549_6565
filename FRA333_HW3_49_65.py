# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
import numpy as np
from HW3_utils import FKHW3

'''
1. มนัสวิน_6549
2. การันยภาส_6565
'''

#=============================================<คำตอบข้อ 1>======================================================#

'''
Input: Joint angle list
Output: Jacobain matrix
'''

def endEffectorJacobianHW3(q:list[float]):
    # Forward Kinematic function, consist of Rotation matrix from frame 0 to frame 1-3 / Position vector from frame 0 to frame 1-3 
    # / Rotation matrix from frame 0 to end-effector / Position vector from frame 0 to end-effector frame
    R,P,R_e,P_e = FKHW3(q)

    #  Rotation matrix from end_effector to frame 0
    R_e_0 = np.array(R_e).transpose()

    # Store the Jacobian components
    J_temp = []

    # Extract the data from function
    for i in range(len(q)):

        # Position vector 
        P_0_i = P[:,i]

        # Rotation matrix
        Z_i = np.array(R[:,2,i])

        # Linear Jacobian 
        J_i = np.array(np.cross(Z_i,(P_e - P_0_i)))

        # Combine the Angular Jacobian with Linear Jacobian in vertical to be a Jacobian component matrix
        J_col = np.vstack((R_e_0 @ J_i.reshape(3,1),R_e_0 @ Z_i.reshape(3,1))) ## Reference at the end effector

        # Append the component to J_temp
        J_temp.append(J_col)
    
    # Combine the matrix in horizontal to be a Jacobian matrix
    J = np.hstack(J_temp)
    return J


#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#

'''
Input: Joint angle list
Output: Singularity flag
'''

def checkSingularityHW3(q:list[float])->bool:

    # We can't control the orientation at the end effector, compute only linear and remove the angular term. 
    J = endEffectorJacobianHW3(q)[:3,:]

    # Determinant of Jacobian Matrix
    det_J = np.linalg.det(J)
    flag = None
    # Threshold for checking if det is not equal to 
    if abs(det_J) < 1e-4:  
        flag = True
    else:
        flag = False
    
    print(f"\n Determinant of Jacobian Matrix: {det_J} \n Singularity Status: {flag} \n")

    return flag

#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#

'''
Input: Joint angle list, External forces and torques list
Output: Effort list
'''

def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J_e = (endEffectorJacobianHW3(q))
    J_T = np.transpose(J_e) # Jacobian matrix transpose
    
    effort = J_T @ w # J_T cross to wrench
    return effort
    
#==============================================================================================================#