# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi
from FRA333_HW3_49_65 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3

'''
1. มนัสวิน_6549
2. การันยภาส_6565
'''

d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d = d_1,offset = pi),
        rtb.RevoluteMDH(alpha = pi/2),
        rtb.RevoluteMDH(a = a_2)
    ],tool = SE3([[0 , 0 , -1 ,a_3 - d_6],
                 [0 , 1 , 0 ,-d_5],
                 [1 , 0 , 0 , d_4],
                 [0 , 0 , 0 ,  1]]),
    name = "3R"
)

# create more samples here
q_sample = [[pi/3,-pi/4,pi/5],[0,0,0],[pi/2,pi/16,-pi/4]] # --> just add more upon this list.

#===========================================<ตรวจคำตอบข้อ 1>====================================================#

print('\n        ========= Answer check 1 =========')
for i in range(len(q_sample)):
    print(f'''
>>> iteration {i+1}
        
Roboticstoolbox computation:

{robot.jacobe(q_sample[i])}

---

From our Function:

{endEffectorJacobianHW3(q_sample[i])}
∟        ''')

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#

print('\n        ========= Answer check 2 =========')
for i in range(len(q_sample)):
    print(f'\n>>> iteration {i+1}')
    checkSingularityHW3(q_sample[i])

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#

w = [1.0, 0.0, 1.0, 0.0, 1.0, 0.0]
q = [0.0, 0.0, 0.0]

print('\n        ========= Answer check 3 =========')
for i in range(len(q_sample)):
    print(f'''
>>> iteration {i+1}
        
Robot's effort:

{robot.pay(w , q_sample[i] , robot.jacobe(q_sample[i]))}

---

External force:

{computeEffortHW3(q_sample[i], w)}

---

Forces are in equilibrium
         ''')

#==============================================================================================================#







