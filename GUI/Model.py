import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

##Create model 
# link lengths in mm
a1 = float(input("a1 = "))
a2 = float(input("a2 = "))
a3 = float(input("a3 = "))
a4 = float(input("a4 = "))

#link conversion to meters
def mm_to_meter(a):
    m = 1000 #1 m = 1000mm
    return a/m

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2)
a3 = mm_to_meter(a3)
a4 = mm_to_meter(a4)

#limit of variable
d1 = float(input("d1 = "))
d1 = mm_to_meter(d1)

d2 = float(input("d2 = "))
d2 = mm_to_meter(d2)

d3 = float(input("d3 = "))
d3 = mm_to_meter(d3)

#Create links
#robot_variable = DHRobot([ReboluteDH(r,alpha,offset,qlim)])
CARTESIAN = DHRobot([
    PrismaticDH(0,0,(270/180.0)*np.pi,a1,qlim=[0,0]),
    PrismaticDH((270/180.0)*np.pi,0,(270/180.0)*np.pi,a2,qlim=[0,d1]),
    PrismaticDH((90/180.0)*np.pi,0,(270/180.0)*np.pi,a3,qlim=[0,d2]),
    PrismaticDH(0,0,0,a4,qlim=[0,d3]),
    
], name = "CARTESIAN")
print(CARTESIAN)

CARTESIAN.teach(q=[0,0,0,0])