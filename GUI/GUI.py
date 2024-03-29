# The code below is importing the necessary libraries for the program to run.
from tkinter import *
from tkinter import messagebox
from tkinter import PhotoImage
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib
matplotlib.use('GTK3Agg')


# Creating a GUI window with a title and an icon.
gui = Tk()
gui.title("CARTESIAN DESIGN CALCULATOR")
gui.resizable(False,False)
gui.config(bg="black")

def reset():
    """
    It clears all the text boxes.
    """
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)
    a4_E.delete(0, END)

    d1_E.delete(0, END)
    d2_E.delete(0, END)
    d3_E.delete(0, END)
    
    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

def f_k():
 
    """
    It calculates the forward kinematics of the robot.
    """
    
    ### Forward Kinematics of Cartesian Manipulator


    # link lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
    a4 = float(a4_E.get())

    # joint variables: is cm if f, is degrees if theta
    d1 = float(d1_E.get())
    d2 = float(d2_E.get())
    d3 = float(d3_E.get())
    

    # Parametric Table (theta, alpha, r, d)
    PT = [[(0.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a1],
          [(270.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a2+d1],
          [(90.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a3+d2],
          [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a4+d3]]

    # HTM formulae
    i = 0
    H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 1
    H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 2
    H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 3
    H3_4 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    # Position/Translation Joints
    H0_1 = np.matrix(H0_1)

    H1_2 = np.matrix(H1_2)

    H2_3 = np.matrix(H2_3)

    H3_4 = np.matrix(H3_4)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3,H3_4)

    X0_4 = H0_4[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_4,3))

    Y0_4 = H0_4[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_4,3))

    Z0_4 = H0_4[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_4,3))
        
        
    ## Jacobian Matrix Program

    # Jacobian Window

    J_sw = Toplevel()
    J_sw.title("Velocity Calculator")
    J_sw.resizable(False,False)
    J_sw.config(bg="black")
   
    #1. Linear/Translation Vectors
    Z_1 = [[0],
           [0],
           [1]] #0,0,1 vector

    #Row 1 to 3, column 1
    J1 = [[1,0,0],
          [0,1,0],
          [0,0,1]] #R0_0
    J1 = np.dot(J1,Z_1)
    J1 = np.matrix(J1)

    #Row 1 to 3, column 2
    Z_2 = [[0],
           [0],
           [1]] #0,0,1 vector

    #Row 1 to 3, column 2
    J2 = [[0,0,1],
          [1,0,0],
          [0,1,0]] #R0_1
    J2 = np.dot(J2,Z_2)
    J2 = np.matrix(J2)

    #Row 1 to 3, column 3
    Z_3 = [[0],
           [0],
           [1]] #0,0,1 vector

    #Row 1 to 3, column 3
    J3a = [[0,0,1],
           [1,0,0],
           [0,1,0]]
        
    J3b = [[0,0,-1],
           [1,0,0],
           [0,-1,0]] #R0_2
        
    J3c = np.dot(J3a,J3b)
    J3 = np.dot(J3c,Z_3)
    J3 = np.matrix(J3)

    #Row 4 to 6, column 1
    J4 = [[0],[0],[0]]
    J4 = np.matrix(J4)

    #Row 4 to 6, column 2
    J5 = J4
    J5 = np.matrix(J5)
        
    #Row 4 to 6, column 3
    J6 = J4
    J6 = np.matrix(J6)

    #3. Concatenated Jacobian Matrix
    JM1= np.concatenate((J1,J2,J3),1)
    JM2 = np.concatenate((J4,J5,J6),1)

    J = np.concatenate((JM1,JM2),0)
    J = np.matrix(J)


    def update_velo():
        d1p = d1_slider.get()
        d2p = d2_slider.get()
        d3p = d3_slider.get()

        q = np.array([[d1p],[d2p],[d3p]])
        E = np.dot(J,q)

        xp_e = E[0,0]
        x_entry.delete(0,END)
        x_entry.insert(0,str(xp_e))

        yp_e = E[1,0]
        y_entry.delete(0,END)
        y_entry.insert(0,str(yp_e))

        zp_e = E[2,0]
        z_entry.delete(0,END)
        z_entry.insert(0,str(zp_e))

        ωx_e = E[3,0]
        ωx_entry.delete(0,END)
        ωx_entry.insert(0,str(ωx_e))

        ωy_e = E[4,0]
        ωy_entry.delete(0,END)
        ωy_entry.insert(0,str(ωy_e))

        ωz_e = E[5,0]
        ωz_entry.delete(0,END)
        ωz_entry.insert(0,str(ωz_e))

    # Jacobian Sliders

    d1_velo = Label(J_sw,text=("d1* = "),font=(5),bg="black",fg="white") 
    d1_slider = Scale(J_sw,from_=0,to_=30,orient=HORIZONTAL,length=100,sliderlength=10,bg="black",fg="white")
    d1_unit = Label(J_sw,text=("cm/s"),font=(5),bg="black",fg="white")

    d2_velo = Label(J_sw,text=("d2* = "),font=(5),bg="black",fg="white") 
    d2_slider = Scale(J_sw,from_=0,to_=30,orient=HORIZONTAL,length=100,sliderlength=10,bg="black",fg="white")
    d2_unit = Label(J_sw,text=("cm/s"),font=(5),bg="black",fg="white")

    d3_velo = Label(J_sw,text=("d3* = "),font=(5),bg="black",fg="white") 
    d3_slider = Scale(J_sw,from_=0,to_=30,orient=HORIZONTAL,length=100,sliderlength=10,bg="black",fg="white")
    d3_unit = Label(J_sw,text=("cm/s"),font=(5),bg="black",fg="white")

    d1_velo.grid(row=0,column=0)
    d1_slider.grid(row=0,column=1)
    d1_unit.grid(row=0,column=2)

    d2_velo.grid(row=1,column=0)
    d2_slider.grid(row=1,column=1)
    d2_unit.grid(row=1,column=2)

    d3_velo.grid(row=2,column=0)
    d3_slider.grid(row=2,column=1)
    d3_unit.grid(row=2,column=2)

    # Jacobian Entries and Labels
    x_velo = Label(J_sw,text=("x* = "),font=(5),bg="black",fg="white") 
    x_entry = Entry(J_sw,width=10,font=(10))
    x_unit = Label(J_sw,text=("cm/s"),font=(5),bg="black",fg="white")
    x_velo.grid(row=3,column=0)
    x_entry.grid(row=3,column=1)
    x_unit.grid(row=3,column=2)

    y_velo = Label(J_sw,text=("y* = "),font=(5),bg="black",fg="white") 
    y_entry = Entry(J_sw,width=10,font=(10))
    y_unit = Label(J_sw,text=("cm/s"),font=(5),bg="black",fg="white")
    y_velo.grid(row=4,column=0)
    y_entry.grid(row=4,column=1)
    y_unit.grid(row=4,column=2)

    z_velo = Label(J_sw,text=("z* = "),font=(5),bg="black",fg="white") 
    z_entry = Entry(J_sw,width=10,font=(10))
    z_unit = Label(J_sw,text=("cm/s"),font=(5),bg="black",fg="white")
    z_velo.grid(row=5,column=0)
    z_entry.grid(row=5,column=1)
    z_unit.grid(row=5,column=2)

    ωx_velo = Label(J_sw,text=("ωx = "),font=(5),bg="black",fg="white") 
    ωx_entry = Entry(J_sw,width=10,font=(10))
    ωx_unit = Label(J_sw,text=("rad/s"),font=(5),bg="black",fg="white")
    ωx_velo.grid(row=6,column=0)
    ωx_entry.grid(row=6,column=1)
    ωx_unit.grid(row=6,column=2)

    ωy_velo = Label(J_sw,text=("ωy = "),font=(5),bg="black",fg="white") 
    ωy_entry = Entry(J_sw,width=10,font=(10))
    ωy_unit = Label(J_sw,text=("rad/s"),font=(5),bg="black",fg="white")
    ωy_velo.grid(row=7,column=0)
    ωy_entry.grid(row=7,column=1)
    ωy_unit.grid(row=7,column=2)

    ωz_velo = Label(J_sw,text=("ωz = "),font=(5),bg="black",fg="white") 
    ωz_entry = Entry(J_sw,width=10,font=(10))
    ωz_unit = Label(J_sw,text=("rad/s"),font=(5),bg="black",fg="white")
    ωz_velo.grid(row=8,column=0)
    ωz_entry.grid(row=8,column=1)
    ωz_unit.grid(row=8,column=2)

    # Update Button
    update_but = Button(J_sw,text="Update",bg="green",fg="white",command=update_velo)
    update_but.grid(row=9,column=0)


    # Create Links
    CARTESIAN = DHRobot([
        PrismaticDH(0,0,(270.0/180.0)*np.pi,a1/100,qlim=[0,(50/100)]),
        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0,(d1/100)]),
        PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3/100,qlim=[0,(d2/100)]),
        PrismaticDH(0,0,0,a4/100,qlim=[0,(d3/100)]),
    
    ], name = "CARTESIAN")

    #plot joints
    q1 = np.array([0,d1/100,d2/100,d3/100])


    #plot scale
    x1 = -0.2
    x2 = 0.6
    y1 = -0.2
    y2 = 0.6
    z1 = 0.0
    z2 = 0.6

    # Plot commands
    CARTESIAN.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)



def i_k():
    ### Inverse Kinematics of Cartesian

    #link lengths in mm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
    a4 = float(a4_E.get())


    #Position Vector in mm
    x0_4 = float(X_E.get())
    y0_4 = float(Y_E.get())
    z0_4 = float(Z_E.get())

    # Solution 1
    D1 = y0_4 - a2

    # Solution 2
    D2 = x0_4 - a3

    # Solution 3
    D3 = a1 - a4 - z0_4 

    d1_E.delete(0,END)
    d1_E.insert(0,np.around(D1,3))

    d2_E.delete(0,END)
    d2_E.insert(0,np.around(D2,3))

    d3_E.delete(0,END)
    d3_E.insert(0,np.around(D3,3))

    CARTESIAN = DHRobot([
        PrismaticDH(0,0,(270.0/180.0)*np.pi,a1/100,qlim=[0,(50/100)]),
        PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0,(D1/100)]),
        PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3/100,qlim=[0,(D2/100)]),
        PrismaticDH(0,0,0,a4/100,qlim=[0,(D3/100)]),
    
    ], name = "CARTESIAN")
    

    #plot joints
    q1 = np.array([0,D1/100,D2/100,D3/100])


    #plot scale
    x1 = -0.2
    x2 = 0.6
    y1 = -0.2
    y2 = 0.6
    z1 = 0.0
    z2 = 0.6    
    

    # Plot commands
    CARTESIAN.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)



# Link Lengths and Joint Variables Frame
FI = LabelFrame(gui,text="Link Lengths and Joint Variables",font=(12),bg="black",fg="violet")
FI.grid(row=0,column=0)

# The code below is creating a label frame and 
# then creating labels and entries for the link lengths.
# Link lengths
a1 = Label(FI,text=("a1 = "),font=(10),bg="black",fg="white")
a1_E = Entry(FI,width=5,font=(10))
cm1 = Label(FI,text=("cm"),font=(10),bg="black",fg="white")

a2 = Label(FI,text=("a2 = "),font=(10),bg="black",fg="white")
a2_E = Entry(FI,width=5,font=(10))
cm2 = Label(FI,text=("cm"),font=(10),bg="black",fg="white")

a3 = Label(FI,text=("a3 = "),font=(10),bg="black",fg="white")
a3_E = Entry(FI,width=5,font=(10))
cm3 = Label(FI,text=("cm"),font=(10),bg="black",fg="white")

a4 = Label(FI,text=("a4 = "),font=(10),bg="black",fg="white")
a4_E = Entry(FI,width=5,font=(10))
cm4 = Label(FI,text=("cm"),font=(10),bg="black",fg="white")

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

a4.grid(row=3,column=0)
a4_E.grid(row=3,column=1)
cm4.grid(row=3,column=2)

# The code below is creating a label frame and 
# then creating labels and entries for the Joint Variables.
# Joint Variables
d1 = Label(FI,text=("d1 = "),font=(10),bg="black",fg="white")
d1_E = Entry(FI,width=5,font=(10))
cm7 = Label(FI,text=("cm"),font=(10),bg="black",fg="white")

d2 = Label(FI,text=("d2 = "),font=(10),bg="black",fg="white")
d2_E = Entry(FI,width=5,font=(10))
cm8 = Label(FI,text=("cm"),font=(10),bg="black",fg="white")

d3 = Label(FI,text=("d3 = "),font=(10),bg="black",fg="white")
d3_E = Entry(FI,width=5,font=(10))
cm9 = Label(FI,text=("cm"),font=(10),bg="black",fg="white")

d1.grid(row=0,column=3)
d1_E.grid(row=0,column=4)
cm7.grid(row=0,column=5)

d2.grid(row=1,column=3)
d2_E.grid(row=1,column=4)
cm8.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm9.grid(row=2,column=5)

# The code below is creating a frame and buttons.
# Buttons Frame
BF = LabelFrame(gui,text="    Forward & Inverse Kinematics     ",font=(12),bg="black",fg="violet")
BF.grid(row=1,column=0)

# Buttons
FK = Button(BF,text="↓ Forward",font=(10),bg="green",fg="white",command=f_k)
rst = Button(BF,text="RESET",font=(10),bg="red",fg="white",command=reset)
IK = Button(BF,text="↑ Inverse",font=(10),bg="blue",fg="white",command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)

# The code below is creating a label frame, which is a frame that contains labels. The label frame is
# called PV, and it is placed in the gui window. The label frame is given the title "Position
# Vectors", and the font is set to 5. The label frame is placed in the second row and the first
# column.

# Position Vectors Frame
PV = LabelFrame(gui,text=" Position Vectors  ",font=(12),bg="black",fg="violet")
PV.grid(row=2,column=0)

# Position Vector
X = Label(PV,text=("X = "),font=(10),bg="black",fg="white")
X_E = Entry(PV,width=8,font=(10))
cm10 = Label(PV,text=("cm"),font=(10),bg="black",fg="white")

Y = Label(PV,text=("Y = "),font=(10),bg="black",fg="white")
Y_E = Entry(PV,width=8,font=(10))
cm11 = Label(PV,text=("cm"),font=(10),bg="black",fg="white")

Z = Label(PV,text=("Z = "),font=(10),bg="black",fg="white")
Z_E = Entry(PV,width=8,font=(10))
cm12 = Label(PV,text=("cm"),font=(10),bg="black",fg="white")

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm10.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm11.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm12.grid(row=2,column=2)

# image frame
img = PhotoImage(file="Cartesian.png")
img = img.subsample(1,2)
PI = Label(gui,image=img)
PI.grid(row=3,column=0)


gui.mainloop()
