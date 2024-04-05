# Midterms: Forward and Inverse Kinematics of Cartesian Manipulator of Group 15

**Group Members:**
 
 **Bool, Carlyn Grace M. (Project Leader)**
 
 **Del Castillo, Cherry (Project Supervisor)**
 
 **Ponsica, Sydney Ruoj (Project QA)**
 
 **Abante, Billy (Project Programmer)**

## I. Abstract of the Project

This project explores the fundamental concepts and practical applications of a Cartesian Manipulator, focusing on their kinematics through various aspects including degrees of freedom (DOF), Denavit-Hartenberg (D-H) notation, parametric tables, homogeneous transformation matrices, and inverse kinematics.


## I. Introduction of the Project

<img align="left" width="300" height="auto" src="https://github.com/billyabante/Robotics2_FK-IK_Group15_Cartesian_2024/assets/157590037/f7cb8c29-eac4-48e7-8785-64519570bc43/300/auto">

This is where the intro goes...
   
   ...
   
   ...
   
   ...
   
   ...
   ...
   
   ...
   
   ...
   
   ....
   
   ...



## Il. Degrees of Freedom of Cartesian Manipulator

**Degrees of Freedom (DOF)** - minimum number of independent parameters/variables/coordinates needed to describe a system completely.

In order to get the DOF of a Cartesian Manipulator, the **_Grubler's Formula_** was used.

<p align="center">
  <img width="460" height="auto" src="https://github.com/billyabante/Robotics2_FK-IK_Group15_Cartesian_2024/assets/157590037/b45d7805-ce31-4a46-abb5-8ec1b60be907/460/auto">
</p>

## IV. Kinematic Diagram and D-H Frame assignment of (assigned mechanical manipulator) description and computation.

**Kinematics Diagram** is a diagram that shows how the links and joints are connected together when all of the joint variables have a value of 0.

In a **Cartesian Manipulator**,there are 3 joint variables which is **PPP** or prismatic, prismatic,prismatic as shown on the image.

<p align="center">
  <img width="460" height="auto" 
src="https://github.com/billyabante/Robotics2_FK-IK_Group15_Cartesian_2024/assets/157568463/1b0aa4e1-065d-4690-861e-3d844a38415f/460/auto">

After creating the Kinematic Diagram of the Cartesian Manipulator,the next step is the assigning of the X-axis, Y-axis and Z-axis of each frame,there's a rule called D-H Frame Rules.

**D-H Frame Rules** is use to assign frames in a kinematics diagram for applying DH notation.

D-H Frame Rules:

**Rule 1:** The Z axis must be the axis of rotation for a revolute/twisting or the direction of translation for a prismatic joint.

**Rule 2:** The X axis must be perpendicular both to it's own Z axis,and the Z axis of the frame before it.

**Rule 3:** Each X axis must intersect the Z axis of ghe frame before it.
**Rules for complying Rule 3:**

• rotate the axis until it hits the other.

• or translate the axis until it hits the other.

**Rule 4:** All frames must follow the right-hand rule.

<p align="center">
  <img width="200" height="auto" 
src="https://github.com/billyabante/Robotics2_FK-IK_Group15_Cartesian_2024/assets/157568463/e8eee44f-6903-4712-85ba-5b08b09c4192/200/auto">


## V. D-H Parametric Table of (assigned mechanical manipulator) description and computation.

D-H Parametric is a shortcut for finding homogeneous transformation matrices and is commonly seen in documentation for industrial robots as well as in the research literature.
The **D-H Parametric Table** is consist of four variables:

The two variables used for rotation/orientation are **θ** and **α**.

The two variables used for position/translation are **r** and **d**.

Columns = **no. of parameters**

Rows = **no. of frames - 1**



<p align="center">
  <img width="460" height="auto" 
src="https://github.com/billyabante/Robotics2_FK-IK_Group15_Cartesian_2024/assets/157568463/b17506d8-e3ca-4ff5-9afb-56cbade1789b/460/auto">


## VI. HTM of a Cartesian Manipulator

   **Homogeneous Transformation Matrix (HTM)** is a mathematical construct used in robotics and computer graphics to represent the position and orientation of an object in space. It combines rotation and translation into a single 4x4 matrix, allowing for efficient computations and transformations of points or objects within a coordinate system.

<details>
<summary> Click to expand </summary>

The general form of a homogeneous transformation matrix ( T ) is:

 ![image](https://github.com/billyabante/Robotics2_FK-IK_Group15_Cartesian_2024/assets/157590037/72d4f66d-8c9f-4a83-ac38-1ec4d16c9279)

where:
- \( R \) is a 3x3 rotation matrix representing the orientation of the object.
- \( \mathbf{d} \) is a 3x1 translation vector representing the position of the object.
- \( \mathbf{0} \) is a 1x3 zero vector.
- The bottom-right element is always 1, which allows for the matrix to be invertible and used in various transformations.

This matrix can be used to perform operations such as rotating, translating, or scaling objects in a three-dimensional space, and it is particularly useful in the context of robotic manipulators and computer graphics modeling.

</details>

<p align="center">
  <img width="auto" height="750" src="https://github.com/billyabante/Robotics2_FK-IK_Group15_Cartesian_2024/assets/157590037/f3dd7077-0813-4a22-8f73-c2a31fff1dee/auto/750">
</p>

## VII. Inverse Kinematics of (assigned mechanical manipulator) description and computation.
<p align="center">
  <img width="460" height="auto" 
src="https://github.com/billyabante/Robotics2_FK-IK_Group15_Cartesian_2024/assets/157568463/fbb5d904-0171-49c3-8c5a-02067dca4d1b/460/auto">


## VIll. Forward and Inverse Kinematics GUl calculator of (assigned mechanical manipulator) description and computation.

 
