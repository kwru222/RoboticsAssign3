# RoboticsAssign3
My Submission for the third assignment for ME 699: Robotics at the University of Kentucky

submitted by: Keith Russell


## Usage
### 1) Clone respoitory
```
cd home/...../parent_directory/
git clone
```

### 2) Run Main Program
```
cd project_folder/
julia
include("main.jl")
```

### 3) Program Functionality
#### There are multiple ways this program can be operated from the terminal:
#### 1. Trajectory Generation
The first function in this program is the trajectory generator.  It takes a time (in seconds) as input and returns three vectors corresponding to the desired position, velocity, and acceleration of the manipulator at that time.  It is operated using the following command from the terminal:

```
Traj(t)
```

Where t is the time in seconds.

#### 2) Control
The second functionality of this program is the manipulator controller.  This function takes a controller type as input and computes the control of the manipulator based on that input.  There are two controller types to choose from: a PD controller and a CTC controller that uses a combination of PD control with feedforward control.  Usage is as follows:

```
controller(Control_PD!) #to implement the PD controller

controller(Control_CTC!) #to implement the CTC controller
```
Once the command with desired control is entered into the terminal, a window will be opened to visualize the resulting control of the Panda manipulator.  The 2 norm of the manipulator joint angle configuration will also be printed in the terminal.  Note: gain values for PD and CTC controllers have been adjusted to achieve a 2 norm of less than .01.

It was found that the proportional gain could be lowered to 20 and retain the 2 norm limit of configuration error of .01.


## Credit
I would like to aknowledge the sources that helped me create this code.  

First and foremost, Dr. Poonawala's Panda.jl code helped give a basic understanding of controller implementation and visualization. https://github.com/hpoonawala/rmc-s21/blob/master/julia/odes/panda.jl

Secondly, I would like to aknowledge the textbook Modern Robotics by Kevin M. Lynch and Frank C. Park, as it helped greatly in designing the trajectory and control CTC functions.  In particular, equations 9.11, 9.12, 9.13 and 11.35. http://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf

