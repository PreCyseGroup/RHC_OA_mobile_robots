# Waypoint Tracking with Obstacle Avoidance (WPT-OA) algorithm for mobile robots 
Code developed for the work published in "C. Tiriolo, G. Franzè, W. Lucia -  An Obstacle-Avoidance Receding Horizon Control Scheme for Constrained Differential-Drive Robot via Dynamic Feedback Linearization - ACC 2023"

Full paper at: https://doi.org/10.23919/ACC55779.2023.10156498


# Waypoint Tracking with Obstacle Avoidance (WPT-OA) Problem Formulation 
Let  $p(k)=\left[x(k),y(k)\right]^T\in {\mathop{{\rm I}\mskip-4.0mu{\rm R}}\nolimits}^2$ be the planar position of a differential-drive robot at time $k$,    $p_f=[x_f,y_f]^T\in {\mathop{{\rm I}\mskip-4.0mu{\rm R}}\nolimits}^2$ the desired target location, and  $\mathcal{O}_{f}(k)$ the obstacle-free region detected by an on-board perception module at $k$.

Under the assumption that an obstacle-free path exists from $p(0)$ to $p_f,$ design a feedback control strategy
$\left[ \omega_r(k) , \omega_l(k)\right]^T =f(p(k),p_f, O_{f}(k)), \quad \forall k $

such that the robot is asymptotically driven to $p_f,$ avoiding collisions and fulfilling the velocity constraint , i.e.,

$\lim\limits_{k\rightarrow\infty} p(k) = p_f,  p(k) \in O_{f}(k)   \left[\omega_r(k),\omega_l(k)\right]^T \in U_{d} $

# Prerequisites 
The code was tested on Matlab 2020a environment and it requires Ellipsoidal Toolbox ET (https://www.mathworks.com/matlabcentral/fileexchange/21936-ellipsoidal-toolbox-et) and Navigation Toolbox (https://www.mathworks.com/products/navigation.html). 
The code simulates the Receding Horizon Control-based  obstacle avoidance strategy for mobile robots developed in the above paper (refer to the full paper for further details).

# File Descriptions 
- ST_Dynamic_FL_RHC_OA.m: It is the main script to run, which implements and simulates the obstacle avoidance strategy.  
- DiffDrive.m: It's a Matlab function that implements the mobile robot kinematics. It's used by the ode45 method to solve the differential equations describing the robot's motion in the plane. 
- dyn_FL_kothare_constraint_OA.m: It's a Matlab function implementing the LMI-based receding horizon control law used to solve the considered WPT-OA Problem formulated above. 

# Demo 
- Install the required toolboxes detailed in the Prerequisites section of this file.
- Run the script ST_Dynamic_FL_RHC_OA.m
