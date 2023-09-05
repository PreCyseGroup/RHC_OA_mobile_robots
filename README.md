# RHC_OA_mobile_robots


# Waypoint Tracking with Obstacle Avoidance (WPT-OA) algorithm for mobile robots 
Code developed for the work published in "C. Tiriolo, G. Franzè, W. Lucia -  A Receding Horizon Trajectory Tracking Strategy for Input-Constrained Differential-Drive Robots via Feedback Linearization - TCST 2022"

Full paper at: https://doi.org/10.23919/ACC55779.2023.10156498


# Trajectory Tracking Problem Formulation 
Let  $p(k)=\left[x(k),y(k)\right]^T\in {\mathop{{\rm I}\mskip-4.0mu{\rm R}}\nolimits}^2$ be the planar position of a differential-drive robot at time $k$,    $p_f=[x_f,y_f]^T\in {\mathop{{\rm I}\mskip-4.0mu{\rm R}}\nolimits}^2$ the desired target location, and  $\mathcal{O}_{f}(k)$ the obstacle-free region detected by an on-board perception module at $k$.

Under the assumption that an obstacle-free path exists from $p(0)$ to $p_f,$ design a feedback control strategy
$\left[ \omega_r(k) , \omega_l(k)\right]^T =f(p(k),p_f, O_{f}(k)), \quad \forall k $

such that the robot is asymptotically driven to $p_f,$ avoiding collisions and fulfilling the velocity constraint , i.e.,

$\lim\limits_{k\rightarrow\infty} p(k) = p_f,  p(k) \in O_{f}(k)   \left[\omega_r(k),\omega_l(k)\right]^T \in U_{d} $

# Prerequisites 
The code was tested on Matlab 2020a environment and it requires a Khepera IV robot (see https://www.k-team.com/khepera-iv) to run. 
The code implements a Bluetooth client that sends velocity commands to the robot in order to make it track a desired trajectory (eight-shaped and circular tractories are currently available). For further details refer to the paper.


# File Descriptions 
- Khepera_iv_FL_RHC_traj_track.m: It is the main script to run in order to perform the experiment proposed in the paper. 
- Khepera4.m: It represents the core of the application. It is a Matlab class that implements the main communication functionalities between the tracking controller running on Matlab and the server running on Khepera IV
- STTT_control_parameters.m: It's a Matlab class defining the parameters needed by the proposed tracking controller.
- "eight_traj_generation.m" and "circular_traj_generation.m" implements the reference trajectory, an eight-shaped and a circular one, respectively.

# Demo 
- Connect KheperaIV to the host machine through Bluetooth and set the right port on the script "Khepera_iv_FL_RHC_traj_track.m".
- Run the Bluetooth server on the KheperaIV side and then, run the script Khepera_iv_FL_RHC_traj_track.m

# Videos
- https://www.youtube.com/watch?v=A0Tlbgr08tY&ab_channel=PreCySeGroup
- https://www.youtube.com/watch?v=L3wmg-pHx_4&list=PLh-6B_s-jPuT8RTDOJM96GXu4y1IBIeoC&ab_channel=PreCySeGroup
