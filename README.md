 Project Title

Real-Time 3D Passive Object Tracking & Error Minimization System (Ground-to-Air)




Overview

This project focuses on designing a real-time passive tracking system capable of estimating the 3D position of airborne objects using angular measurements from multiple ground stations. The system improves tracking accuracy by minimizing estimation errors through advanced filtering and mathematical modeling techniques.





Key Contributions

1)Developed a real-time 3D tracking model using angular inputs (azimuth & elevation) from multiple    observation stations.
2)Implemented multi-station data fusion to improve localization accuracy.
3)Applied Kalman Filtering for continuous state estimation and noise reduction.
4)Designed an error minimization pipeline using decomposition-based optimization and Kalman filter  techniques.
5)Built a simulation framework to evaluate tracking performance under dynamic conditions.



🔹 Technical Approach

The system follows a structured multi-stage pipeline to estimate the 3D position of a target using only angular measurements from two passive Electro-Optical Tracking System (EOTS) stations.

1. Data Acquisition & Synchronization
Collected azimuth and elevation measurements from two spatially separated stations.
Synchronized both station data streams using timestamps to ensure time-aligned observations.
2. Line-of-Sight (LOS) Modeling
Converted angular measurements into unit direction vectors (LOS vectors) in 3D space.
Each station provides a directional constraint toward the target without range information.
3. Linear Least-Squares Triangulation
Estimated initial 3D position by solving the intersection of two LOS vectors.
Formulated as a least-squares optimization problem to handle non-intersecting rays due to noise.
Used:
Cholesky Decomposition → for fast computation (well-conditioned cases)
Pseudo-inverse → for stability (ill-conditioned geometry, small apex angle)
4. Residual-Based Quality Check
Computed residual error of the linear solution.
If error exceeds a threshold → indicates poor geometry or noise impact.
5. Nonlinear Least-Squares Refinement
Applied nonlinear optimization only when needed (adaptive approach).
Minimized geometric reprojection error between estimated position and LOS constraints.
Used weighted residuals to improve robustness against distance and noise variations.
6. Temporal Smoothing
Applied exponential smoothing to reduce frame-to-frame fluctuations.
Ensures smoother trajectory before filtering.
7. Kalman Filtering (State Estimation)
Modeled target motion using a constant velocity model.
Used Kalman Filter to:
Reduce measurement noise
Maintain temporal consistency
Predict and correct target trajectory in real time
8. Performance Evaluation
Evaluated system using:
3D Euclidean Error
Horizontal & Vertical Error
RMSE (Root Mean Square Error)
Analyzed impact of sensor geometry (apex angle) on tracking accuracy.



 Results & Performance  

- Nonlinear refinement significantly reduced geometric error compared to linear triangulation.  
- Kalman filtering improved trajectory smoothness and temporal consistency.  
- System performance was evaluated using RMSE, horizontal error, and vertical error metrics.  
- Tracking accuracy was strongly dependent on sensor geometry (apex angle).  

Overall, the hybrid approach achieved stable and accurate 3D tracking even under noisy conditions.


System Architecture / Workflow
1. Input angular data (Azimuth, Elevation) from two stations  
2. Convert to Line-of-Sight vectors  
3. Perform linear triangulation  
4. Check residual error  
5. Apply nonlinear refinement (if required)  
6. Apply smoothing + Kalman filtering  
7. Output final 3D trajectory


 ![image alt](https://github.com/KaushalTheprogrammer/Real-Time-3D-Passive-Object-Tracking-Simulation/blob/0275be5ffcb800ff7f618807893e6fb01bbe6407/Screenshot%202026-04-14%20213730.png)





5. Applications

- Defense surveillance and tracking systems  
- Radar-less target detection  
- Air traffic monitoring  
- Autonomous defense systems 


