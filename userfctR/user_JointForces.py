# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


#Importation :

import sys
sys.path.insert(0, "/Users/matthieuxaussems/Documents/MBProjects/Modele_V4/User_function")

import Muscle_actuation_layer as muscle
import Neural_control_layer as neural
import useful_functions as u_f
import numpy as np
import MBsysPy as Robotran

#Indexes :

VAS =0
SOL =1
GAS = 2
TA = 3
HAM = 4
GLU = 5
HFL = 6

ankle = 0 
knee = 1
hip = 2

#Initial parameters :

time = np.zeros(1)
n_muscle = 7  # Number of muscles
n_articulation = 3 # number of articulations

ground_limit = 2.0

#Memories :

Act_L = np.zeros(n_muscle) # Memory of motor activations at different time steps for the left leg.
Act_R = np.zeros(n_muscle) # Memory of motor activations at different time steps for the right leg.

Stim_L = np.zeros(n_muscle) # Memory of motor stimulations at different time steps for the left leg.
Stim_R = np.zeros(n_muscle) # Memory of motor stimulations at different time steps for the right leg.

Fm_memoryL = np.zeros(n_muscle) # Memory of motor forces at different time steps for the muscles of the left leg.
Fm_memoryR = np.zeros(n_muscle) # Memory of motor forces at different time steps for the muscles of the right leg.

Ldx_memory = np.zeros(1) # Memory of the displacement of the inner left thigh soft joint 
Rdx_memory = np.zeros(1) # Memory of the displacement of the inner right thigh soft joint 

StanceL_memory = False # Memory of the stance state of the left leg
StanceR_memory = False # Memory of the stance state of the right leg

lce_memoryL = np.zeros(n_muscle) #Memory of the contractile element length of the muscles of the left leg
lce_memoryR = np.zeros(n_muscle) #Memory of the contractile element length of the muscles of the right leg

theta_knee_memoryL = np.zeros(1) #Memory of the left knee angle 
dtheta_knee_memoryL = np.zeros(1) #Memory of the right knee angular velocity
theta_knee_memoryR = np.zeros(1) #Memory of the right knee angle 
dtheta_knee_memoryR = np.zeros(1) #Memory of the right knee angular velocity

theta_trunk_memory = np.zeros(1) #Memory of the orientation of the trunk with regards to the vertical 
dtheta_trunk_memory = np.zeros(1) #Memory of the angular velocity of the trunk 



def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    #sensors index :
    
    id_BallL = mbs_data.sensor_id["Sensor_BallL"]
    id_HeelL = mbs_data.sensor_id["Sensor_HeelL"]
    id_BallR = mbs_data.sensor_id["Sensor_BallR"]
    id_HeelR = mbs_data.sensor_id["Sensor_HeelR"]
    id_trunk = mbs_data.sensor_id["Sensor_trunk"]
    id_hip = mbs_data.sensor_id["Sensor_hip"]
    
    
    # # Joints id :
    
    id_ankleL = mbs_data.joint_id["ankleL"]
    id_ankleR = mbs_data.joint_id["ankleR"]
    id_kneeL = mbs_data.joint_id["kneeL"]
    id_kneeR = mbs_data.joint_id["kneeR"]
    id_hipL = mbs_data.joint_id["hipL"]
    id_hipR = mbs_data.joint_id["hipR"]
    
    id_thighloadL = mbs_data.joint_id["thighloadL"]
    id_thighloadR = mbs_data.joint_id["thighloadR"]

    # #compute of the time step :
    
    global time
    dt= np.round(tsim - time[-1], decimals=4)
    time = np.append(time,tsim)
    
    print("tsim : ",tsim)
    print("diff_t : ",dt)
    
    # #Global variables (memories) :
    
    global Act_L
    global Act_R
    
    global Stim_L
    global Stim_R
    
    global Fm_memoryL
    global Fm_memoryR
    
    global lce_memoryL
    global lce_memoryR
    
    global Ldx_memory
    global Rdx_memory
    
    global StanceL_memory
    global StanceR_memory
    
    global theta_knee_memoryL
    global dtheta_knee_memoryL
    global theta_knee_memoryR
    global dtheta_knee_memoryR
    
    global theta_trunk_memory
    global dtheta_trunk_memory
    
    
    # ##### (1) The first step, since the stimulation of the muscles of a leg depends on whether that same leg is in stance or in swing,
    # #is to detect which leg is in stance or not. 
    
    # # We first compute a boolean vector which indicates which sensors are in contact with the ground :
    
    Boolean_vector= neural.Boolean_sensors(mbs_data.sensors[id_BallL].P[3], mbs_data.sensors[id_HeelL].P[3],
                                  mbs_data.sensors[id_BallR].P[3], mbs_data.sensors[id_HeelR].P[3],ground_limit)
    
    # # Then, from the boolean vector we can compute the stance state : (True if Left leg in stance, true if right leg in stance, 
    # #true if double support) and we create a memory because we have a delay of the afferent signal coming from Ball and Heel sensors :
    
    Stance_state = neural.stance_swing(Boolean_vector[0], Boolean_vector[1], Boolean_vector[2], Boolean_vector[3])
    
    
    # ##### (2) Update of the memories (vectors) of the displacement of the inner thigh joint,stance states, the knee angle and the trunk orientation 
    # #because we use them with a delay :
    
    
    if tsim ==0 :
        
        Ldx_memory = mbs_data.q[id_thighloadL]
        Rdx_memory = mbs_data.q[id_thighloadR]
        StanceL_memory = [Stance_state[0]]
        StanceR_memory = [Stance_state[1]]
        theta_knee_memoryL = muscle.model_angle(knee, mbs_data.q[id_kneeL])
        dtheta_knee_memoryL = mbs_data.qd[id_kneeL]
        theta_knee_memoryR = muscle.model_angle(knee, mbs_data.q[id_kneeR])
        dtheta_knee_memoryR = mbs_data.qd[id_kneeR]
        theta_trunk_memory = np.array([u_f.trunk_angle(mbs_data.sensors[id_hip].P, mbs_data.sensors[id_trunk].P,tsim)])
        dtheta_trunk_memory = 0
        
    else :
        
        if dt != 0 :
            
            Ldx_memory= np.hstack([Ldx_memory,mbs_data.q[id_thighloadL]])
            Rdx_memory= np.hstack([Rdx_memory,mbs_data.q[id_thighloadR]])
            StanceL_memory.append(Stance_state[0])
            StanceR_memory.append(Stance_state[1])
            theta_knee_memoryL = np.hstack([theta_knee_memoryL, muscle.model_angle(knee, mbs_data.q[id_kneeL])])
            dtheta_knee_memoryL = np.hstack([dtheta_knee_memoryL, mbs_data.qd[id_kneeL]])
            theta_knee_memoryR = np.hstack([theta_knee_memoryR, muscle.model_angle(knee, mbs_data.q[id_kneeR])])
            dtheta_knee_memoryR = np.hstack([dtheta_knee_memoryR, mbs_data.qd[id_kneeR]])
            
            if len(theta_trunk_memory) == 1 :
                dtheta_trunk_memory = np.hstack([dtheta_trunk_memory, u_f.dt_angle(u_f.trunk_angle(mbs_data.sensors[id_hip].P, mbs_data.sensors[id_trunk].P,tsim),theta_trunk_memory, dt)])
            else :
                dtheta_trunk_memory = np.hstack([dtheta_trunk_memory, u_f.dt_angle(u_f.trunk_angle(mbs_data.sensors[id_hip].P, mbs_data.sensors[id_trunk].P,tsim),theta_trunk_memory[-1], dt)])
            
            theta_trunk_memory = np.hstack([theta_trunk_memory, u_f.trunk_angle(mbs_data.sensors[id_hip].P, mbs_data.sensors[id_trunk].P,tsim)])
    
    # ##### (3) Once we have updated these memories, we can calculate the stimulation of the muscles of the left leg and the stimulation of the muscles 
    # # of the right leg. At time 0 we don't have any motor force and contractile element lenght calculated yet, but that's okay because in the calculation of the stimulation 
    # # at time 0 we only need the pre-stimulation of the muscle.  The stimuli are calculated with a delay, so the stimulation depends on the 
    # # variables of a few time steps before. That's why we need memories matrix and vectors.
    
    # #We find wich leg is leading if there is double support (we have a mid delay) :
        
    if (tsim-0.01)<0 : 
        
        LonR=0
        RonL=0
    
    else :
    
        LonR = neural.lead(StanceL_memory[:int(round(tsim-0.01))], StanceR_memory[:int(round(tsim-0.01))])[0]
        RonL = neural.lead(StanceL_memory[:int(round(tsim-0.01))], StanceR_memory[:int(round(tsim-0.01))])[1]
    
    # #Computation of the stimulation :   
    
    if tsim == 0 :
        
        Stim_L = neural.Feedback(StanceL_memory, Fm_memoryL, Ldx_memory , Rdx_memory, RonL, lce_memoryL, theta_knee_memoryL, dtheta_knee_memoryL, theta_trunk_memory, dtheta_trunk_memory, tsim, 0)
        Stim_R = neural.Feedback(StanceR_memory, Fm_memoryR, Rdx_memory , Ldx_memory, LonR, lce_memoryR, theta_knee_memoryR, dtheta_knee_memoryR, theta_trunk_memory, dtheta_trunk_memory, tsim, 0)
    
    else :
        
        if dt == 0 :
            
            Stim_L = Stim_L
            Stim_R = Stim_R
        
        else :
            
            Stim_L = np.vstack([Stim_L,neural.Feedback(StanceL_memory, Fm_memoryL, Ldx_memory , Rdx_memory, RonL, lce_memoryL, theta_knee_memoryL, dtheta_knee_memoryL, theta_trunk_memory, dtheta_trunk_memory, tsim, dt)])
            Stim_R = np.vstack([Stim_R,neural.Feedback(StanceR_memory, Fm_memoryR, Rdx_memory , Ldx_memory, LonR, lce_memoryR, theta_knee_memoryR, dtheta_knee_memoryR, theta_trunk_memory, dtheta_trunk_memory, tsim, dt)])


    # ##### (4) From the stimulations we can calculate the activation of the muscles. This relation is characterized by a differential equation 
    # #describing the excitation-contraction coupling.  This equation is solved by using a low pass filter :
    
    tau = 0.01 #time constant of the differential equation 
    
    if tsim == 0 :
        
        Act_L = u_f.low_filter(Stim_L, tau, 0,0)
        Act_R = u_f.low_filter(Stim_R, tau, 0,0)
    
    else :
        
        if dt == 0:
            
            Act_L = Act_L
            Act_R = Act_R
            
        else :
            if len(np.shape(Act_L)) == 1 :
                
                Act_L = np.vstack([Act_L,u_f.low_filter(Stim_L[-1], tau, dt,Act_L)])
                Act_R = np.vstack([Act_R,u_f.low_filter(Stim_R[-1], tau, dt,Act_R)])
            else :
                
                Act_L = np.vstack([Act_L,u_f.low_filter(Stim_L[-1], tau, dt,Act_L[-1])])
                Act_R = np.vstack([Act_R,u_f.low_filter(Stim_R[-1], tau, dt,Act_R[-1])])
    
    
    # ##### (5) At each iteration, we update the length of the muscle unit from the angle of the joint.
    
    for i in range(n_muscle):
        if i == GAS :
            
            muscle.lmtu_update(muscle.model_angle(ankle, mbs_data.q[id_ankleL]), muscle.model_angle(knee, mbs_data.q[id_kneeL]), i, 0)
            muscle.lmtu_update(muscle.model_angle(ankle, mbs_data.q[id_ankleR]), muscle.model_angle(knee, mbs_data.q[id_kneeR]), i, 1)
        
        elif i == HAM :
            
            muscle.lmtu_update(muscle.model_angle(knee, mbs_data.q[id_kneeL]), muscle.model_angle(hip, mbs_data.q[id_hipL]), i, 0)
            muscle.lmtu_update(muscle.model_angle(knee, mbs_data.q[id_kneeR]), muscle.model_angle(hip, mbs_data.q[id_hipR]), i, 1)
        
        elif i == HFL or i == GLU  :
            
            muscle.lmtu_update(muscle.model_angle(hip, mbs_data.q[id_hipL]), 0, i, 0)
            muscle.lmtu_update(muscle.model_angle(hip, mbs_data.q[id_hipR]), 0, i, 1)
        
        elif i == VAS :
            
            muscle.lmtu_update(muscle.model_angle(knee, mbs_data.q[id_kneeL]), 0, i, 0)
            muscle.lmtu_update(muscle.model_angle(knee, mbs_data.q[id_kneeR]), 0, i, 1)
        
        elif i == SOL or i == TA :
            
            muscle.lmtu_update(muscle.model_angle(ankle, mbs_data.q[id_ankleL]), 0, i, 0)
            muscle.lmtu_update(muscle.model_angle(ankle, mbs_data.q[id_ankleR]), 0, i, 1)
    
    # ##### (6) We can update the torques. Each torque corresponds to the torque involved in a specific muscle on a specific articulation joint.
    
    for j in range(n_articulation):
        for i in range(n_muscle):
            if len(np.shape(Fm_memoryL)) == 1 :
            
                if j == ankle :
            
                    muscle.torque_update(muscle.model_angle(ankle, mbs_data.q[id_ankleL]), Fm_memoryL, j, i, 0)
                    muscle.torque_update(muscle.model_angle(ankle, mbs_data.q[id_ankleR]), Fm_memoryR, j, i, 1)
                
                elif j == knee : 
                    
                    muscle.torque_update(muscle.model_angle(knee, mbs_data.q[id_kneeL]), Fm_memoryL, j, i, 0)
                    muscle.torque_update(muscle.model_angle(knee, mbs_data.q[id_kneeR]), Fm_memoryR, j, i, 1)
                
                elif j == hip :
                    
                    muscle.torque_update(muscle.model_angle(hip, mbs_data.q[id_hipL]), Fm_memoryL, j, i, 0)
                    muscle.torque_update(muscle.model_angle(hip, mbs_data.q[id_hipR]), Fm_memoryR, j, i, 1)
            
            else : 
                
                if j == ankle :
            
                    muscle.torque_update(muscle.model_angle(ankle, mbs_data.q[id_ankleL]), Fm_memoryL[-1], j, i, 0)
                    muscle.torque_update(muscle.model_angle(ankle, mbs_data.q[id_ankleR]), Fm_memoryR[-1], j, i, 1)
                
                elif j == knee : 
                    
                    muscle.torque_update(muscle.model_angle(knee, mbs_data.q[id_kneeL]), Fm_memoryL[-1], j, i, 0)
                    muscle.torque_update(muscle.model_angle(knee, mbs_data.q[id_kneeR]), Fm_memoryR[-1], j, i, 1)
                
                elif j == hip :
                    
                    muscle.torque_update(muscle.model_angle(hip, mbs_data.q[id_hipL]), Fm_memoryL[-1], j, i, 0)
                    muscle.torque_update(muscle.model_angle(hip, mbs_data.q[id_hipR]), Fm_memoryR[-1], j, i, 1)
    
    # ##### (7) Update of Fm and Lce memory. Once we have the muscle activations and that we have updated the length of the muscle unit, 
    # #we can obtain the motor forces that result from these muscle activations. 
    # #For this, the hill model is used, which will allow to do two things, an update of the length of the contractile element 
    # #but also to obtain the motor force developed by the muscles.
    
    Fm_newL = np.zeros(n_muscle)
    Fm_newR = np.zeros(n_muscle)
    lce_newL = np.zeros(n_muscle)
    lce_newR = np.zeros(n_muscle)
    
    
    
    for i in range(n_muscle): #allows to obtain the muscular force for each of the muscles
        
        if len(np.shape(Act_L)) == 1 :
            
            Fm_newL[i] = muscle.EulerIterations(dt, 5, i, Act_L[i], 0, tsim)[0]
            Fm_newR[i] = muscle.EulerIterations(dt, 5, i, Act_R[i], 1, tsim)[0]
            lce_newL[i] = muscle.EulerIterations(dt, 5, i, Act_L[i], 0, tsim)[1]
            lce_newR[i] = muscle.EulerIterations(dt, 5, i, Act_R[i], 1, tsim)[1]
        else :
        
            Fm_newL[i] = muscle.EulerIterations(dt, 5, i, Act_L[-1,i], 0, tsim)[0]
            Fm_newR[i] = muscle.EulerIterations(dt, 5, i, Act_R[-1,i], 1, tsim)[0]
            lce_newL[i] = muscle.EulerIterations(dt, 5, i, Act_L[-1,i], 0, tsim)[1]
            lce_newR[i] = muscle.EulerIterations(dt, 5, i, Act_R[-1,i], 1, tsim)[1]
    
    if tsim == 0 :
        
        Fm_memoryL = Fm_newL
        Fm_memoryR = Fm_newR
        lce_memoryL = lce_newL
        lce_memoryR = lce_newR
        
        
    else : 
        
        if dt != 0 :
            
            Fm_memoryL = np.vstack([Fm_memoryL,Fm_newL])
            Fm_memoryR = np.vstack([Fm_memoryR,Fm_newR])
            lce_memoryL = np.vstack([lce_memoryL,lce_newL])
            lce_memoryR = np.vstack([lce_memoryR,lce_newR])
    
    
    ##### (8) Thanks to the torques update we can calculate the total torque on a joint and we can apply these total torques on our joints with 
    # robotran.
    

    
    mbs_data.Qq[id_ankleL] = muscle.torque_compute(ankle, 0, muscle.model_angle(ankle, mbs_data.q[id_ankleL]), - mbs_data.qd[id_ankleL])
    mbs_data.Qq[id_ankleR] = muscle.torque_compute(ankle, 1, muscle.model_angle(ankle, mbs_data.q[id_ankleR]), - mbs_data.qd[id_ankleR])
    mbs_data.Qq[id_kneeL] = muscle.torque_compute(knee, 0, muscle.model_angle(knee, mbs_data.q[id_kneeL]), mbs_data.qd[id_kneeL])
    mbs_data.Qq[id_kneeR] = muscle.torque_compute(knee, 1, muscle.model_angle(knee, mbs_data.q[id_kneeR]), mbs_data.qd[id_kneeR])
    mbs_data.Qq[id_hipL] = muscle.torque_compute(hip, 0, muscle.model_angle(hip, mbs_data.q[id_hipL]), - mbs_data.qd[id_hipL])
    mbs_data.Qq[id_hipR] = muscle.torque_compute(hip, 1, muscle.model_angle(hip, mbs_data.q[id_hipR]), - mbs_data.qd[id_hipR])
    
    # #We can also compute the force on the inner thigh shift joint
    
    mbs_data.Qq[id_thighloadL] = u_f.pressure_sheet(mbs_data.q[id_thighloadL],mbs_data.qd[id_thighloadL])
    mbs_data.Qq[id_thighloadR] = u_f.pressure_sheet(mbs_data.q[id_thighloadR],mbs_data.qd[id_thighloadR])
     
    
    return
