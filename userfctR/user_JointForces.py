# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


#Importation :

import sys
sys.path.insert(0, "/Users/matthieuxaussems/Documents/MBProjects/Modele_V4_copie_ni/User_function")

import Muscle_actuation_layer_copie as muscle
import Neural_control_layer_copie as neural
import useful_functions_copie as u_f
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

ground_limit = 1.79503-2.332694446494088e-06

#Memories :

Act_L = np.zeros(n_muscle+1) # Memory of motor activations at different time steps for the left leg.
Act_R = np.zeros(n_muscle+1) # Memory of motor activations at different time steps for the right leg.

Stim_L = np.zeros(n_muscle+1) # Memory of motor stimulations at different time steps for the left leg.
Stim_R = np.zeros(n_muscle+1) # Memory of motor stimulations at different time steps for the right leg.

Fm_memoryL = np.zeros(n_muscle+1) # Memory of motor forces at different time steps for the muscles of the left leg.
Fm_memoryR = np.zeros(n_muscle+1) # Memory of motor forces at different time steps for the muscles of the right leg.

Ldx_memory = np.zeros(2) # Memory of the displacement of the inner left thigh soft joint 
Rdx_memory = np.zeros(2) # Memory of the displacement of the inner right thigh soft joint 

StanceL_memory = np.zeros((1,2),dtype = object) # Memory of the stance state of the left leg
StanceL_memory[0,0] = True
StanceL_memory[0,1] = 0
 
StanceR_memory = np.zeros((1,2),dtype = object) # Memory of the stance state of the right leg
StanceR_memory[0,0] = False
StanceR_memory[0,1] = 0

lmtu_memoryL = np.zeros(n_muscle+1) #memory of the muscle-tendon complex length of the muscles of the left leg
lmtu_memoryR = np.zeros(n_muscle+1) #memory of the muscle-tendon complex length of the muscles of the right leg

torque_memoryL = np.zeros((3,n_muscle+1)) # memory of the torque left leg
torque_memoryR = np.zeros((3,n_muscle+1)) # memory of the torque right leg

lce_memoryL = np.zeros(n_muscle+1) #[m] #Memory of the contractile element length of the muscles of the left leg
lce_memoryR =np.zeros(n_muscle+1) #[m] #Memory of the contractile element length of the muscles of the right leg

vce_memoryL = np.zeros(n_muscle+1) #Memory of the contractile element speed of the muscles of the left leg
vce_memoryR = np.zeros(n_muscle+1) #Memory of the contractile element speed of the muscles of the right leg

theta_knee_memoryL = np.zeros(2) #Memory of the left knee angle 
dtheta_knee_memoryL = np.zeros(2) #Memory of the right knee angular velocity
theta_knee_memoryR = np.zeros(2) #Memory of the right knee angle 
dtheta_knee_memoryR = np.zeros(2) #Memory of the right knee angular velocity

theta_trunk_memory = np.zeros(2) #Memory of the orientation of the trunk with regards to the vertical 
dtheta_trunk_memory = np.zeros(2) #Memory of the angular velocity of the trunk 


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
    
    
    # Joints id :
    
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
    dt= tsim - time[-1]
    time = np.append(time,tsim)
    
    print("tsim : ",tsim)
    #print("diff_t : ",dt)
    
    # #Global variables (memories) :
        
    
    global Act_L
    global Act_R
    
    global Stim_L
    global Stim_R
    
    global Fm_memoryL
    global Fm_memoryR
    
    global lmtu_memoryL
    global lmtu_memoryR
    
    global torque_memoryL
    global torque_memoryR
    
    global lce_memoryL
    global lce_memoryR
    global vce_memoryL
    global vce_memoryR
    
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
    
    if dt!=0 or tsim ==0 : 
    
        # ##### (1) The first step, since the stimulation of the muscles of a leg depends on whether that same leg is in stance or in swing,
        # #is to detect which leg is in stance or not. 
        
        # # We first compute a boolean vector which indicates which sensors are in contact with the ground :
        
        Boolean_vector= neural.Boolean_sensors(mbs_data.sensors[id_BallL].P[3], mbs_data.sensors[id_HeelL].P[3],
                                      mbs_data.sensors[id_BallR].P[3], mbs_data.sensors[id_HeelR].P[3],ground_limit)
        
        
        # # Then, from the boolean vector we can compute the stance state : (True if Left leg in stance, true if right leg in stance, 
        # #true if double support) and we create a memory because we have a delay of the afferent signal coming from Ball and Heel sensors :
        
        Stance_state = neural.stance_swing(Boolean_vector[0], Boolean_vector[1], Boolean_vector[2], Boolean_vector[3])
        
        
        # # ##### (2) Update of the memories (vectors) of the displacement of the inner thigh joint,stance states, the knee angle and the trunk orientation 
        # # #because we use them with a delay :
        
        
        if tsim ==0 :
            
            Ldx_memory = np.hstack([mbs_data.q[id_thighloadL],tsim])
            Rdx_memory = np.hstack([mbs_data.q[id_thighloadR],tsim])
            StanceL_memory[0,0] = True
            StanceL_memory[0,1] = tsim
            StanceR_memory[0,0] = False
            StanceR_memory[0,1] = tsim
            theta_knee_memoryL = np.hstack([muscle.model_angle(knee, mbs_data.q[id_kneeL]),tsim])
            dtheta_knee_memoryL = np.hstack([mbs_data.qd[id_kneeL],tsim])
            theta_knee_memoryR = np.hstack([muscle.model_angle(knee, mbs_data.q[id_kneeR]),tsim])
            dtheta_knee_memoryR = np.hstack([mbs_data.qd[id_kneeR],tsim])
            theta_trunk_memory = np.hstack([u_f.trunk_angle(mbs_data.sensors[id_hip].P, mbs_data.sensors[id_trunk].P,tsim),tsim])
            dtheta_trunk_memory = np.hstack([0,tsim])
            
        else :
            
            Ldx_memory_new= np.hstack([mbs_data.q[id_thighloadL],tsim])
            Ldx_memory = np.vstack([Ldx_memory,Ldx_memory_new])
            Ldx_memory = u_f.pop_malone(Ldx_memory)
            
            # à voir si on ajoute ou non
            # if len(np.shape(Ldx_memory)) !=1 :
            #     Ldx_memory= u_f.filter_signal(Ldx_memory, [1], [0.02,1])
            
                        
            Rdx_memory_new = np.hstack([mbs_data.q[id_thighloadR],tsim])
            Rdx_memory= np.vstack([Rdx_memory,Rdx_memory_new])
            Rdx_memory = u_f.pop_malone(Rdx_memory)
            
            # if len(np.shape(Ldx_memory)) !=0 :
            #     Rdx_memory= u_f.filter_signal(Rdx_memory, [1], [0.02,1])


            StanceL_memory_new = np.zeros((1,2),dtype=object)
            StanceL_memory_new[0,0] = Stance_state[0]
            StanceL_memory_new[0,1] = tsim
            StanceL_memory = np.vstack([StanceL_memory,StanceL_memory_new])
            StanceL_memory = u_f.pop_malone(StanceL_memory)
    
            StanceR_memory_new = np.zeros((1,2),dtype=object)
            StanceR_memory_new[0,0] = Stance_state[1]
            StanceR_memory_new[0,1] = tsim
            StanceR_memory = np.vstack([StanceR_memory,StanceR_memory_new])
            StanceR_memory = u_f.pop_malone(StanceR_memory)

            theta_knee_memoryL_new = np.hstack([muscle.model_angle(knee, mbs_data.q[id_kneeL]),tsim])
            theta_knee_memoryL = np.vstack([theta_knee_memoryL, theta_knee_memoryL_new])
            theta_knee_memoryL = u_f.pop_malone(theta_knee_memoryL)
            
            dtheta_knee_memoryL_new = np.hstack([mbs_data.qd[id_kneeL],tsim])
            dtheta_knee_memoryL = np.vstack([dtheta_knee_memoryL,dtheta_knee_memoryL_new])
            dtheta_knee_memoryL = u_f.pop_malone(dtheta_knee_memoryL)
            
            theta_knee_memoryR_new = np.hstack([muscle.model_angle(knee, mbs_data.q[id_kneeR]),tsim])
            theta_knee_memoryR = np.vstack([theta_knee_memoryR, theta_knee_memoryR_new])
            theta_knee_memoryR = u_f.pop_malone(theta_knee_memoryR)
            
            dtheta_knee_memoryR_new = np.hstack([mbs_data.qd[id_kneeR],tsim])
            dtheta_knee_memoryR = np.vstack([dtheta_knee_memoryR,dtheta_knee_memoryR_new])
            dtheta_knee_memoryR = u_f.pop_malone(dtheta_knee_memoryR)
          
            
            if len(np.shape(theta_trunk_memory)) == 1 :
                dtheta_trunk_memory_new = np.hstack([ u_f.dt_angle(u_f.trunk_angle(mbs_data.sensors[id_hip].P, mbs_data.sensors[id_trunk].P,tsim),theta_trunk_memory[0], dt),tsim])
                dtheta_trunk_memory = np.vstack([dtheta_trunk_memory,dtheta_trunk_memory_new])
            else :
                dtheta_trunk_memory_new = np.hstack([ u_f.dt_angle(u_f.trunk_angle(mbs_data.sensors[id_hip].P, mbs_data.sensors[id_trunk].P,tsim),theta_trunk_memory[-1,0], dt),tsim])
                dtheta_trunk_memory = np.vstack([dtheta_trunk_memory,dtheta_trunk_memory_new])
                dtheta_trunk_memory = u_f.pop_malone(dtheta_trunk_memory)
            
            
            theta_trunk_memory_new = np.hstack([u_f.trunk_angle(mbs_data.sensors[id_hip].P, mbs_data.sensors[id_trunk].P,tsim),tsim])
            theta_trunk_memory = np.vstack([theta_trunk_memory, theta_trunk_memory_new])
            theta_trunk_memory = u_f.pop_malone(theta_trunk_memory)
           
            print(StanceL_memory[-1])
        
        # ##### (3) Once we have updated these memories, we can calculate the stimulation of the muscles of the left leg and the stimulation of the muscles 
        # # of the right leg. At time 0 we don't have any motor force and contractile element lenght calculated yet, but that's okay because in the calculation of the stimulation 
        # # at time 0 we only need the pre-stimulation of the muscle.  The stimuli are calculated with a delay, so the stimulation depends on the 
        # # variables of a few time steps before. That's why we need memories matrix and vectors.
        
        # #We find wich leg is leading if there is double support (we have a mid delay) :
            
        if (tsim-0.01)<0 : 
            
            LonR=0
            RonL=0
        
        else :
        
            LonR = neural.lead(StanceL_memory, StanceR_memory,tsim)[0]
            RonL = neural.lead(StanceL_memory, StanceR_memory,tsim)[1]
        
        # # #Computation of the stimulation :   
        
        Stim_L = neural.Feedback(tsim, dt,StanceL_memory, Fm_memoryL, Ldx_memory , Rdx_memory, RonL, lce_memoryL, theta_knee_memoryL, dtheta_knee_memoryL, theta_trunk_memory, dtheta_trunk_memory)
        Stim_R = neural.Feedback(tsim, dt,StanceR_memory, Fm_memoryR, Rdx_memory , Ldx_memory, LonR, lce_memoryR, theta_knee_memoryR, dtheta_knee_memoryR, theta_trunk_memory, dtheta_trunk_memory)    
        
        print(Stim_L)
        
        
        # # ##### (4) From the stimulations we can calculate the activation of the muscles. This relation is characterized by a differential equation 
        # # #describing the excitation-contraction coupling.  This equation is solved by using a low pass filter :
        
        tau = 0.01 #time constant of the differential equation 
        
        if tsim == 0 :
            
            Act_L = np.hstack([u_f.low_filter(Stim_L, tau, 0,0),tsim])
            Act_R = np.hstack([u_f.low_filter(Stim_R, tau, 0,0),tsim])

        
        else :
            
                if len(np.shape(Act_L)) == 1 :
                    Act_L_new = np.hstack([u_f.low_filter(Stim_L, tau, dt,Act_L[:-1]),tsim])
                    Act_L = np.vstack([Act_L,Act_L_new])
                    Act_R_new = np.hstack([u_f.low_filter(Stim_R, tau, dt,Act_R[:-1]),tsim])
                    Act_R = np.vstack([Act_R,Act_R_new])
                    
                else :
                    
                    Act_L_new = np.hstack([u_f.low_filter(Stim_L, tau, dt,Act_L[-1,:-1]),tsim])
                    Act_L = np.vstack([Act_L,Act_L_new])
                    Act_L = u_f.pop_malone(Act_L)                    
                    Act_R_new = np.hstack([u_f.low_filter(Stim_R, tau, dt,Act_R[-1,:-1]),tsim])
                    Act_R = np.vstack([Act_R,Act_R_new])
                    Act_R = u_f.pop_malone(Act_R)
        
                    
        
        # # ##### (5) At each iteration, we update the length of the muscle unit from the angle of the joint.
        
        Lmtu_newL = np.zeros(n_muscle)
        Lmtu_newR = np.zeros(n_muscle)
        
        for i in range(n_muscle):
            if i == GAS :
                
                Lmtu_newL[i] = muscle.lmtu_update(muscle.model_angle(ankle, mbs_data.q[id_ankleL]), muscle.model_angle(knee, mbs_data.q[id_kneeL]), i, 0)
                Lmtu_newR[i] = muscle.lmtu_update(muscle.model_angle(ankle, mbs_data.q[id_ankleR]), muscle.model_angle(knee, mbs_data.q[id_kneeR]), i, 1)
            
            elif i == HAM :
                
                Lmtu_newL[i] = muscle.lmtu_update(muscle.model_angle(knee, mbs_data.q[id_kneeL]), muscle.model_angle(hip, mbs_data.q[id_hipL]), i, 0)
                Lmtu_newR[i] = muscle.lmtu_update(muscle.model_angle(knee, mbs_data.q[id_kneeR]), muscle.model_angle(hip, mbs_data.q[id_hipR]), i, 1)
            
            elif i == HFL or i == GLU  :
                
                Lmtu_newL[i] = muscle.lmtu_update(muscle.model_angle(hip, mbs_data.q[id_hipL]), 0, i, 0)
                Lmtu_newR[i] = muscle.lmtu_update(muscle.model_angle(hip, mbs_data.q[id_hipR]), 0, i, 1)
            
            elif i == VAS :
                
                Lmtu_newL[i] = muscle.lmtu_update(muscle.model_angle(knee, mbs_data.q[id_kneeL]), 0, i, 0)
                Lmtu_newR[i] = muscle.lmtu_update(muscle.model_angle(knee, mbs_data.q[id_kneeR]), 0, i, 1)
            
            elif i == SOL or i == TA :
                
                Lmtu_newL[i] = muscle.lmtu_update(muscle.model_angle(ankle, mbs_data.q[id_ankleL]), 0, i, 0)
                Lmtu_newR[i] = muscle.lmtu_update(muscle.model_angle(ankle, mbs_data.q[id_ankleR]), 0, i, 1)
        
        #print(Lmtu_newL[0],Lmtu_newL[3])
        ######## we can find the initial condition for lce thanks to the initial condition of lmtu and lslack
        
        if tsim ==0 :
            
            lce_memoryL = np.hstack([Lmtu_newL - muscle.l_slack_muscle,tsim])
            lce_memoryR = np.hstack([Lmtu_newR - muscle.l_slack_muscle,tsim])
        
        # # ##### (6) We can update the torques. Each torque corresponds to the torque involved in a specific muscle on a specific articulation joint.
        
        torque_newL = np.zeros((3,n_muscle))
        torque_newR = np.zeros((3,n_muscle))
        
        for j in range(n_articulation):
            for i in range(n_muscle):
                if len(np.shape(Fm_memoryL)) == 1 :
                
                    if j == ankle :
                
                        torque_newL[j,i] = muscle.torque_update(muscle.model_angle(ankle, mbs_data.q[id_ankleL]), Fm_memoryL[:-1], j, i, 0)
                        torque_newR[j,i] = muscle.torque_update(muscle.model_angle(ankle, mbs_data.q[id_ankleR]), Fm_memoryR[:-1], j, i, 1)
                    
                    elif j == knee : 
                        
                        torque_newL[j,i] = muscle.torque_update(muscle.model_angle(knee, mbs_data.q[id_kneeL]), Fm_memoryL[:-1], j, i, 0)
                        torque_newR[j,i] = muscle.torque_update(muscle.model_angle(knee, mbs_data.q[id_kneeR]), Fm_memoryR[:-1], j, i, 1)
                    
                    elif j == hip :
                        
                        torque_newL[j,i] = muscle.torque_update(muscle.model_angle(hip, mbs_data.q[id_hipL]), Fm_memoryL[:-1], j, i, 0)
                        torque_newR[j,i] = muscle.torque_update(muscle.model_angle(hip, mbs_data.q[id_hipR]), Fm_memoryR[:-1], j, i, 1)
                
                else : 
                    
                    if j == ankle :
                
                        torque_newL[j,i] = muscle.torque_update(muscle.model_angle(ankle, mbs_data.q[id_ankleL]), Fm_memoryL[-1,:-1], j, i, 0)
                        torque_newR[j,i] = muscle.torque_update(muscle.model_angle(ankle, mbs_data.q[id_ankleR]), Fm_memoryR[-1,:-1], j, i, 1)
                    
                    elif j == knee : 
                        
                        torque_newL[j,i] = muscle.torque_update(muscle.model_angle(knee, mbs_data.q[id_kneeL]), Fm_memoryL[-1,:-1], j, i, 0)
                        torque_newR[j,i] = muscle.torque_update(muscle.model_angle(knee, mbs_data.q[id_kneeR]), Fm_memoryR[-1,:-1], j, i, 1)
                    
                    elif j == hip :
                        
                        torque_newL[j,i] = muscle.torque_update(muscle.model_angle(hip, mbs_data.q[id_hipL]), Fm_memoryL[-1,:-1], j, i, 0)
                        torque_newR[j,i] = muscle.torque_update(muscle.model_angle(hip, mbs_data.q[id_hipR]), Fm_memoryR[-1,:-1], j, i, 1) 
        
        #print("torque R ankle : ",torque_newR[ankle])
        # # ##### (7) Update of Fm and vce memory. Once we have the muscle activations and that we have updated the length of the muscle unit, 
        # # #we can obtain the motor forces that result from these muscle activations. 
        # # #For this, the hill model is used, which will allow to do two things, an update of the length of the contractile element 
        # # #but also to obtain the motor force developed by the muscles.
        
        Fm_newL = np.zeros(n_muscle)
        Fm_newR = np.zeros(n_muscle)
        vce_newL = np.zeros(n_muscle)
        vce_newR = np.zeros(n_muscle)
          
        
        for i in range(n_muscle): #allows to obtain the muscular force for each of the muscles
            
            if len(np.shape(Act_L)) == 1 :
                
                vce_resultL = muscle.vce_compute(lce_memoryL[i], Lmtu_newL[i], Act_L[i], i)
                Fm_newL[i], vce_newL[i] = vce_resultL[1], vce_resultL[0]
    
                vce_resultR = muscle.vce_compute(lce_memoryR[i], Lmtu_newR[i], Act_R[i], i)
                Fm_newR[i], vce_newR[i] = vce_resultR[1], vce_resultR[0]
            else :
                if len(np.shape(lce_memoryL)) == 1 :
                    
                    vce_resultL = muscle.vce_compute(lce_memoryL[i], Lmtu_newL[i], Act_L[-1,i], i)
                    Fm_newL[i], vce_newL[i] = vce_resultL[1], vce_resultL[0]
                    
                    vce_resultR = muscle.vce_compute(lce_memoryR[i], Lmtu_newR[i], Act_R[-1,i], i)
                    Fm_newR[i], vce_newR[i] = vce_resultR[1], vce_resultR[0]
                    
                else : 
                    
                    vce_resultL = muscle.vce_compute(lce_memoryL[-1,i], Lmtu_newL[i], Act_L[-1,i], i)
                    Fm_newL[i], vce_newL[i] = vce_resultL[1], vce_resultL[0]
                    
                    vce_resultR = muscle.vce_compute(lce_memoryR[-1,i], Lmtu_newR[i], Act_R[-1,i], i)
                    Fm_newR[i], vce_newR[i] = vce_resultR[1], vce_resultR[0]
        
        #print(Fm_newR[SOL])

        if tsim == 0 :
              
            Fm_memoryL = np.hstack([Fm_newL,tsim])
            Fm_memoryR = np.hstack([Fm_newR,tsim])
            vce_memoryL = np.hstack([vce_newL,tsim])
            vce_memoryR = np.hstack([vce_newR,tsim]) 
            
        else : 
            
            Fm_memoryL_new = np.hstack([Fm_newL,tsim])
            Fm_memoryL = np.vstack([Fm_memoryL,Fm_memoryL_new])
            Fm_memoryL = u_f.pop_malone(Fm_memoryL)
            
            Fm_memoryR_new = np.hstack([Fm_newR,tsim])
            Fm_memoryR = np.vstack([Fm_memoryR,Fm_memoryR_new])
            Fm_memoryR = u_f.pop_malone(Fm_memoryR)
            
            vce_memoryL_new = np.hstack([vce_newL,tsim])
            vce_memoryL = np.vstack([vce_memoryL,vce_memoryL_new])
            vce_memoryL = u_f.pop_malone(vce_memoryL)
            
            vce_memoryR_new = np.hstack([vce_newR,tsim])
            vce_memoryR = np.vstack([vce_memoryR,vce_memoryR_new])
            vce_memoryR = u_f.pop_malone(vce_memoryR)
        
        # ##### (8) Thanks to the vce update we can calculate the contractile element length (lce) by using integration
        
        lce_newL = np.zeros(n_muscle)
        lce_newR = np.zeros(n_muscle)
        
        if dt !=0 :
            
            if len(np.shape(vce_memoryL)) == 1 :
                
                lce_newL = muscle.integrate_trapezoidal_2steps(lce_memoryL[:-1], vce_memoryL[:-1], dt)
                lce_newR = muscle.integrate_trapezoidal_2steps(lce_memoryR[:-1], vce_memoryR[:-1], dt)
            
            else : 
                if len(np.shape(lce_memoryL)) == 1 :
                
                    lce_newL = muscle.integrate_trapezoidal_2steps(lce_memoryL[:-1], vce_memoryL[-1,:-1], dt)
                    lce_newR = muscle.integrate_trapezoidal_2steps(lce_memoryR[:-1], vce_memoryR[-1,:-1], dt)
                else :
                    
                    lce_newL = muscle.integrate_trapezoidal_2steps(lce_memoryL[-1,:-1], vce_memoryL[-1,:-1], dt)
                    lce_newR = muscle.integrate_trapezoidal_2steps(lce_memoryR[-1,:-1], vce_memoryR[-1,:-1], dt)
                    
        else :
            
            lce_newL = lce_memoryL[:-1]
            lce_newR = lce_memoryR[:-1]
        
            
        if tsim == 0 : 
            
            lce_memoryL = np.hstack([lce_newL,tsim])
            lce_memoryR = np.hstack([lce_newR,tsim])
        
        else :
            
                
            lce_memoryL_new = np.hstack([lce_newL,tsim])
            lce_memoryL = np.vstack([lce_memoryL,lce_memoryL_new])
            lce_memoryL = u_f.pop_malone(lce_memoryL)
            
            lce_memoryR_new = np.hstack([lce_newR,tsim])
            lce_memoryR = np.vstack([lce_memoryR,lce_memoryR_new])
            lce_memoryR = u_f.pop_malone(lce_memoryR)
        
        #print("lce : ", lce_newL[0],lce_newL[3])
            
        ##### (9) Thanks to the torques update we can calculate the total torque on a joint and we can apply these total torques on our joints with 
        # robotran.
        
            
        mbs_data.Qq[id_ankleL] = muscle.torque_compute(ankle, muscle.model_angle(ankle, mbs_data.q[id_ankleL]),  - mbs_data.qd[id_ankleL],torque_newL)
        mbs_data.Qq[id_ankleR] = muscle.torque_compute(ankle, muscle.model_angle(ankle, mbs_data.q[id_ankleR]),  - mbs_data.qd[id_ankleR],torque_newR)
        mbs_data.Qq[id_kneeL] = muscle.torque_compute(knee, muscle.model_angle(knee, mbs_data.q[id_kneeL]), mbs_data.qd[id_kneeL],torque_newL)
        mbs_data.Qq[id_kneeR] = muscle.torque_compute(knee, muscle.model_angle(knee, mbs_data.q[id_kneeR]), mbs_data.qd[id_kneeR],torque_newR)
        mbs_data.Qq[id_hipL] = muscle.torque_compute(hip, muscle.model_angle(hip, mbs_data.q[id_hipL]),  - mbs_data.qd[id_hipL],torque_newL)
        mbs_data.Qq[id_hipR] = muscle.torque_compute(hip, muscle.model_angle(hip, mbs_data.q[id_hipR]),  - mbs_data.qd[id_hipR],torque_newR)
        
        # print("L:",mbs_data.Qq[id_kneeL])
        # print("D:",mbs_data.Qq[id_kneeR])
        
        #We can also compute the force on the inner thigh shift joint
        
        mbs_data.Qq[id_thighloadL] = u_f.pressure_sheet(mbs_data.q[id_thighloadL],mbs_data.qd[id_thighloadL])
        mbs_data.Qq[id_thighloadR] = u_f.pressure_sheet(mbs_data.q[id_thighloadR],mbs_data.qd[id_thighloadR])
         
        
    return
