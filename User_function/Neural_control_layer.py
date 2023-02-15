#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 14 14:29:41 2022

@author: matthieuxaussems
"""
import numpy as np
import useful_functions as u_f

# 1) Fonction utile pour réaliser les lois de feedback des muscles en stance et en swing phase.
# VAS : Force feedback, index du muscle 0
# SOL : force feedback, index du muscle 1
#
# parametre in : stance_or_swing(boolean true if stance, false if swing), Fm_memory c'est une matrice qui possède dans chaque ligne l'historique
# des Forces motrices à chaque pas de temps, current time, pas de temps qui a permis de générer la mémoire de la force motrice.
# parametre out : Vecteur Stim possédant l'update de la stimulation des muscles
#
def Feedback(Stance_memory,Fm_memory,ipsiDx_thigh,contraDx_thigh,contra_on_ipsi,lce_memory, theta_knee_memory, 
             dtheta_knee_memory,theta_trunk_memory,dtheta_trunk_memory,current_t, diff_t):
    
    VAS =0
    SOL =1
    GAS = 2
    TA = 3
    HAM = 4
    GLU = 5
    HFL = 6
    
    n_muscle = 7

    # données nécessaires pour les lois de reflexes
    
    G_VAS = 1.15/6000 #gains for the VAS muscle normalised with the maximal force
    G_SOL = 1.2/4000 #gains for the SOL muscle normalised with the maximal force
    
    G_GAS = 1.1/1500
    G_TA  = 1.1
    G_SOL_TA = 0.0001
    
    G_HAM = 0.65/3000
    G_GLU = 3.33e-4
    G_HFL = 0.5
    G_HAM_HFL = 4
    
    #offset
    
    loff_TA = 0.71
    lopt_TA = 6
    loff_HAM = 0.85
    lopt_HAM = 10
    loff_HFL = 0.6
    lopt_HFL = 11
    
    #pre-stimulations
    
    So = 0.01 
    So_VAS = 0.09 #VAS
    So_BAL = 0.05 #HAM,GLU,HFL en stance
    
    #parameters
    
    k_lean = 1.15
    k_p = 1.91
    k_d = 0.2
    k_bw = 1.2/80 #normaliser par le bodyweight
    k_phi = 2
    phi_k_off = 2.97
    theta_ref = 0.105 
    delta_S = 0.25
    
    #delay
    
    t_l = current_t - 0.02 # (t- time delay) for long neural signal delay 
    t_m = current_t - 0.01 # (t- time delay) for middle neural signal delay 
    t_s = current_t - 0.005 # (t- time delay) for small neural signal delay 
    
    #Double support constant 
    
    DS=0.25
    
    #delay on stance informations
    
    if t_m<0 : 
        
        stance_or_swing = Stance_memory[0]
    
    else :
        
        stance_or_swing = Stance_memory[int(round(t_m))]
    
    
    #lois de reflexe
    
    Stim = np.zeros(n_muscle)
    PTO= 0
    
    ####### STANCE PHASE #######

    if stance_or_swing == True: 
        
        ###### loi de reflex pour les muscles ayant un delai small (HAM,GLU,HFL) :
        
        if t_s < 0:
            
            Stim[HAM] = So_BAL
            Stim[GLU] = So_BAL
            Stim[HFL] = So_BAL
            
        else: 
            
            Stim[HAM] = So_BAL + u_f.limit_range((k_p * (u_f.interpolation_memory(theta_trunk_memory, diff_t, t_s)-theta_ref) + k_d* u_f.interpolation_memory(dtheta_trunk_memory, diff_t, t_s)),0,float("inf"))
            Stim[GLU] = So_BAL + u_f.limit_range((k_p * (u_f.interpolation_memory(theta_trunk_memory, diff_t, t_s)-theta_ref) + k_d* u_f.interpolation_memory(dtheta_trunk_memory, diff_t, t_s)),0,float("inf")) 
            Stim[HFL] = So_BAL - u_f.limit_range((k_p * (u_f.interpolation_memory(theta_trunk_memory, diff_t, t_s)-theta_ref) + k_d* u_f.interpolation_memory(dtheta_trunk_memory, diff_t, t_s)),float("-inf"),0) 
            
            PTO = (u_f.interpolation_memory(theta_trunk_memory, diff_t, t_s)-theta_ref) 
        
        ###### loi de reflex pour les muscles ayant un delai mid (VAS) :
        
        if t_m < 0 : #tant que l'on a pas depassé le delay du signal le muscle est silent au debut de la simulation
            
            Stim[VAS]= So_VAS
        
        else :
            
            if len(np.shape(Fm_memory)) == 1 : 
                
                Stim[VAS] = So_VAS + G_VAS*Fm_memory[VAS]
                
            else :
                
                Stim[VAS] = So_VAS + G_VAS*u_f.interpolation_memory(Fm_memory, diff_t, t_m)[VAS]
                
            
            if u_f.interpolation_memory(theta_knee_memory, diff_t, t_m) > phi_k_off and u_f.interpolation_memory(dtheta_knee_memory, diff_t, t_m)>0:
                
                Stim[VAS] -= k_phi*(u_f.interpolation_memory(theta_knee_memory, diff_t, t_m)-phi_k_off)
                

        ###### loi de reflex pour les muscles ayant un delai long (SOL,GAS,TA)
        
        if t_l < 0 :
            
            Stim[SOL] = So
            Stim[GAS] = So
            Stim[TA] = So 
        
        else :
            
            if len(np.shape(Fm_memory)) == 1 : 
                
                Stim[SOL] = So + G_SOL*Fm_memory[SOL]
                Stim[GAS] = So + G_GAS*Fm_memory[GAS]
                Stim[TA]  = So - G_SOL_TA*Fm_memory[SOL]

            else :
                
                Stim[SOL] = So + G_SOL*u_f.interpolation_memory(Fm_memory, diff_t, t_l)[SOL]
                Stim[GAS] = So + G_GAS*u_f.interpolation_memory(Fm_memory, diff_t, t_l)[GAS]
                Stim[TA]  = So - G_SOL_TA*u_f.interpolation_memory(Fm_memory, diff_t, t_l)[SOL]
            
            if len(np.shape(lce_memory)) == 1 :
                
                Stim[TA] += G_TA*u_f.limit_range((lce_memory[TA]/lopt_TA - loff_TA),0,float("inf"))
            
            else : 
                
                Stim[TA] += G_TA*u_f.limit_range((u_f.interpolation_memory(lce_memory, diff_t, t_l)[TA]/lopt_TA - loff_TA),0,float("inf"))
                
        
        ##### Impact de la charge portée par la jambe ipsilatérale
        
        DeltaThRef =0.005
        
        if t_s >= 0 :
            IDx_thigh = u_f.limit_range(u_f.interpolation_memory(ipsiDx_thigh, diff_t, t_s), 0, float("inf"))
            Stim[HAM] = u_f.limit_range((IDx_thigh/DeltaThRef *Stim[HAM]),0,1)
            Stim[GLU] = u_f.limit_range((IDx_thigh/DeltaThRef *Stim[GLU]),0,1)
            Stim[HFL] = u_f.limit_range((IDx_thigh/DeltaThRef *Stim[HFL]),0,1)
        
        ##### Double support 
        
        if t_s >= 0 :
            
            CDx_thigh = u_f.limit_range(u_f.interpolation_memory(contraDx_thigh, diff_t, t_s), 0, float("inf"))
            Stim[VAS] -= contra_on_ipsi*u_f.interpolation_memory(CDx_thigh, diff_t, t_s)/DeltaThRef
        
        Stim[GLU] -= contra_on_ipsi*DS
        Stim[HFL] += contra_on_ipsi*DS
        
        ##### Gain parameter for GLU
        
        Stim[GLU]*=0.7
         
        
        
        
    ####### SWING PHASE #######
    

    else : #permet dobtenir la stimulation des muscles en swing phase 
        
        ###### loi de reflex pour le VAS, SOL et GAS (juste pré-stimulation)
        
        Stim[VAS] = 0    
        Stim[SOL] = So 
        Stim[GAS] = So
        
        ###### loi de reflex pour les muscles ayant un delai small (HAM,GLU,HFL)
        
        if t_s <0 :
        
            Stim[HAM] = So
            Stim[GLU] = So
            Stim[HFL] = So
            
        else :
            
            if len(np.shape(Fm_memory)) == 1 : 
                
                Stim[HAM] = So + G_HAM * Fm_memory[HAM]
                Stim[GLU] = So + G_GLU * Fm_memory[GLU]
                
            else :
                
                Stim[HAM] = So + G_HAM * u_f.interpolation_memory(Fm_memory, diff_t, t_s)[HAM]
                Stim[GLU] = So + G_GLU * u_f.interpolation_memory(Fm_memory, diff_t, t_s)[GLU]
        
            
            if len(np.shape(lce_memory)) == 1 :
                
                Stim[HFL] = So + G_HFL * u_f.limit_range((lce_memory[HFL]/lopt_HFL - loff_HFL),0,float("inf")) - G_HAM_HFL * u_f.limit_range((lce_memory[HAM]/lopt_HAM  - loff_HAM),0,float("inf")) + k_lean*PTO
                
            else : 
                
                Stim[HFL] = So + G_HFL * u_f.limit_range((u_f.interpolation_memory(lce_memory, diff_t, t_s)[HFL]/lopt_HFL - loff_HFL),0,float("inf")) - G_HAM_HFL * u_f.limit_range((u_f.interpolation_memory(lce_memory, diff_t, t_s)[HAM]/lopt_HAM  - loff_HAM),0,float("inf")) + k_lean*PTO
                
                
        ###### loi de reflex pour les muscles ayant un delai long (TA)
        
        if t_l <0 :
        
            Stim[TA] = So
            
        else :
            
            if len(np.shape(lce_memory)) ==1 :
                
                Stim[TA] = So + G_TA * u_f.limit_range((lce_memory[TA]/lopt_TA - loff_TA),0,float("inf"))
            
            else : 
                
                Stim[TA] = So + G_TA * u_f.limit_range((u_f.interpolation_memory(lce_memory, diff_t, t_l)[TA]/lopt_TA - loff_TA),0,float("inf"))
    
    for i in range(len(Stim)):
        Stim[i] = u_f.limit_range(Stim[i], 0.01, 1)
        
    return Stim

# 2) Fonction qui permet de savoir si on est dans la Stance ou la Swing phase ou dans la phase de double support à savoir la phase de transition
#
# parametre in : Ball_Sensor_L, Heel_Sensor_L,Ball_Sensor_R, Heel_Sensor_R (boolean true if point in contact with the ground at current time)
# parametre out : Stance_L, Stance_R, Double_Support,flight (booleans respectively true if left leg in stance, true if right leg in stance, 
#true if double support, true if in flight) 
#

def stance_swing(Ball_Sensor_L, Heel_Sensor_L,Ball_Sensor_R, Heel_Sensor_R):
    
    Flag = [False,False,False,False]
    
    if Ball_Sensor_L == True or Heel_Sensor_L == True :
        
        Flag[0]= True 
    
    if Ball_Sensor_R == True or Heel_Sensor_R == True :
        
        Flag[1] = True
    
    if Flag[0] == True and Flag[1] == True: 
        
        Flag[2] = True
    
    if Flag[0]== False and Flag[1] == False:
        
        Flag[3] = True
    
    return Flag

# 3) Fonction qui permet d'obtenir les boolean d'entrée de la fonction 2)
#
# parametre in :Position en z des 4 senseurs, 2 sur les talons et 2 sur les pointes (BallL,HeelL,BallR,HeelR) et position du sol.
# parametre out : Ball_Sensor_L, Heel_sensor_L, Ball_Sensor_R, Heel_sensor_R
#

def Boolean_sensors(P_Ball_L,P_Heel_L,P_Ball_R,P_Heel_R,Ground_limit):
    Ball_Sensor_L = False
    Heel_Sensor_L = False
    Ball_Sensor_R = False
    Heel_Sensor_R = False
    
    if P_Ball_L >= Ground_limit:
        Ball_Sensor_L = True
    if P_Ball_R >= Ground_limit:
        Ball_Sensor_R = True
    if P_Heel_L >= Ground_limit:
        Heel_Sensor_L = True
    if P_Heel_R >= Ground_limit:
        Heel_Sensor_R = True
        
    
    return [Ball_Sensor_L,Heel_Sensor_L,Ball_Sensor_R,Heel_Sensor_R]
    

# 4) Fonction qui permet de savoir quelle jambe a le lead sur l'autre, c'est à dire que lorsque les 2 jambes sont en stance depuis plusieurs 
# itérations, c'est la jambe est rentrée en dernière en stance phase
#
# parametre in : StanceL_memory vecteur de boolean mémoire de l'etat de stance de la jambe gauche (chaque valeur du vecteur correspond à une 
#itération) et StanceR_memory vecteur de boolean mémoire de l'etat de stance de la jambe droite
# parametre out :  LonR (1 si L lead on R et 0 sinon) et RonL (1 si R lead on L 0 sinon)
#

def lead(StanceL_memory,StanceR_memory): 
    
    LonR = 0
    RonL = 0
    
    CountL =0
    CountR =1
    
    for i in range(len(StanceL_memory)):
        
        if StanceL_memory[i]:
            
            CountL=CountL + 1
        else :
            CountL = 0
        
        if StanceR_memory [i]:
            
            CountR = CountR +1
        else : 
            CountR = 0
    
    if CountR !=0 and CountL !=0 :
        
        if CountR < CountL :
            
            RonL = 1
            LonR = 0
        else :
            
            LonR = 1
            RonL = 0
    else :
        
        LonR = 0
        RonL =0
    
    return [LonR,RonL]
    
   
    
    
    
    
    
