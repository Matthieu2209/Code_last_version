#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 10 14:38:35 2022

@author: matthieuxaussems
"""

###validation des résultats intermédiaires 

import numpy as np
import math
import matplotlib.pyplot as plt
import useful_functions as u_f

import Muscle_actuation_layer as m_layer
import Neural_control_layer as neural

### muscle actuation ###
ankle=0
knee = 1
hip=2
VAS =0
SOL=1
GAS=2
TA=3
HAM=4
GLU=5
HFL=6

time= np.linspace(0, 10, 100)
Stim = np.zeros(100)
l = np.ones(100)*0.08
lce=np.zeros(100)
lse = np.zeros(100)
VAS = 0
fv_C1 = np.zeros(100)
fv_C2 = np.zeros(100)
phiknee = np.zeros(100)
dphi = np.zeros(100)
lkneestate = np.zeros(100)
theta = np.zeros(100)
dtheta = np.zeros(100)
Ldxthigh = np.zeros(100)
Rdxthigh = np.zeros(100)
RonL = np.zeros(100)
LfmVAS = np.zeros(100)
LfmGAS = np.zeros(100)
LfmSOL = np.zeros(100)
LfmHAM = np.zeros(100)
LfmGLU = np.zeros(100)
LlceTA = np.zeros(100)
LlceHFL = np.zeros(100)
LlceHAM = np.zeros(100)
deltatheta = np.zeros(100)

z_fn = np.zeros(100)
vz_fn = np.zeros(100)
x_fn = np.zeros(100)
vx_fn = np.zeros(100)



for i in range (100):
    if i <=9 :
        
        Stim[i] = 0.08
        lce[i] = 2*0.08
        lse[i] = 2*0.23
        fv_C1[i] = 0.1
        fv_C2[i] = 1.1
        phiknee[i] = 4
        dphi[i] = 0.1
        lkneestate[i] = 0
        theta[i] = 0.2
        dtheta[i] = 0.1
        Ldxthigh[i] = 0
        Rdxthigh[i] = 0.006
        RonL[i] = 1
        LfmVAS[i] = 0
        LfmGAS[i] = 0
        LfmSOL[i] = 0
        LlceTA[i] = 0
        deltatheta[i]= 0
        LlceHFL[i]=0.1
        LlceHAM[i] =0
        LfmHAM[i] = 0
        LfmGLU[i] = 0
        z_fn[i] = 1.91
        vz_fn[i] = 0.01
        x_fn[i] = 0
        
    elif i <= 19 :
        
        Stim[i] = 0.25
        lce[i] = 1.8*0.08
        lse[i] = 1.8*0.23
        fv_C1[i] = 0.2
        fv_C2[i] = 1.2
        phiknee[i] = 4.1
        dphi[i] = 0.1
        lkneestate[i] = 0.1745
        theta[i] = 0.1
        dtheta[i] = -0.1
        Ldxthigh[i] = 0
        Rdxthigh[i] = 0.006
        RonL[i] = 1
        LfmVAS[i] = 600
        LfmGAS[i] = 0
        LfmSOL[i] = 0
        LlceTA[i] = 0.15
        deltatheta[i]= 0.1
        LlceHFL[i]=0.45
        LlceHAM[i] =0.16
        LfmHAM[i] = 400
        LfmGLU[i] = 400
        z_fn[i] = 1.9
        vz_fn[i] = -0.01
        x_fn[i] = 0.01
        
    elif i <=29 :
        
        Stim[i] = 0.08
        l[i] = 1.6
        lce[i] = 1.6*0.08
        lse[i] = 1.6*0.23
        fv_C1[i] = 0.3
        fv_C2[i] = 1.3
        phiknee[i] = 4.2
        dphi[i] = 0.1
        lkneestate[i] = 0
        theta[i] = 0.2
        dtheta[i] = 0.1
        Ldxthigh[i] = 0.006
        Rdxthigh[i] = 0
        RonL[i] = 0
        LfmVAS[i] = 1300
        LfmGAS[i] = 1100
        LfmSOL[i] = 2100
        LlceTA[i] = 0.22
        deltatheta[i]= 0.15
        LlceHFL[i]=0.1
        LlceHAM[i] =0
        LfmHAM[i] = 1400
        LfmGLU[i] = 550
        z_fn[i] = 1.91
        vz_fn[i] = 0.01
        x_fn[i] = 0
    
    elif i <= 39 :
        
        Stim[i] = 0.25
        l[i] = 1.4
        lce[i] = 1.4*0.08
        lse[i] = 1.4*0.23
        fv_C1[i] = 0.4
        fv_C2[i] = 1.4
        phiknee[i] = 4.3
        dphi[i] = 0.1
        lkneestate[i] = 0.1745
        theta[i] = 0.1
        dtheta[i] = -0.1
        Ldxthigh[i] = 0.006
        Rdxthigh[i] = 0
        RonL[i] = 0
        LfmVAS[i] = 0
        LfmGAS[i] = 0
        LfmSOL[i] = 0
        LlceTA[i] = 0
        deltatheta[i]= 0
        LlceHFL[i]=0.45
        LlceHAM[i] =0.16
        LfmHAM[i] = 0
        LfmGLU[i] = 0
        z_fn[i] = 1.9
        vz_fn[i] = -0.01
        x_fn[i] = 0.01
    
    elif i <= 49 :
        
        Stim[i] = 0.08
        l[i] = 1.2
        lce[i] = 1.2*0.08
        lse[i] = 1.2*0.23
        fv_C1[i] = 0.5
        fv_C2[i] = 1.5
        phiknee[i] = 4.4
        dphi[i] = 0.1
        lkneestate[i] = 0
        theta[i] = 0.2
        dtheta[i] = 0.1
        Ldxthigh[i] = 0
        Rdxthigh[i] = 0.006
        RonL[i] = 1
        LfmVAS[i] = 600
        LfmGAS[i] = 0
        LfmSOL[i] = 0
        LlceTA[i] = 0.15
        deltatheta[i]= 0.1
        LlceHFL[i]=0.1
        LlceHAM[i] =0
        LfmHAM[i] = 400
        LfmGLU[i] = 400
        z_fn[i] = 1.91
        vz_fn[i] = 0.01
        x_fn[i] = 0
        
    elif i <= 59 :
        
        Stim[i] = 0.25
        l[i] = 0.4
        lce[i] = 1*0.08
        lse[i] = 1*0.23
        fv_C1[i] = 0.5
        fv_C2[i] = 1.5
        phiknee[i] = 4.5
        dphi[i] = 0.1
        lkneestate[i] = 0.1745
        theta[i] = 0.1
        dtheta[i] = -0.1
        Ldxthigh[i] = 0
        Rdxthigh[i] = 0.006
        RonL[i] = 1
        LfmVAS[i] = 1300
        LfmGAS[i] = 1100
        LfmSOL[i] = 2100
        LlceTA[i] = 0.22
        deltatheta[i]= 0.15
        LlceHFL[i]=0.45
        LlceHAM[i] =0.16
        LfmHAM[i] = 1400
        LfmGLU[i] = 550
        z_fn[i] = 1.9
        vz_fn[i] = -0.01
        x_fn[i] = 0.01
    
    elif i <= 69 :
        
        Stim[i] = 0.08
        lce[i] = 1.2*0.08
        lse[i] = 1.2*0.23
        fv_C1[i] = 0.6
        fv_C2[i] = 1.4
        phiknee[i] = 4.6
        dphi[i] = 0.1
        lkneestate[i] = 0
        theta[i] = 0.2
        dtheta[i] = 0.1
        Ldxthigh[i] = 0.006
        Rdxthigh[i] = 0
        RonL[i] = 0
        LfmVAS[i] = 0
        LfmGAS[i] = 0
        LfmSOL[i] = 0
        LlceTA[i] = 0
        deltatheta[i]= 0
        LlceHFL[i]=0.1
        LlceHAM[i] =0
        LfmHAM[i] = 0
        LfmGLU[i] = 0
        z_fn[i] = 1.91
        vz_fn[i] = 0.01
        x_fn[i] = 0
        
    
    elif i <= 79 :
        
        Stim[i] = 0.25
        lce[i] = 1.4*0.08
        lse[i] = 1.4*0.23
        fv_C1[i] = 0.7
        fv_C2[i] = 1.3
        phiknee[i] = 4.7
        dphi[i] = 0.1
        lkneestate[i] = 0.1745
        theta[i] = 0.1
        dtheta[i] = -0.1
        Ldxthigh[i] = 0.006
        Rdxthigh[i] = 0
        RonL[i] = 0
        LfmVAS[i] = 600
        LfmGAS[i] = 0
        LfmSOL[i] = 0
        LlceTA[i] = 0.15
        deltatheta[i]= 0.1
        LlceHFL[i]=0.45
        LlceHAM[i] =0.16
        LfmHAM[i] = 400
        LfmGLU[i] = 400
        z_fn[i] = 1.9
        vz_fn[i] = -0.01
        x_fn[i] = 0.01
    
    elif i <= 89 :
        
        Stim[i] = 0.08
        lce[i] = 1.6*0.08
        lse[i] = 1.6*0.23
        fv_C1[i] = 0.8
        fv_C2[i] = 1.2
        phiknee[i] = 4.8
        dphi[i] = 0.1
        lkneestate[i] = 0
        theta[i] = 0.2
        dtheta[i] = 0.1
        Ldxthigh[i] = 0
        Rdxthigh[i] = 0.006
        RonL[i] = 1
        LfmVAS[i] = 1300
        LfmGAS[i] = 1100
        LfmSOL[i] = 2100
        LlceTA[i] = 0.22
        deltatheta[i]= 0.15
        LlceHFL[i]=0.1
        LlceHAM[i] =0
        LfmHAM[i] = 1400
        LfmGLU[i] = 550
        z_fn[i] = 1.91
        vz_fn[i] = 0.01
        x_fn[i] = 0
        
        
    elif i <= 99 :
        
        Stim[i] = 0.25
        lce[i] = 1.8*0.08
        lse[i] = 1.8*0.23
        fv_C1[i] = 0.9
        fv_C2[i] = 1.1
        phiknee[i] = 4.9
        dphi[i] = 0.1
        lkneestate[i] = 0.1745
        theta[i] = 0.1
        dtheta[i] = -0.1
        Ldxthigh[i] = 0
        Rdxthigh[i] = 0.006
        RonL[i] = 1
        LfmVAS[i] = 0
        LfmGAS[i] = 0
        LfmSOL[i] = 0
        LlceTA[i] = 0
        deltatheta[i]= 0
        LlceHFL[i]=0.45
        LlceHFL[i]=0.1
        LlceHAM[i] =0.16
        LfmHAM[i] = 0
        LfmGLU[i] = 0
        z_fn[i] = 1.9
        vz_fn[i] = -0.01
        x_fn[i] = 0.01

vx_fn = -vz_fn
pos_FP = 0.01
        
# force-length relation for se : ok

f_see=np.zeros(100)
for i in range(100):
    epsilon = (lse[i] - m_layer.l_slack_muscle[VAS])/m_layer.l_slack_muscle[VAS]
    if lse[i]/m_layer.l_slack_muscle[VAS] > 1 :
        f_see[i] = (epsilon/m_layer.epsilon_ref)**2
    else:
        f_see[i] = 0
    
    if i ==9 :
        print(f_see)

# force-length relation for be : ok

F_be = np.zeros(100)
for i in range(100):
    
    if lce[i] <= m_layer.l_min_muscle[VAS]:
    
        F_be[i] = ((m_layer.l_min_muscle[VAS]-lce[i])/(m_layer.l_opt_muscle[VAS]*m_layer.epsilon_be))**2
    else :
        F_be[i]=0
    
    if i ==9 :
        print(F_be)

# force-length relation for pe : ok

F_pe_star = np.zeros(100)
for i in range(100):
    
    if lce[i] > m_layer.l_opt_muscle[VAS]:
    
        F_pe_star[i] =((lce[i]-m_layer.l_opt_muscle[VAS])/(m_layer.l_opt_muscle[VAS]*m_layer.epsilon_pe))**2
    else :
        F_pe_star[i] = 0
    
    if i ==9 :
        print(F_pe_star)

# force-length relation for ce : ok

f_l = np.zeros(100)
for i in range(100):
    
    f_l[i] = np.exp(m_layer.c*(abs((lce[i]-m_layer.l_opt_muscle[VAS])/(m_layer.l_opt_muscle[VAS]*m_layer.w_muscle)))**3)
    #if f_l[i] < m_layer.fl_inf:
    #    f_l[i] = m_layer.fl_inf
    
    if i ==9 :
        print(f_l)


#excitation-contraction coupling : ok

tau = 0.01
Act= np.zeros(100)

for i in range(100):
    
    if i ==0 :
        
        Act[i]= u_f.low_filter(Stim[i], tau, 0, 0)
    
    else : 
        
        Act[i]= u_f.low_filter(Stim[i], tau, 1, Act[i-1])
        if i ==9 :
            print(Act)

# calcul de fv : ok

F_se = f_see * m_layer.F_max_muscle[VAS]
F_be_ = F_be * m_layer.F_max_muscle[VAS]
F_pe = F_pe_star * m_layer.F_max_muscle[VAS]


f_v = np.zeros(100)
for i in range(100):
    
    if Act[i] ==0 :
        
        f_v[i] = m_layer.fv_inf

    
    else :
    
        f_v[i] = (f_see[i] + F_be[i])/(Act[i]*f_l[i] + F_pe_star[i])
        if i ==9 :
            print(f_v)

# calcul de v_ce à partir de la relation force-vitesse inverse : ok 
#(possibilité de tester les 3 conditions en changeant les variables fv)

vce = np.zeros(100)
for i in range(100):
    
    if fv_C2[i] <= 1 : 
         
        v_ce_norm = -m_layer.v_max_muscle[VAS] * ((1-fv_C2[i])/(1+m_layer.K_muscle*fv_C2[i]))
    
    elif fv_C2[i] > 1 and fv_C2[i] <= m_layer.N_muscle :
    
        v_ce_norm = -m_layer.v_max_muscle[VAS] * ((fv_C2[i] - 1.0) / (7.56 * m_layer.K_muscle * (fv_C2[i] - m_layer.N_muscle) + 1.0 - m_layer.N_muscle))
    
    else : 
    
        v_ce_norm = m_layer.v_max_muscle[VAS] * ((fv_C2[i] - m_layer.N_muscle)*(0.01) + 1)

    
    vce[i] = v_ce_norm*m_layer.l_opt_muscle[VAS]


#######################

#verification de torque update(#pour tester suffit de choisir le muscle et l'articulation) ok
lever =np.zeros(100)
articulation = ankle
muscle = SOL
for i in range(100):
    
    if articulation == hip : 
        
        lever[i] = m_layer.r_0[articulation,muscle]
        
    else :
        
        lever[i] = m_layer.r_0[articulation,muscle]*np.cos(phiknee[i]-m_layer.phi_max[articulation,muscle])


#verification de lmtu update (#pour tester suffit de choisir le muscle) ok 

lmtu = np.zeros(100)
test1 = np.zeros(100)
muscle = HFL

for i in range(100):
    

    if muscle == VAS :
        
        delta_lmtu = + m_layer.rho[knee,VAS]*m_layer.r_0[knee,VAS]*(np.sin(m_layer.phi_ref[knee,VAS]-m_layer.phi_max[knee,VAS]) - np.sin(phiknee[i]-m_layer.phi_max[knee,VAS]))
        
    elif muscle == SOL :
        
        delta_lmtu = + m_layer.rho[ankle,SOL]*m_layer.r_0[ankle,SOL]*(np.sin(m_layer.phi_ref[ankle,SOL]-m_layer.phi_max[ankle,SOL]) - np.sin(phiknee[i]-m_layer.phi_max[ankle,SOL]))
    
    elif muscle == GAS :
        
        delta_lmtu = (m_layer.rho[ankle,GAS]*m_layer.r_0[ankle,GAS]*(np.sin(m_layer.phi_ref[ankle,GAS]-m_layer.phi_max[ankle,GAS]) - np.sin(phiknee[i]-m_layer.phi_max[ankle,GAS])) 
        - m_layer.rho[knee,GAS]*m_layer.r_0[knee,GAS]*(np.sin(m_layer.phi_ref[knee,GAS]-m_layer.phi_max[knee,GAS]) - np.sin(phiknee[i]-m_layer.phi_max[knee,GAS])))
    
    elif muscle == TA : 
        
        delta_lmtu = - m_layer.rho[ankle,TA]*m_layer.r_0[ankle,TA]*(np.sin(m_layer.phi_ref[ankle,TA]-m_layer.phi_max[ankle,TA]) - np.sin(phiknee[i]-m_layer.phi_max[ankle,TA]))
    
    elif muscle == HAM :
        
        delta_lmtu = (-m_layer.rho[knee,HAM]*m_layer.r_0[knee,HAM]*(np.sin(m_layer.phi_ref[knee,HAM]-m_layer.phi_max[knee,HAM]) - np.sin(phiknee[i]-m_layer.phi_max[knee,HAM]))
        - m_layer.rho[hip,HAM]*m_layer.r_0[hip,HAM]*(phiknee[i]-m_layer.phi_ref[hip,HAM]))
    
    elif muscle == GLU :
        
        delta_lmtu = - m_layer.rho[hip,GLU]*m_layer.r_0[hip,GLU]*(phiknee[i]-m_layer.phi_ref[hip,GLU])
    
    elif muscle == HFL :
        
        delta_lmtu = + m_layer.rho[hip,HFL]*m_layer.r_0[hip,HFL]*(phiknee[i]-m_layer.phi_ref[hip,HFL])
        
    
    lmtu[i]= m_layer.l_opt_muscle[muscle] + m_layer.l_slack_muscle[muscle] + delta_lmtu
    test1[i] = m_layer.rho[ankle,GAS]*m_layer.r_0[ankle,GAS]*(np.sin(m_layer.phi_ref[ankle,GAS]-m_layer.phi_max[ankle,GAS]) - np.sin(phiknee[i]-m_layer.phi_max[ankle,GAS]))



#verification de joint limits (pourchanger l'articulation suffit d'indiquer l'articulation à analyser dans art )
ext_torque = np.zeros(100)
torque_j = np.zeros(100)

phi_a_up = 130 * np.pi/180
phi_a_low = 70 * np.pi/180
phi_k_up = 175 * np.pi/180
phi_h_up = 230 * np.pi/180

c_joint = 17.1887
w_max = 0.01745

art = hip

if art == knee :
    for i in range(100):
        
    
        if phiknee[i] > 40 * np.pi/180 and phiknee[i] < 185 * np.pi/180 : #check that the angle values do not take unrealistic values
        
            u1 = (phiknee[i] - phi_k_up)*c_joint
            u2 = - dphi[i] / w_max
    
            if u2<1 and u1>0 :
                
                ext_torque[i] = -u1*(1-u2)
            
            else : 
                
                ext_torque[i] = 0
            
        
        else : # raise an error if unrealistic values 
            
            ext_torque[i] = -100
        print(ext_torque[i])

if art == ankle :
    
    for i in range(100):
        
        if phiknee[i]> 30 * np.pi/180 and phiknee[i] < 170 * np.pi/180 : #check that the angle values do not take unrealistic values
            
            u1 = (phiknee[i]- phi_a_up)*c_joint
            u2 = - dphi[i] / w_max
            u3 = (phiknee[i]- phi_a_low)*c_joint
            u4 = dphi[i]/w_max
            
            if u2<1 and u1>0 :
                
                ext_torque = -u1*(1-u2)
            
            else : 
                
                ext_torque = 0
            
            if u4<1 and u3 < 0 :
                
                flex_torque = -u3*(1-u4)
            
            else :
                
                flex_torque = 0
            
            torque_j[i] = (ext_torque + flex_torque)
        
        else : # raise an error if unrealistic values 
            
            torque_j[i]=-200
        print(torque_j[i])
        
if art == hip :
    
    for i in range(100):
        if phiknee[i]> 10 * np.pi/180 and phiknee[i] < 260 * np.pi/180 : #check that the angle values do not take unrealistic values
    
            u1 = (phiknee[i]- phi_h_up)*c_joint
            u2 = - dphi[i] / w_max
            if u2<1 and u1>0 :
                
                ext_torque[i] = -u1*(1-u2)
            
            else : 
                
                ext_torque[i] = 0
        
        else : # raise an error if unrealistic values 
                
            ext_torque[i]=-200
        print(ext_torque[i])
        
        
# Verification Stance reflex

So_BAL = 0.05 #HAM,GLU,HFL
So_VAS = 0.09 #VAS
So = 0.01 
G_VAS = 1.15/6000 #gains for the VAS muscle normalised with the maximal force
G_GAS = 1.1/1500
G_SOL_TA = 0.0001
G_TA  = 1.1
G_SOL = 1.2/4000 #gains for the SOL muscle normalised with the maximal force
G_HFL = 0.5
G_HAM_HFL = 4
G_GLU = 3.33e-4
G_HAM = 0.65/3000

lopt_TA = 6
loff_TA = 0.71
k_p = 1.91
theta_ref = 0.105
k_d = 0.2
DS=0.25
phi_k_off = 2.97
k_phi = 2
DeltaThRef =0.005
k_lean = 1.1459
Stim_HFL = np.zeros(100)
Stim_GLU = np.zeros(100)
Stim_HAM = np.zeros(100)
Stim_VAS = np.zeros(100)
Stim_GAS = np.zeros(100)
Stim_TA = np.zeros(100)
Stim_SOL = np.zeros(100)

for i in range(100) :
    
    #HFL
    
    Stim_HFL[i] = So_BAL - u_f.limit_range((k_p * (theta[i] -theta_ref) + k_d* dtheta[i]),float("-inf"),0)
    Dx_thigh = u_f.limit_range(Ldxthigh[i], 0, float("inf"))
    Stim_HFL[i] = u_f.limit_range((Dx_thigh/DeltaThRef *Stim_HFL[i]),0,1)
    Stim_HFL[i] += RonL[i]*DS
    
    #GLU
    
    Stim_GLU[i] = So_BAL + u_f.limit_range((k_p * (theta[i]-theta_ref) + k_d* dtheta[i]),0,float("inf"))
    Dx_thigh = u_f.limit_range(Ldxthigh[i], 0, float("inf"))
    Stim_GLU[i] = u_f.limit_range((Dx_thigh/DeltaThRef *Stim_GLU[i]),0,1)
    Stim_GLU[i] -= RonL[i]*DS
    Stim_GLU[i]*=0.7
    
    #HAM
    
    Stim_HAM[i] = So_BAL + u_f.limit_range((k_p * (theta[i]-theta_ref) + k_d* dtheta[i]),0,float("inf"))
    Dx_thigh = u_f.limit_range(Ldxthigh[i], 0, float("inf"))
    Stim_HAM[i] = u_f.limit_range((Dx_thigh/DeltaThRef *Stim_HAM[i]),0,1)
    
    #VAS
    
    Stim_VAS[i] = So_VAS+ G_VAS *LfmVAS[i]
    Stim_VAS[i] -= k_phi*lkneestate[i]
    Dx_thigh = u_f.limit_range(Rdxthigh[i], 0, float("inf"))
    Stim_VAS[i] -= RonL[i]*Dx_thigh/DeltaThRef
    
    #GAS 
    
    Stim_GAS[i] = So + G_GAS*LfmGAS[i]
    
    #TA
    
    Stim_TA[i] = So - G_SOL_TA*LfmSOL[i]
    Stim_TA[i] += G_TA*LlceTA[i]
    
    #SOL
    
    Stim_SOL[i] = So + G_SOL*LfmSOL[i]


#verification Swing reflex 

for i in range(100):
    
    #HFL
    Stim_HFL[i] = So + k_lean*deltatheta[i]
    Stim_HFL[i] += G_HFL * LlceHFL[i]  
    Stim_HFL[i] -= G_HAM_HFL * LlceHAM[i]
    
    #GLU
    Stim_GLU[i] = So + G_GLU * LfmGLU[i]
    
    #HAM 
    Stim_HAM[i] = So + G_HAM * LfmHAM[i]
    
    #TA
    Stim_TA[i] = So + G_TA * LlceTA[i]


#verification external force/ calcul de la force normal ok

class GRF:
    def __init__(self, xcp, xcp_dot, x0, zcp, zcp_dot,z0=1.9, kx=8200, kz=78480, musl=0.8, must=0.9, vmax= 0.03):
        
        
        self.xcp = xcp
        self.zcp = zcp
        self.xcp_dot = xcp_dot
        self.zcp_dot = zcp_dot
        self.x0 = x0
        self.z0 = z0
        self.kx = kx
        self.kz = kz
        self.musl = musl
        self.must = must
        self.vmax = vmax
        
    
    def vertical_force(self):
        
        delta_zcp = self.zcp - self.z0
        
        return self.kz * -delta_zcp *(1+self.zcp_dot/self.vmax)
        

    def sliding_force(self,F_normal):
        
        return math.copysign(1, self.xcp_dot) * self.musl * F_normal

    def stiction_force(self):
        
        delta_xcp = self.xcp-self.x0
        delta_xcp_dot = self.xcp_dot / self.vmax
        
        return -self.kx * delta_xcp * (1 + math.copysign(1, delta_xcp) * delta_xcp_dot)

    def forces(self):
        
        F_normal = self.vertical_force()
        
        if abs(self.xcp_dot) < (self.vmax/100):
            f_st = self.stiction_force()
            if f_st >= self.must * F_normal:
                return F_normal,f_st
            else:
                return F_normal,0
        else:
            return F_normal, self.sliding_force(F_normal)

GRForce_normal = GRF(x_fn[0],vx_fn[0],pos_FP,z_fn[0],vz_fn[0])
force_normal = np.zeros(100)
sliding_force = np.zeros(100)
stiction_force = np.zeros(100)
for i in range(100):
    GRForce_normal.zcp = z_fn[i]
    GRForce_normal.zcp_dot = vz_fn[i]
    GRForce_normal.xcp = x_fn[i]
    GRForce_normal.xcp_dot = vx_fn[i]
    force_normal[i] = GRForce_normal.forces()[0]
    

#verification external forces : sliding force ok
    sliding_force[i] = GRForce_normal.sliding_force(force_normal[i])

#verification external forces : stiction force ok
    stiction_force[i] = GRForce_normal.stiction_force()
    
# plot 
import matplotlib.pyplot as plt
plt.plot(time,stiction_force)
