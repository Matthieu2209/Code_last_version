#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 10 14:38:35 2022

@author: matthieuxaussems
"""

###validation des résultats intermédiaires 

import numpy as np
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

for i in range (100):
    if i <=9 :
        
        Stim[i] = 0.08
        lce[i] = 2*0.08
        lse[i] = 2*0.23
        fv_C1[i] = 0.1
        fv_C2[i] = 1.1
        phiknee[i] = 2
        
        
    elif i <= 19 :
        
        Stim[i] = 0.25
        lce[i] = 1.8*0.08
        lse[i] = 1.8*0.23
        fv_C1[i] = 0.2
        fv_C2[i] = 1.2
        phiknee[i] = 2.1
        
    elif i <=29 :
        
        Stim[i] = 0.08
        l[i] = 1.6
        lce[i] = 1.6*0.08
        lse[i] = 1.6*0.23
        fv_C1[i] = 0.3
        fv_C2[i] = 1.3
        phiknee[i] = 2.2
    
    elif i <= 39 :
        
        Stim[i] = 0.25
        l[i] = 1.4
        lce[i] = 1.4*0.08
        lse[i] = 1.4*0.23
        fv_C1[i] = 0.4
        fv_C2[i] = 1.4
        phiknee[i] = 2.3
    
    elif i <= 49 :
        
        Stim[i] = 0.08
        l[i] = 1.2
        lce[i] = 1.2*0.08
        lse[i] = 1.2*0.23
        fv_C1[i] = 0.5
        fv_C2[i] = 1.5
        phiknee[i] = 2.4

    elif i <= 59 :
        
        Stim[i] = 0.25
        l[i] = 0.4
        lce[i] = 1*0.08
        lse[i] = 1*0.23
        fv_C1[i] = 0.5
        fv_C2[i] = 1.5
        phiknee[i] = 2.5
    
    elif i <= 69 :
        
        Stim[i] = 0.08
        lce[i] = 1.2*0.08
        lse[i] = 1.2*0.23
        fv_C1[i] = 0.6
        fv_C2[i] = 1.4
        phiknee[i] = 2.6
    
    elif i <= 79 :
        
        Stim[i] = 0.25
        lce[i] = 1.4*0.08
        lse[i] = 1.4*0.23
        fv_C1[i] = 0.7
        fv_C2[i] = 1.3
        phiknee[i] = 2.7
    
    elif i <= 89 :
        
        Stim[i] = 0.08
        lce[i] = 1.6*0.08
        lse[i] = 1.6*0.23
        fv_C1[i] = 0.8
        fv_C2[i] = 1.2
        phiknee[i] = 2.8
        
        
    elif i <= 99 :
        
        Stim[i] = 0.25
        lce[i] = 1.8*0.08
        lse[i] = 1.8*0.23
        fv_C1[i] = 0.9
        fv_C2[i] = 1.1
        phiknee[i] = 2.9
        
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


    
# plot 
import matplotlib.pyplot as plt
plt.plot(time,lmtu)
