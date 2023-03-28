#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  1 22:27:06 2022

@author: matthieuxaussems
"""

####TEST 1

import numpy as np
import matplotlib.pyplot as plt
    
###test 


Fm_memory = np.zeros(7)
ts = 0.005
tm = 0.01
tl= 0.02

Stim_S = np.zeros(101) #ts
Stim_VAS= np.zeros(101) #tm
Stim_SOL= np.zeros(101) #tl
time = np.zeros(101)
dt=0.001

for i in range(101):
    
    
    tsim = ((i)/1000)
    print(tsim-ts)
    
    time[i]=tsim
    
    
    # met dans une mémoire un vecteur différents pour chaque pas de temps pas de temps de 0.001 et allant jusque 0.1, c'est ce qui est sencé
    #se passer dans le code muscle actuation layer ou on ajoute une ligne à chaque fois que l'on ava,ce d'un pas de temps.

    if tsim >0 :
        
        Fm = np.ones(7)*((i)/1000)
        Fm_memory = np.vstack([Fm_memory,Fm])
    
    
    # utiliser les delay et permet de claculer la stimulation
    if tsim-ts <0:
        Stim_S[i] = 0
    else:
        Stim_S[i] = Fm_memory[int(np.round_((tsim-ts)/dt)),3]
    
    if tsim-tm <0:
        Stim_VAS[i] = 0
    else:
        Stim_VAS[i] = Fm_memory[int(np.round_((tsim-tm)/dt)),3]
    
    if tsim-tl <0:
        Stim_SOL[i] = 0
    else:
        Stim_SOL[i] = Fm_memory[int(np.round_((tsim-tl)/dt)),3]


# print(Stim_S)
# print(Stim_VAS)
# print(Stim_SOL)
# print(Fm_memory)

plt.plot(time,Stim_S)
plt.plot(time,Stim_VAS)
plt.plot(time,Stim_SOL)



# Stim_S=np.zeros(101)
# Stim_M=np.zeros(101)
# Stim_L=np.zeros(101)

# time= np.ones(101)

# for i in range(101):
#     time[i]=time[i]*i
#     if i-ts <= 0 : 
#         Stim_S[i]= 0
#     else :
#         Stim_S[i]= 2*Fm_memory[i-ts,0]
    
#     if i-tm <=0 :
#         Stim_M[i]= 0
#     else :
#         Stim_M[i]= 3*Fm_memory[i-tm,0]
#     if i-tl <=0 :
#         Stim_L[i]= 0
#     else :
#         Stim_L[i]= 4*Fm_memory[i-tl,0]
    
    

# plt.plot(time,Stim_M)
# plt.plot(time,Stim_L)






###TEST 2


# import numpy as np 

# Fm = np.zeros(7)

# dt=0.001
# tf= 2

# delay_long= 0.02
# delay_mid = 0.01
# delay_small = 0.005

# Stim_l_i = np.zeros(7)
# Stim_m_i = np.zeros(7)
# Stim_s_i = np.zeros(7)

# Stim_l = np.zeros(7)
# Stim_m = np.zeros(7)
# Stim_s = np.zeros(7)

# G=5

# time = np.linspace(0.001,2,2000)

# for i in range(2000):
#     Fm_new=np.zeros(7)
#     for j in range (7):
        
#         if i < delay_small/dt:
        
#             Stim_l[j] = Stim_l_i[j]
#             Stim_m[j] = Stim_m_i[j]
#             Stim_s[j] = Stim_s_i[j]
    
#         elif i < delay_mid/dt:
            
#             s = int(i-(delay_small/dt))
            
#             Stim_l[j] = Stim_l_i[j]
#             Stim_m[j] = Stim_m_i[j]
#             Stim_s[j] = Stim_s_i[j] + G*Fm[s,j]
            
#         elif i < delay_long/dt:
            
#             s = int(i-(delay_small/dt))
#             m = int(i-(delay_mid/dt))
            
#             Stim_l[j] = Stim_l_i[j]
#             Stim_m[j] = Stim_m_i[j] + G*Fm[m,j]
#             Stim_s[j] = Stim_s_i[j] + G*Fm[s,j]
        
#         else :
            
#             s = int(i-(delay_small/dt))
#             m = int(i-(delay_mid/dt))
#             l = int(i-(delay_long/dt))
            
#             Stim_l[j] = Stim_l_i[j] + G*Fm[l,j]
#             Stim_m[j] = Stim_m_i[j] + G*Fm[m,j]
#             Stim_s[j] = Stim_s_i[j] + G*Fm[s,j]
        
#         Fm_new[j]= i
#         Fm = np.vstack([Fm,Fm_new])
        


            
