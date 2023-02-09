#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 10 13:51:51 2022

@author: matthieuxaussems
"""

import numpy as np
import math
from scipy import signal 

################################ Usefuls functions ####################################################################################

# 1) Low filter du premier ordre qui permet de résoudre l'équation différentielle de 1er ordre de l'excitation-contraction coupling permettant d'obtenir
# l'activation musculaire à partir de la stimulation musculaire
#
# parametre in : stimulation,constante de temps,temps actuel
# parametre out : activation
#


def low_filter(stimulation,tau,diff_t,last_activation):

    if diff_t==0 : #lorsque current_t=last_t alors l'activation est égale à la stimulation car delta-t = 0 dans l'équation différentielle ce qui revient
                   #à Stimulation(t)-activation(t)=0 donc stimulation=activation
        activation = stimulation
        return activation
    
    else :
        f = diff_t/tau
        frac = 1/(1+f)
        activation = f*frac*stimulation +frac*last_activation
        return activation
    


def interpolation_memory(matrix,diff_t,t_search):
    
    nb_rows = matrix.shape[0] if matrix.ndim > 1 else 1
    current_time = 0
    lower_row = -1
    upper_row = -1

    for i in range(nb_rows):
        if current_time <= t_search and t_search < current_time + diff_t:
            lower_row = i
            upper_row = i + 1
            break
        current_time += diff_t
        

    lower_time = current_time
    upper_time = current_time + diff_t
    
    if upper_row == nb_rows:
        
        upper_row -= 1
        lower_row -= 1
        lower_time = current_time - diff_t
        upper_time = current_time

    lower_vars = matrix[lower_row] 
    upper_vars = matrix[upper_row]

    interpolated_vars = lower_vars + (t_search - lower_time) * (upper_vars - lower_vars) / (upper_time - lower_time)
    
    return interpolated_vars
    

def pressure_sheet(p,v):
    
    k_pressure = 104967
    v_max_pressure = 0.5
    
    u1 = p*k_pressure
    u2 = v/v_max_pressure
    
    F_reaction = -u1 *(1+math.copysign(1, u1)*u2)
    
    return F_reaction



"""
    fonction qui permet de determiner l angle que fait le bust par rapport à la verticale
    
    input : position de l'articulation de la hanche et d'un point sur le buste grace aux senseurs dans le MBsyspad
            sensor_hip = PxF du hip tq PxF = [0,x,y,z]
    out : angle en radian entre le buste et la verticale
"""

def trunk_angle(P_sensor_hip, P_sensor_trunk,tsim):
    
    if tsim ==0 :
        
        angle = 0
        
    else : 
    
        xa, ya, za = P_sensor_hip[1:]
        xb, yb, zb = P_sensor_trunk[1:]
        
        xc = xa
        zc = za - 0.8
        
        
        # Calculer les vecteurs AB et BC
        ab_x = xb - xa
        ab_z = za - zb
        
        
        ac_x = 0
        ac_z = za - zc
        
          
        # Calculer la longueur des vecteurs
        ab_length = math.sqrt(ab_x**2 + ab_z**2 ) 
        ac_length = math.sqrt(ac_x**2 + ac_z**2 ) 
          
        # Calculer le produit scalaire des vecteurs AB et BC
        dot_product = ab_x * ac_x + ab_z * ac_z
        
        
        # Calculer l'angle en radians entre les deux vecteurs
        
        if dot_product == 0 :
            
            angle = np.pi/2
        
        else :
            
            angle = math.acos(dot_product / (ab_length * ac_length))
      
      
    return angle




def dt_angle(angle_t, angle_0, dt):
    
  dt_angle = (angle_t - angle_0) / dt
  
  return dt_angle


def limit_range(x,min,max): #fonction permettant de borner une variable
    if x<min :
        return min
    if x>max :
        return max
    else : return x


def filter_signal(s, num, den):
    
    return signal.lfilter(num, den, s)


