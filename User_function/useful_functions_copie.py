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
    


def interpolation_memory(matrix, t_search):
    
    nb_rows = matrix.shape[0] if matrix.ndim > 1 else 1
    
    times = matrix[:, -1]
    index = np.searchsorted(times, t_search)
    
    if index == 0:
        # t_search est inférieur au temps de simulation de la première ligne
        interpolated_vars = matrix[0, :-1]
    elif index == nb_rows:
        # t_search est supérieur au temps de simulation de la dernière ligne
        interpolated_vars = matrix[-1, :-1]
    else:
        # Interpolation linéaire entre les valeurs de deux lignes successives
        lower_row = index - 1
        upper_row = index
        lower_time = times[lower_row]
        upper_time = times[upper_row]
        lower_vars = matrix[lower_row, :-1] 
        upper_vars = matrix[upper_row, :-1]
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
        
        angle = 0.087441
        
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

# le tableau s qui est mis en entrée est un tableau avec 2 colonnes, une correspondant au valeur du signal et une correspondant au temps de simulation


def filter_signal(s, num, den):
    
    # Extraire les valeurs et les temps de simulation à partir du tableau d'entrée
    x = s[:,0]
    t = s[:,1]
   
    # Échantillonner les valeurs sur une grille régulière
    t_interp = np.linspace(t.min(), t.max(), len(t))
    x_interp = np.interp(t_interp, t, x)

    # Appliquer le filtre linéaire
    y = signal.lfilter(num, den, x_interp)

    # Réinterpoler les valeurs filtrées sur la grille non régulière d'origine
    y_interp = np.interp(t, t_interp, y)

    # Créer un tableau numpy de deux colonnes pour le signal filtré
    filtered_signal = np.column_stack((y_interp, t))

    return filtered_signal
    
    
def pop_malone(memory):
    if - memory[0,-1] + memory[-1,-1] > 0.025 and len(np.shape(memory)) !=1 :
        memory = np.delete(memory,0,axis=0)
     
    return memory
