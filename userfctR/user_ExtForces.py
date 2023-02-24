# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019

import numpy as np
import math

class GRF:
    def __init__(self, xcp, xcp_dot, x0, zcp, zcp_dot,z0=1.9, kx=7848, kz=78480, musl=0.8, must=0.9, vmax=0.03):
        
        
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
        
        if self.zcp_dot/self.vmax >= -1:
        
            return self.kz * -delta_zcp *(1+self.zcp_dot/self.vmax)
        
        else:
            return 0
        

    def sliding_force(self,F_normal):
        
        if self.xcp_dot != 0 :
            return math.copysign(1, self.xcp_dot) * self.musl * F_normal
        else :
            return 0

    def stiction_force(self):
        
        delta_xcp = self.xcp-self.x0
        delta_xcp_dot = self.xcp_dot / self.vmax
        
        return -self.kx * delta_xcp * (1 + math.copysign(1, delta_xcp) * delta_xcp_dot)


def Stiction_flipflop(stick,slide):
    if stick == True and slide == False:
        return True 
    elif stick == False and slide == True:
        return False 
    else :
        raise NameError('Problems with the flip-flop function')

stiction_br = False
stiction_br_prec = True
stick_br = False
slide_br = False

stiction_bl = False
stiction_bl_prec = True
stick_bl = False
slide_bl = False

stiction_hr = False
stiction_hr_prec = True
stick_hr = False
slide_hr = False

stiction_hl = False
stiction_hl_prec = True
stick_hl = False
slide_hl = False

x0_BallR=0 #Position x ou la stiction est engagée pour le ball R
x0_BallL=0 #Position x ou la stiction est engagée pour le ball L 
x0_HeelR=0 #Position x ou la stiction est engagée pour le heel R
x0_HeelL=0 #Position x ou la stiction est engagée pour le heel L

time = np.zeros(1)


def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Compute an user-specified external force.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from the inertial frame to the
        force sensor frame: Frame_sensor = RxF[1:4,1:4] * Frame_inertial
    VxF : numpy.ndarray
        Velocity vector (index starting at 1) of the force sensor expressed in
        the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Angular velocity vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Acceleration vector (index starting at 1) of the force sensor expressed
        in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Angular acceleration vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed force sensor.

    Notes
    -----
    For 1D numpy.ndarray with index starting at 1, the first index (array[0])
    must not be modified. The first index to be filled is array[1].

    For 2D numpy.ndarray with index starting at 1, the first row (mat[0, :]) and
    line (mat[:,0]) must not be modified. The subarray to be filled is mat[1:, 1:].

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 10 equal to [0., Fx, Fy, Fz, Mx, My, Mz, dxF].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
        dxF is an array of length 3 containing the component of the forces/torque
        application point expressed in the BODY-FIXED frame.
        This array is a specific line of MbsData.SWr.
    """

    #initialisation des parametres :
        
    Fx = 0.0
    Fy = 0.0
    Fz = 0.0
    Mx = 0.0
    My = 0.0
    Mz = 0.0
    
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]
    
    
    ground_limit = 1.9 #taille des jambes tendues
    v_limit = 0.01 # vitesse en dessous de laquelle on est en stiction 
    
    #initialisation des sensors de forces externes:
    
    Force_BallR = mbs_data.extforce_id["Force_BallR"]
    Force_BallL = mbs_data.extforce_id["Force_BallL"]
    Force_HeelR = mbs_data.extforce_id["Force_HeelR"]
    Force_HeelL = mbs_data.extforce_id["Force_HeelL"]
    
    #global parameters :
    
    global stiction_br
    global stiction_br_prec
    global slide_br
    global stick_br
    global stiction_bl
    global stiction_bl_prec
    global slide_bl
    global stick_bl
    global stiction_hr
    global stiction_hr_prec
    global slide_hr
    global stick_hr
    global stiction_hl
    global stiction_hl_prec
    global slide_hl
    global stick_hl
        
    global x0_BallR
    global x0_BallL
    global x0_HeelR
    global x0_HeelL
    
    #compute of the time step :
    
    global time
    dt= np.round(tsim - time[-1], decimals=4)
    time = np.append(time,tsim)
    
    #calcul des forces :
    
    ### BallR
    
    if ixF == Force_BallR:
        
        if PxF[3]- ground_limit >=0 :
                  
            GRForce_BallR = GRF(PxF[1],VxF[1],x0_BallR,PxF[3],VxF[3])
            if stiction_br == True : # on est en stiction --> Static friction model 
                Fz = GRForce_BallR.vertical_force()
                Fx = GRForce_BallR.stiction_force()
                
                if abs(Fx)-abs(Fz*GRForce_BallR.must) >= 0:
                    slide_br=True
                    stick_br=False
                else :
                    stick_br = True
                    slide_br = False
                
                stiction_br = stiction_br_prec
                stiction_br_prec = Stiction_flipflop(stick_br, slide_br)
            
            else : # on est en sliding --> Kinetic friction model
                
                Fz = GRForce_BallR.vertical_force()
                Fx = GRForce_BallR.sliding_force(Fz)
                x0_BallR = PxF[1]
                
                if abs(VxF[1]) - v_limit <= 0:
                    stick_br = True
                    slide_br = False
                else :
                    stick_br = False
                    slide_br = True
                
                stiction_br = stiction_br_prec
                stiction_br_prec = Stiction_flipflop(stick_br, slide_br)
        
        else :
            
            Fz = 0
            Fx = 0
            stiction_br = False
            stick_br = False
            slide_br = False
                
    ### BallL
        
    if ixF == Force_BallL:
        
        
        if PxF[3] - ground_limit >=0 :
            
            GRForce_BallL = GRF(PxF[1],VxF[1],x0_BallL,PxF[3],VxF[3])
            if stiction_bl == True : # on est en stiction --> Static friction model 
                Fz = GRForce_BallL.vertical_force()
                Fx = GRForce_BallL.stiction_force()
                
                if abs(Fx)-abs(Fz*GRForce_BallL.must) >= 0:
                    slide_bl=True
                    stick_bl=False
                else :
                    stick_bl = True
                    slide_bl = False
                
                stiction_bl = stiction_bl_prec
                stiction_bl_prec = Stiction_flipflop(stick_bl, slide_bl)
            
            else : # on est en sliding --> Kinetic friction model
                
                Fz = GRForce_BallL.vertical_force()
                Fx = GRForce_BallL.sliding_force(Fz)
                x0_BallL = PxF[1]
                
                if abs(VxF[1]) - v_limit <= 0:
                    stick_bl = True
                    slide_bl = False
                else :
                    stick_bl = False
                    slide_bl = True
                
                stiction_bl = stiction_bl_prec
                stiction_bl_prec = Stiction_flipflop(stick_bl, slide_bl)
        
        else :
            
            Fz = 0
            Fx = 0
            stiction_bl = False
            stick_bl = False
            slide_bl = False
    
    ### HeelR
    
    if ixF == Force_HeelR:
        
     
        if PxF[3]- ground_limit >=0 :
            
            GRForce_HeelR = GRF(PxF[1],VxF[1],x0_HeelR,PxF[3],VxF[3])
            if stiction_hr == True : # on est en stiction --> Static friction model 
                Fz = GRForce_HeelR.vertical_force()
                Fx = GRForce_HeelR.stiction_force()
                
                if abs(Fx)-abs(Fz*GRForce_HeelR.must) >= 0:
                    slide_hr=True
                    stick_hr=False
                else :
                    stick_hr = True
                    slide_hr = False
                
                stiction_hr = stiction_hr_prec
                stiction_hr_prec = Stiction_flipflop(stick_hr, slide_hr)
            
            else : # on est en sliding --> Kinetic friction model
                
                Fz = GRForce_HeelR.vertical_force()
                Fx = GRForce_HeelR.sliding_force(Fz)
                x0_HeelR = PxF[1]
                
                if abs(VxF[1]) - v_limit <= 0:
                    stick_hr = True
                    slide_hr = False
                else :
                    stick_hr = False
                    slide_hr = True
                
                stiction_hr = stiction_hr_prec
                stiction_hr_prec = Stiction_flipflop(stick_hr, slide_hr)
        
        else :
            
            Fz = 0
            Fx = 0
            stiction_hr = False
            stick_hr = False
            slide_hr = False
    
    ### HeelL
        
    if ixF == Force_HeelL:
        
        
        if PxF[3]- ground_limit >=0 :
                        
            GRForce_HeelL = GRF(PxF[1],VxF[1],x0_HeelL,PxF[3],VxF[3])
            if stiction_hl == True : # on est en stiction --> Static friction model 
                Fz = GRForce_HeelL.vertical_force()
                Fx = GRForce_HeelL.stiction_force()
                
                if abs(Fx)-abs(Fz*GRForce_HeelL.must) >= 0:
                    slide_hl=True
                    stick_hl=False
                else :
                    stick_hl = True
                    slide_hl = False
                
                stiction_hl = stiction_hl_prec
                stiction_hl_prec = Stiction_flipflop(stick_hl, slide_hl)
            
            else : # on est en sliding --> Kinetic friction model
                
                Fz = GRForce_HeelL.vertical_force()
                Fx = GRForce_HeelL.sliding_force(Fz)
                x0_HeelL = PxF[1]
                
                if abs(VxF[1]) - v_limit <= 0:
                    stick_hl = True
                    slide_hl = False
                else :
                    stick_hl = False
                    slide_hl = True
                
                stiction_hl = stiction_hl_prec
                stiction_hl_prec = Stiction_flipflop(stick_hl, slide_hl)

    
        else :
            
            Fz = 0
            Fx = 0
            stiction_hl = False
            stick_hl = False
            slide_hl = False

    
    
    Swr=np.zeros(9+1)
    Swr[1:]=np.r_[Fx,Fy,Fz,Mx,My,Mz,dxF]
    
    return Swr
