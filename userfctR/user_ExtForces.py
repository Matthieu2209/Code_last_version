# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019

import numpy as np
import math

class GRF:
    def __init__(self, xcp, xcp_dot, x0, zcp, zcp_dot,z0=1.9, kx=8200, kz=78480, musl=0.8, must=0.9, vmax=0.03):
        
        
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
        
        return -math.copysign(1, self.xcp_dot) * self.musl * F_normal

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

stiction_br = False
stiction_bl = False
stiction_hr = False
stiction_hl = False

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
    
    #initialisation des sensors de forces externes:
    
    Force_BallR = mbs_data.extforce_id["Force_BallR"]
    Force_BallL = mbs_data.extforce_id["Force_BallL"]
    Force_HeelR = mbs_data.extforce_id["Force_HeelR"]
    Force_HeelL = mbs_data.extforce_id["Force_HeelL"]
    
    #global parameters :
    
    global stiction_br
    global stiction_bl
    global stiction_hr
    global stiction_hl
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
            
            if tsim==0:
                stiction_br =[True]
            elif dt!=0 :
                stiction_br.append(True)
                
                if stiction_br[-1] and not stiction_br[-2] :
                    
                    x0_BallR=PxF[1]
            
            GRForce_BallR = GRF(PxF[1],VxF[1],x0_BallR,PxF[3],VxF[3])
            Fz = GRForce_BallR.forces()[0]
            Fx = GRForce_BallR.forces()[1]
        
        else :
            
            if tsim==0:
                stiction_br = [False]
            elif dt!=0 :
                stiction_br.append(False)
                
    ### BallL
        
    if ixF == Force_BallL:
        
        
        if PxF[3] - ground_limit >=0 :
            
            if tsim==0:
                stiction_bl =[True]
            elif dt!=0 :
                stiction_bl.append(True)
                
                if stiction_bl[-1] and not stiction_bl[-2] :
                    
                    x0_BallL=PxF[1]
                    
            
            GRForce_BallL = GRF(PxF[1],VxF[1],x0_BallL,PxF[3],VxF[3])
            Fz = GRForce_BallL.forces()[0]
            Fx = GRForce_BallL.forces()[1]
            
        
        else :
            
            if tsim==0:
                stiction_bl = [False]
            elif dt!=0 :
                stiction_bl.append(False)
    
    ### HeelR
    
    if ixF == Force_HeelR:
        
     
        if PxF[3]- ground_limit >=0 :
            
            if tsim==0:
                stiction_hr =[True]
            elif dt!=0 :
                stiction_hr.append(True)
                
                if stiction_hr[-1] and not stiction_hr[-2] :
                    
                    x0_HeelR=PxF[1]
            
            GRForce_HeelR = GRF(PxF[1],VxF[1],x0_HeelR,PxF[3],VxF[3])
            Fz = GRForce_HeelR.forces()[0]
            Fx = GRForce_HeelR.forces()[1]
        
        else :
            
            if tsim==0:
                stiction_hr = [False]
            elif dt!=0 :
                stiction_hr.append(False)
    
    ### HeelL
        
    if ixF == Force_HeelL:
        
        
        if PxF[3]- ground_limit >=0 :
            
            if tsim==0:
                stiction_hl =[True]
            elif dt!=0 :
                stiction_hl.append(True)
                
                if stiction_hl[-1] and not stiction_hl[-2] :
                    
                    x0_HeelL=PxF[1]
            
            GRForce_HeelL = GRF(PxF[1],VxF[1],x0_HeelL,PxF[3],VxF[3])
            Fz = GRForce_HeelL.forces()[0]
            Fx = GRForce_HeelL.forces()[1]

    
        else :
            
            if tsim==0:
                stiction_hl = [False]
            elif dt!=0 :
                stiction_hl.append(False)
    

    
    
    Swr=np.zeros(9+1)
    Swr[1:]=np.r_[Fx,Fy,Fz,Mx,My,Mz,dxF]
    
    return Swr
