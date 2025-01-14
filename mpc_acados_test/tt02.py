### KINEMATIC MODEL FOR THE TT02


from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import os
from pathlib import Path
import numpy as np


def getTrack(filename):
    track_file = os.path.join(str(Path(__file__).parent), filename)
    # array=np.loadtxt(track_file, delimiter=",")
    array= np.loadtxt(track_file)
    sref=array[:,0]
    xref=array[:,1]
    yref=array[:,2]
    psiref=array[:,3]
    kapparef=array[:,4]
    return sref,xref,yref,psiref, kapparef


class AckermannCar:

    def __init__(self, L=0.257):
        self.L = L 

        # we consider the CoM to be centered between the two axles. 
        self.lf = L/2.
        self.lr = L/2. 
        self.m = 2.334

        #Motor params. To be identified
        self.Cm = 0.5
        self.Cr0 = 0.0
        self.Cr2 = 0.0

    def create_model(self, track="su_bigmap_splined.txt"):

        #statedot = xdot ydot thetadot vxdot vydot omegadot thetaAdot deltadot Ddot vthetadot

        x = ca.MX.sym("x") #x in track frame
        y = ca.MX.sym("y") #y in track frame
        theta = ca.MX.sym("theta") #heading error (i think)
        vx = ca.MX.sym("vx") #vx in car frame
        vy = ca.MX.sym("vy") #vy in car frame
        omega = ca.MX.sym("omega") #thetadot, angular velocity
        thetaA = ca.MX.sym("thetaA") #estimated progress along track (s_estim)
        delta = ca.MX.sym("delta") #steering angle 
        D = ca.MX.sym("D") #duty cycle
        vtheta = ca.MX.sym("vtheta") #derivative of thetaA
        
        x_state = ca.vertcat(x,y,theta,vx,vy,omega,thetaA,delta,D,vtheta) #State vector


        derDelta = ca.MX.sym("derDelta") #Steering rate
        derD = ca.MX.sym("derD") #Duty cycle rate
        derVtheta = ca.MX.sym("derVtheta") #Derivative of Vtheta

        u_control = ca.vertcat(derDelta, derD, derVtheta) #Control vector


        #State dot vector

        #Even though some of these are already defined in the state or control vectors, we need to define them and then equate them, for the symbolic computations.
        #I think. At least that's how I've seen everyone do it, and how I've always done it. Haven't tried it the direct way tbh. Might be a stupid sheep. 

        x_dot = ca.MX.sym("x_dot")
        y_dot = ca.MX.sym("y_dot")
        theta_dot = ca.MX.sym("theta_dot")
        vx_dot = ca.MX.sym("vx_dot")
        vy_dot = ca.MX.sym("vy_dot")
        omega_dot = ca.MX.sym("omega_dot")
        thetaA_dot = ca.MX.sym("thetaA_dot")
        delta_dot = ca.MX.sym("delta_dot")
        D_dot = ca.MX.sym("D_dot")
        vtheta_dot = ca.MX.sym("vtheta_dot")

        x_state_dot = ca.vertcat(x_dot, y_dot, theta_dot, vx_dot, vy_dot, omega_dot, thetaA_dot, delta_dot, D_dot, vtheta_dot)


        # If youre confused about thetaA, vtheta, or derVtheta:

        # The position of the car in the track's Frenet frame is computationnally hard to compute at each timestep. As such, we approximate
        # the position (s) along the track with the variable Theta, which we integrate over and over. The MPC will try to both maximize a_theta
        # (aka vtheta_dot) and minimize the distance between the estimation and the actual position of the car. 

        
        #Kinematics: x_state_dot = f(x_state, u_control)
        Fx = self.Cm*D - self.Cr0 - self.Cr2*vx*vx
        f_expl = ca.vertcat(
            vx * ca.cos(theta) - vy * ca.sin(theta),
            vx * ca.sin(theta) + vy * ca.cos(theta),
            omega,
            Fx/self.m,
            (delta_dot * vx + delta*vx_dot) * (self.lr/self.L),
            (delta_dot * vx + delta*vx_dot) * (1/self.L),
            vtheta,
            derDelta,
            derD,
            derVtheta
        )

        
        # Build the AcadosModel
        model = AcadosModel()
        model.f_expl_expr = f_expl
        # For an explicit model: f_impl_expr = xdot - f_expl
        # either form is fine as long as it's consistent
        model.f_impl_expr = ca.vertcat(s_dot, e_dot, alpha_dot, delta_dot) - f_expl
        # model.f_impl_expr = ca.vertcat(s_dot, v_dot) - f_expl

        # model.params = params

        model.x = x
        model.u = u
        model.xdot = x_dot
        model.name = "kinematic_bicycle"

        constraint = ca.types.SimpleNamespace()

        constraint.delta_min = -0.4
        constraint.delta_max = 0.4

        constraint.steering_rate_min = -12
        constraint.steering_rate_max = 12 

        constraint.v_min = -5.0
        constraint.v_max = 5.0


        return model, constraint

    def kinematics(self, state, control, dt=1/12.):
        x, y, theta, beta = state
        

        v = min(max(control[0], -5.0), 5.0)
        steering_rate = min(max(control[1], -10), 10)

        xdot = v * math.cos(theta)
        ydot = v * math.sin(theta)
        thetadot = v/self.L * math.tan(beta)
        betadot = steering_rate

        x = x + dt * xdot
        y = y + dt * ydot
        theta = theta + dt * thetadot
        beta = beta + dt * betadot

        beta = min(max(beta, -0.4), 0.4)

        return [x,y,theta,beta]
    
if __name__ == "__main__":
    tt02 = AckermannCar(L=0.257)

    state = [0.,0.,0.,0.]
    control = [1.0,0.]


    for i in range(12):
        state = tt02.kinematics(state,control)
        print(i, state)

