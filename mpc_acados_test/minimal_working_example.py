### KINEMATIC MODEL FOR THE TT02



from acados_template import AcadosModel
import casadi as ca
import os
from pathlib import Path
import numpy as np


class AckermannCar:

    def __init__(self, L=0.257,track="su_bigmap_splined.txt", dt=1/12.):
        self.L = L 

        #Motor params. To be identified
        self.m = 2.334
        self.Cm = 2.3
        self.lf = L/2.
        self.lr = L/2. 
        self.Cr0 = 0.0518
        self.Cr2 = 0.00035

        self.model = self.create_model(track,dt)

    def create_model(self, track="su_bigmap_splined.txt", dt=1/12.):


        #statedot = xdot ydot thetadot vxdot vydot omegadot thetaAdot deltadot Ddot vthetadot

        vx = ca.MX.sym("vx") #vx in car frame
        vy = ca.MX.sym("vy") #vy in car frame
        omega = ca.MX.sym("omega") #thetadot, angular velocity
        thetaA = ca.MX.sym("thetaA") #estimated progress along track (s_estim)
        delta = ca.MX.sym("delta") #steering angle 
        D = ca.MX.sym("D") #duty cycle
        vtheta = ca.MX.sym("vtheta") #derivative of thetaA
        
        x_state = ca.vertcat(vx,delta,D) #State vector


        derDelta = ca.MX.sym("derDelta") #Steering rate
        derD = ca.MX.sym("derD") #Duty cycle rate
        derVtheta = ca.MX.sym("derVtheta") #Derivative of Vtheta

        u_control = ca.vertcat(derDelta, derD) #Control vector


        #State dot vector

        #Even though some of these are already defined in the state or control vectors, we need to define them and then equate them, for the symbolic computations.
        #I think. At least that's how I've seen everyone do it, and how I've always done it. Haven't tried it the direct way tbh. Might be a stupid sheep. 

    
        vx_dot = ca.MX.sym("vx_dot")
        vy_dot = ca.MX.sym("vy_dot")
        omega_dot = ca.MX.sym("omega_dot")
        thetaA_dot = ca.MX.sym("thetaA_dot")
        delta_dot = ca.MX.sym("delta_dot")
        D_dot = ca.MX.sym("D_dot")
        vtheta_dot = ca.MX.sym("vtheta_dot")

        x_state_dot = ca.vertcat(vx_dot, delta_dot, D_dot)



        # If youre confused about thetaA, vtheta, or derVtheta:

        # The position of the car in the track's Frenet frame is computationnally hard to compute at each timestep. As such, we approximate
        # the position (s) along the track with the variable Theta, which we integrate over and over. The MPC will try to both maximize a_theta
        # (aka vtheta_dot) and minimize the distance between the estimation and the actual position of the car. 

        
        #Kinematics: x_state_dot = f(x_state, u_control)
        Fx = self.Cm*D #- self.Cr0 - self.Cr2*vx*vx
        f_expl = ca.vertcat(
            Fx/self.m,
            derDelta,
            derD,
        )
        

        # Add penalty terms for delta and delta_dot
        kv = 40.0
        kd = 10.0  # New weighting for delta
        J = kv * (vx - 4.5)**2 + kd * (delta)**2
        Je = kv * 10 * (vx - 4.5)**2 + kd * 5 * (delta)**2


        cost_expr = J  # + R# - ktheta*thetaA

        #Constraint to respect track width
        # a_lat = 0.5 * vx * vx * delta + Fx * ca.sin(3.9 * delta) / self.m
        # circle_func = (x - x_ref_s(thetaA))**2 + (y - y_ref_s(thetaA))**2

        # lh_constraints = ca.vertcat(circle_func, a_lat, Fx/self.m)

        # Build the AcadosModel
        model = AcadosModel()
        model.f_expl_expr = f_expl

        # For an explicit model: f_impl_expr = xdot - f_expl
        # either form is fine as long as it's consistent
        model.f_impl_expr = ca.vertcat(vx_dot, delta_dot, D_dot) - f_expl

        # model.params = params

        model.x = x_state
        model.u = u_control
        model.xdot = x_state_dot
        model.name = "tt02"

        # Time step

        # Define the continuous dynamics
        f = ca.Function('f', [x_state, u_control], [f_expl])

        # RK4 Integration
        x_k = ca.MX.sym("x_k", x_state.size1())  # Current state
        u_k = ca.MX.sym("u_k", u_control.size1())  # Control input


        k1 = f(x_k, u_k)
        k2 = f(x_k + dt / 2 * k1, u_k)
        k3 = f(x_k + dt / 2 * k2, u_k)
        k4 = f(x_k + dt * k3, u_k)

        x_next = x_k + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # Create a CasADi function for the discrete-time model
        self.f_discrete = ca.Function('f_discrete', [x_k, u_k], [x_next])

        return model, cost_expr, J, Je

    def integration(self, state, control):
        state = self.f_discrete(state, control)
        # state[3] = min(max(0.05, state[3]), 3.0)
        # # state[4] = min(max(-3.0, state[4]), 3.0)
        # state[5] = min(max(-8.0, state[5]), 8.0)
        # state[6] = min(max(-1.0, state[6]), 100.0)
        # state[7] = min(max(-0.4, state[7]), 0.4)
        # state[8] = min(max(-0.1, state[8]), 1.0)
        # state[9] =  min(max(0.05, state[9]), 3.0)
        return state


    
if __name__ == "__main__":
    tt02 = AckermannCar(L=0.257)

    state = [0.,0.,0.,0.,0.,0.,0.]
    control = [0.,1.,0.] #derDelta, derD, derVtheta


    for i in range(200):
        control[1] = 0.0      # derD = 0
        D = 1.0
        state[8] = D          # force duty cycle = 1
        state = tt02.integration(state, control)
        print(i, state)
