### KINEMATIC MODEL FOR THE TT02



from acados_template import AcadosModel
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

    def __init__(self, L=0.257,track="su_bigmap_splined.txt", dt=1/12.):
        self.L = L 

        # we consider the CoM to be centered between the two axles. 
        self.lf = L/2.
        self.lr = L/2. 
        self.m = 2.334

        #Motor params. To be identified
        self.Cm = 2.3
        self.Cr0 = 0.0518
        self.Cr2 = 0.00035

        # "Cm1" : 0.287,
        # "Cm2" : 0.0545,

        # "Cr0" : 0.0518,
        # "Cr2" : 0.00035,
        self.model = self.create_model(track,dt)

    def create_model(self, track="su_bigmap_splined.txt", dt=1/12.):

        # load track parameters
        [s_ref, x_ref, y_ref, psi_ref, kappa_ref] = getTrack(track)
        
        # THREE INTERPOLANTS
        # Psi, X, Y

        # Pad out the variables start and end to get a smooth transition. 
        # We'll not care about the padded bits cause they'll be redundant, but it's to have 
        # a continuity between L and 0 (track length L, looping, yk)


        #Step 1: 1,2,3,4 -> 1,2,3,4,1,2,3,4, for x, y and psi
        #        1,2,3,4 -> 1,2,3,4,5,6,7,8  for s
        s_ref = np.append(s_ref, s_ref[-1] + s_ref[1:])
        x_ref_s = np.append(x_ref, x_ref[:-1])
        y_ref_s = np.append(y_ref, y_ref[:-1])
        psi_ref_s = np.append(psi_ref, psi_ref[:-1])

        #Step 2: 1,2,3,4,1,2,3,4 -> -3,-2,-1,0,1,2,3,4,1,2,3,4 for s
        
        s_ref = np.append(-s_ref[21] + s_ref[:21], s_ref)
        x_ref_s = np.append(x_ref_s[-21:], x_ref_s)
        y_ref_s = np.append(y_ref_s[-21:], y_ref_s)
        psi_ref_s = np.append(psi_ref_s[-21:], psi_ref_s)


        x_ref_s = ca.interpolant("x_ref_s", "linear", [s_ref], x_ref_s)
        y_ref_s = ca.interpolant("y_ref_s", "linear", [s_ref], y_ref_s)
        psi_ref_s = ca.interpolant("psi_ref_s", "linear", [s_ref], psi_ref_s)
        self.psi_ref_s = psi_ref_s
        self.x_ref_s = x_ref_s
        self.y_ref_s = y_ref_s
        #trackwidths

    
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
        Fx = self.Cm*D #- self.Cr0 - self.Cr2*vx*vx
        f_expl = ca.vertcat(
            vx * ca.cos(theta) - vy * ca.sin(theta),
            vx * ca.sin(theta) + vy * ca.cos(theta),
            omega,
            Fx/self.m,
            (derDelta * vx + delta*Fx/self.m) * (self.lr/self.L),
            delta * vx * (1/self.L),
            vtheta,
            derDelta,
            derD,
            derVtheta
        )
        
        #Cost functions formulation
        
        # Contouring error
        e_c = ca.sin(psi_ref_s(thetaA))*(x - x_ref_s(thetaA)) - ca.cos(psi_ref_s(thetaA))*(y - y_ref_s(thetaA))

        # Lag error
        e_l = -ca.cos(psi_ref_s(thetaA))*(x - x_ref_s(thetaA)) - ca.sin(psi_ref_s(thetaA))*(y - y_ref_s(thetaA))

        self.errors = ca.Function("errors", [x,y,thetaA], [e_c,e_l])

        #e_c = -(y - y_ref_s(0.6))
        #e_l = -(x-x_ref_s(0.6))
        # Weights - To Be Adjusted
        
        # # Contouring cost
        # J = qc*e_c**2 + ql*e_l**2 - kv * vtheta
        # Je = 10 * qc * e_c**2 + ql * e_l**2 - kv*vtheta

        q_c = 40.0
        q_l = 20.0
        kv = 10.0
        ktheta = 6.0

       

        # # Control input penalization
        Ru  = ca.diag([1e-2, 1e-4, 0]) #We want to maximize vtheta!

        Rdu = ca.diag([1e-1, 1e-1, 1e-4])


        #This is part of the state
        u   = ca.vertcat(delta, D, theta)


        J = e_c*e_c*q_c + e_l*e_l*q_l
        Je = e_c*e_c*q_c + e_l*e_l*q_l
        R = u_control.T @ Rdu @ u_control + u.T @ Ru @ u 




        cost_expr = J + R - ktheta*vtheta




        cost_function = ca.Function('cost_function', [x_state, u_control], [J, R, e_c,e_l, cost_expr])




        # Constraint to respect track width
        a_lat = 0.5 * vx * vx * delta + Fx * ca.sin(3.9 * delta) / self.m
        circle_func = (x - x_ref_s(thetaA))**2 + (y - y_ref_s(thetaA))**2

        lh_constraints = ca.vertcat(circle_func, a_lat, Fx/self.m)

        # Build the AcadosModel
        model = AcadosModel()
        model.f_expl_expr = f_expl

        # For an explicit model: f_impl_expr = xdot - f_expl
        # either form is fine as long as it's consistent
        model.f_impl_expr = ca.vertcat(x_dot, y_dot, theta_dot, vx_dot, vy_dot, omega_dot, thetaA_dot, delta_dot, D_dot, vtheta_dot) - f_expl

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

        return model, cost_expr, J, cost_function, Je, lh_constraints

    def integration(self, state, control):
        state = self.f_discrete(state, control)
        print(f"\n\nthetaA = {state[6]}, x_ref, y_ref {float(self.x_ref_s(state[6])), float(self.y_ref_s(state[6]))}, psi_ref = {float(self.psi_ref_s(state[6]))} e_c, e_l{self.errors(state[0], state[1], state[6])} \n\n ")
        e_c =self.errors(state[0], state[1], state[6])[0]
        e_l =self.errors(state[0], state[1], state[6])[1]
        return np.array(state), e_c, e_l

    
if __name__ == "__main__":
    tt02 = AckermannCar(L=0.257)

    state = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
    control = [0.,1.,0.] #derDelta, derD, derVtheta


    for i in range(200):
        control[1] = 0.0      # derD = 0
        D = 1.0
        state[8] = D          # force duty cycle = 1
        state = tt02.integration(state, control)
        print(i, state)
