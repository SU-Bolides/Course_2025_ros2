import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from tt02 import AckermannCar, getTrack
import math
from plot_track import plotAgainstTrack

class AcadosMPC:

    def __init__(self, track="su_bigmap_splined.txt", dt=1/50.):
        track_name = track.split(".txt")[0].split("_splined")[0]

        self.track = track_name + "_splined.txt"
        self.dt = dt
        self.tt02 = AckermannCar(0.257, self.track, dt)
        _,_,_,_,_,_, self.x_ref_s, self.y_ref_s = self.tt02.model
        pass

    def create_ocp_solver(self, Tf=1.0):

        N=int(Tf/self.dt)



        #state = x y theta vx vy omega thetaA delta D vtheta
        #control = delta_dot D_dot vtheta_dot

        model, cost_expr, J, cost_func, Je, lh_constraints, self.x_ref_s, self.y_ref_s = self.tt02.model

        ocp = AcadosOcp()
        ocp.model = model


        ocp.constraints.lbu = np.array([-10.,-10.,-4.]) #DeltaDot, DDot, VthetaDot
        ocp.constraints.ubu = np.array([10.,10.,40.])
        ocp.constraints.idxbu = np.array([0,1,2]) #Indices of the corresponding variables

        ocp.constraints.lbx = np.array([-10,-10,-10, -0.1, -3.0, -10.0, -1.0, -0.4, -1.0, 0.5])
        ocp.constraints.ubx = np.array([10,  10, 10,  7.0,  3.0, 10.0, 100, 0.4, 1.0, 20.0])
        ocp.constraints.idxbx = np.array(range(10))


        # ocp.constraints.lbx = np.array([-10,-10,-10,0.0,-0.5,-1.0,-10])
        # ocp.constraints.ubx = np.array([10, 10, 10, 100, 0.5, 1.0, 10 ])
        # ocp.constraints.idxbx = np.array([3,4,5,6,7,8,9])


        # ocp.constraints.lbx = np.array([-0.5])
        # ocp.constraints.ubx = np.array([0.5])
        # ocp.constraints.idxbx = np.array([7])



        ocp.constraints.lsbx = np.ones(2)*0.1
        ocp.constraints.usbx = np.ones(2)*0.1
        ocp.constraints.idxsbx = np.array([3,9])


        # #Respect track width
        ocp.model.con_h_expr = lh_constraints
        ocp.model.con_h_expr_e = lh_constraints


        # ocp.constraints.lh = np.array([-0.6, -60, -1e3])
        # ocp.constraints.uh = np.array([0.6, 60, 1e3])

        # ocp.constraints.lsh = np.array([-0.1, -10., -10.])
        # ocp.constraints.ush = np.array([0.1, 10., 10.])
        # ocp.constraints.idxsh = np.array(range(3))


        # ocp.cost.zl = 100 * np.ones((3))
        # ocp.cost.zu = 100 * np.ones((3))
        # ocp.cost.Zl = 0.1 * np.ones((3))
        # ocp.cost.Zu = 0.1 * np.ones((3))


        ocp.constraints.lh = np.array([-10,-2])
        ocp.constraints.uh = np.array([0.15**2, 2])

        ocp.constraints.lh_e = np.array([-10, -2])
        ocp.constraints.uh_e = np.array([0.15**2, 2])

        ocp.constraints.lsh = np.array([-0.05, -0.1])
        ocp.constraints.ush = np.array([0.05, 0.1])
        ocp.constraints.idxsh = np.array(range(2))

        ocp.constraints.lsh_e = np.array([-0.05, -0.1])
        ocp.constraints.ush_e = np.array([0.05, 0.1])
        ocp.constraints.idxsh_e = np.array(range(2))


        ocp.cost.zl = 20 * np.ones((4))
        ocp.cost.zu = 20 * np.ones((4))
        ocp.cost.Zl = 0.2 * np.ones((4))
        ocp.cost.Zu = 0.2 * np.ones((4))

        ocp.cost.zl_e = 20 * np.ones((2))
        ocp.cost.zu_e = 20 * np.ones((2))
        ocp.cost.Zl_e = 0.2 * np.ones((2))
        ocp.cost.Zu_e = 0.2 * np.ones((2))

        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"

        ocp.model.cost_expr_ext_cost = cost_expr # Control rate weighing
        ocp.model.cost_expr_ext_cost_e = Je # Contouring cost

        ocp.constraints.x0 = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])

        # set QP solver and integration
        ocp.solver_options.tf = Tf
        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'

        ocp.solver_options.N_horizon = N # N = Tf * 12 because we want 12 Hz

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.hessian_approx = "EXACT"
        ocp.solver_options.regularize_method = "CONVEXIFY"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 3
        ocp.solver_options.nlp_solver_max_iter = 200
        # ocp.solver_options.qp_warm_start  # Enable warm-starting
        ocp.solver_options.tol = 1e-4
        # ocp.solver_options.nlp_solver_tol_comp = 1e-1
        
        # create solver
        acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
        return acados_solver, model, cost_func

if __name__ == "__main__":

    dt = 1/20.

    acados = AcadosMPC(dt = dt, track="ens_race3.txt")

    x_ref_s = acados.x_ref_s
    y_ref_s = acados.y_ref_s

    solver, model, cost_func = acados.create_ocp_solver()

    # when implementing it to the real car you should find closest theta to init thetaA
    state = np.array([acados.tt02.start_pos[0], acados.tt02.start_pos[1], acados.tt02.start_pos[2]+0.1, 0.1, 0., 0., 0.0, 0., 0., 0.2])
    # state = np.array([-6.51790624e-01,  1.66086113e-01 , 6.21816740e+00 , 5.09999999e+00, 4.94699599e-02 , 3.84980233e-01  ,3.29098503e+01,  1.93999843e-02,1.41628476e-02 , 5.57430909e+00])
    
    u0 = np.array([0.,0.,0.])

    state, _, _ = acados.tt02.integration(state,u0)


    [_,x_spline, y_spline, _, _] = getTrack(acados.track)

    i = 0
    j = 0

    x_l = []
    y_l = []    

    omega_plot = []

    thetaA_plot = []

    Dplot = []
    derDplot = []

    delta_plot = []
    derdelta_plot = []

    vx_plot = []
    vtheta_plot = []


    a_lat_plot = []

    e_c_plot = []
    e_l_plot = []

    cost_plot = []

    import time

    start = time.time()

    try:
        while j != 1:
            i+=1
            solver.set(0, "lbx", state)
            solver.set(0, "ubx", state)

            status = solver.solve()

            solver.print_statistics()       
            # print(f"Cost {solver.get_cost()}")

            x_solution = np.array([solver.get(i, "x") for i in range(solver.acados_ocp.solver_options.N_horizon)])
            u_solution = np.array([solver.get(i, "u") for i in range(solver.acados_ocp.solver_options.N_horizon)])

            if status != 4:
                u0 = solver.get(0, "u")
            else:
                u0 = np.array([0.,0.0,0.])

            # u0 = solver.get(0, "u")
            x0 = solver.get(0, "x")

            # print(u0)
            # print(x0)
            
            state, e_c2, e_l2 = acados.tt02.integration(state,u0)

            # print(state.transpose())

            x,y,theta,vx,vy,omega,thetaA,delta,D,vtheta = state

            # print(f"{u0}")
            # print(f"thetaA{thetaA}, vtheta: {vtheta}, final cost: {solver.get_cost()}")
            print(f"Iter {i}, cost {solver.get_cost()}")

            print(f"acados returned status {status} \n")
        
            x_l.append(x)
            y_l.append(y)
            vx_plot.append(vx)
            thetaA_plot.append(thetaA)
            vtheta_plot.append(vtheta)
            omega_plot.append(omega)
            Dplot.append(D)
            delta_plot.append(delta)
            derdelta_plot.append(float(u0[0]))
            a_lat_plot.append(float(0.5 * vx * vx * delta + 2.3*D * math.sin(3.9 * delta) / 2.334))
            derDplot.append(float(u0[1]))
            e_c_plot.append(float(e_c2))
            e_l_plot.append(float(e_l2))   
            cost_plot.append(solver.get_cost()) 

            if j>20 and i<j:
                continue

            if j == 6 and state[6] < acados.tt02.track_length:
                continue
            elif j == 6:
                break
        
            if j == 5 and (status ==0 or status == 2) and i%25:
                continue

            if j == 3 and status == 0:
                continue

            if j == 4 and status == 2:
                continue

            # print(f"Number of SQP iterations: {solver.get_stats('sqp_iter')}")
            # print(f"Final cost value: {solver.get_cost()}")

            # # Check QP solver status
            # print(f"QP solver residuals: {solver.get_stats('qp_stat')}")  
            # print(f"Residuals: {solver.get_stats('residuals')}")  
            # print(f"QP solver iterations: {solver.get_stats('qp_iter')}")
            # # print(f"QP solver statistics: {solver.get_stats('statistics')}")


            # print(f"   Control: {u0}")
            # # total_cost = 0
            # for index in range(40):
            #     print("Total cost : ", cost_func(solver.get(index, "x"), solver.get(index, "u")))
            # # print("Total cost : ", cost_func(solver.get(20, "x"), solver.get(20, "u")))


            index = len(x_solution)-1
        
            # print(f" acados returned status {status}")

            x_pred = np.array(x_solution[:])[1:,0]
            y_pred = np.array(x_solution[:])[1:,1]


            import matplotlib.pyplot as plt        

            try:
                j = int(input("1 to stop, 2 to plot, 3 to skip to next non 0 status, 4 to skip to next non 2 status\n\n"))

                if j == 2:
                    fig, axes = plt.subplots(2, 3, figsize=(9, 6))

                    # 1) XY-plane
                    axes[0,0].plot(x_spline, y_spline, 'b-o', label="Spline Track")
                    axes[0,0].plot(x_l, y_l, 'r-x', label="Car pos")
                    axes[0,0].plot(float(x_ref_s(thetaA)), float(y_ref_s(thetaA)), 'ko', label="ThetaA")
                    axes[0,0].plot(x_pred, y_pred, 'g-x', label="MPC x(Tf), y(Tf)")
                    axes[0,0].plot(float(x_ref_s(thetaA)), float(y_ref_s(thetaA)), 'ko', label="ThetaA")
                    axes[0,0].set_aspect('equal', adjustable='datalim')
                    axes[0,0].set_xlabel("X [m]")
                    axes[0,0].set_ylabel("Y [m]")
                    axes[0,0].set_title("Track in XY-plane")
                    axes[0,0].legend()


                    axes[0,1].plot(a_lat_plot, label="a_lat")
                    # axes[0,1].set_aspect('equal', adjustable='datalim')
                    axes[0,1].set_title("alat")
                    axes[0,1].legend()

                    # axes[0,1].plot(vtheta_plot, label="Vtheta")
                    # axes[0,1].plot(vx_plot, label="vx")
                    # # axes[0,1].set_aspect('equal', adjustable='datalim')
                    # axes[0,1].set_title("Vtheta and Vxplot")
                    # axes[0,1].legend()

                    axes[0,2].plot(e_c_plot, label="e_c")
                    axes[0,2].plot(e_l_plot, label="e_l")
                    # axes[0,2].set_aspect('equal', adjustable='datalim')
                    axes[0,2].set_title("E_c and E_l")
                    axes[0,2].legend()

                    axes[1,0].plot(Dplot, label="D")
                    axes[1,0].plot(derDplot, label="derD")
                    # axes[0,2].set_aspect('equal', adjustable='datalim')
                    axes[1,0].set_title("D and derD")
                    axes[1,0].legend()

                    axes[1,1].plot(delta_plot, label="delta")
                    axes[1,1].plot(derdelta_plot, label="derDelta")
                    # axes[0,2].set_aspect('equal', adjustable='datalim')
                    axes[1,1].set_title("Delta and derDelta")
                    axes[1,1].legend()

                    axes[1,2].plot(vtheta_plot, label="Vtheta")
                    axes[1,2].plot(vx_plot, label="vx")
                    # axes[0,1].set_aspect('equal', adjustable='datalim')
                    axes[1,2].set_title("Vtheta and Vxplot")
                    axes[1,2].legend()


                    plt.tight_layout()
                    plt.show()
            except KeyboardInterrupt:
                print("\n Ciao <3")
                quit()
            except EOFError:
                print("\n Ciao <3")
                quit()
            except:
                pass
    except:
        pass
    print(f"Total track time: {i*dt}s")
    print(f"Total time elapsed: {time.time() - start}")
    print(f"Average time per iter: {(time.time() - start) / i}")


    fig, axes = plt.subplots(2, 3, figsize=(9, 6))

    # 1) XY-plane
    axes[0,0].plot(x_spline, y_spline, 'b-o', label="Spline Track")
    axes[0,0].plot(x_l, y_l, 'r-x', label="Car pos")
    axes[0,0].plot(x_pred, y_pred, 'g-x', label="MPC x(Tf), y(Tf)")
    axes[0,0].set_aspect('equal', adjustable='datalim')
    axes[0,0].set_xlabel("X [m]")
    axes[0,0].set_ylabel("Y [m]")
    axes[0,0].set_title("Track in XY-plane")
    axes[0,0].legend()


    axes[0,1].plot(vtheta_plot, label="Vtheta")
    axes[0,1].plot(vx_plot, label="vx")
    # axes[0,1].set_aspect('equal', adjustable='datalim')
    axes[0,1].set_title("Vtheta and Vxplot")
    axes[0,1].legend()

    axes[0,2].plot(e_c_plot, label="e_c")
    axes[0,2].plot(e_l_plot, label="e_l")
    # axes[0,2].set_aspect('equal', adjustable='datalim')
    axes[0,2].set_title("E_c and E_l")
    axes[0,2].legend()

    axes[1,0].plot(Dplot, label="D")
    axes[1,0].plot(derDplot, label="derD")
    # axes[0,2].set_aspect('equal', adjustable='datalim')
    axes[1,0].set_title("D and derD")
    axes[1,0].legend()

    axes[1,1].plot(delta_plot, label="delta")
    axes[1,1].plot(derdelta_plot, label="derDelta")
    # axes[0,2].set_aspect('equal', adjustable='datalim')
    axes[1,1].set_title("Delta and derDelta")
    axes[1,1].legend()

    axes[1,2].plot(a_lat_plot, label="alat")
    # axes[0,1].set_aspect('equal', adjustable='datalim')
    axes[1,2].set_title("alat")
    axes[1,2].legend()
    plt.tight_layout()
    plt.show()  

    if j == 6:
        plotAgainstTrack(acados.track, x_l, y_l, x_spline, y_spline)
        # plotAgainstTrack("ens55", x_spline, y_spline)
    
   
