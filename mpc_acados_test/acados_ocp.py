import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from tt02 import AckermannCar, getTrack


class AcadosMPC:

    def __init__(self, track="su_bigmap_splined.txt", dt=1/20.):
        self.track = track
        self.dt = dt
        self.tt02 = AckermannCar(0.257, track, dt)
        pass

    def create_ocp_solver(self, Tf=2.0):

        N=int(Tf*20.)

        #state = x y theta vx vy omega thetaA delta D vtheta
        #control = delta_dot D_dot vtheta_dot

        model, cost_expr, J, cost_func, Je, lh_constraints = self.tt02.model

        ocp = AcadosOcp()
        ocp.model = model


        ocp.constraints.lbu = np.array([-10.,-10.,-10.]) #DeltaDot, DDot, VthetaDot
        ocp.constraints.ubu = np.array([10.,10.,10.])
        ocp.constraints.idxbu = np.array([0,1,2]) #Indices of the corresponding variables

        ocp.constraints.lbx = np.array([-10,-10,-10, 0.0, -3.0, -8.0, -1.0, -0.4, -1.0, 0.05])
        ocp.constraints.ubx = np.array([10,  10, 10,  5,  3.0, 8.0, 100, 0.4, 1.0, 3.0])
        ocp.constraints.idxbx = np.array(range(10)) #Indices of the corresponding variables

        # ocp.constraints.lsbx = np.ones([10])*0.1
        # ocp.constraints.usbx = np.ones([10])*0.1
        # ocp.constraints.idxsbx = np.array(range(10))


        # #Respect track width
        ocp.model.con_h_expr = lh_constraints

        ocp.constraints.lh = np.array([0., -40, -40])
        ocp.constraints.uh = np.array([1, 40, 40])

        ocp.constraints.lsh = np.ones(3) * -0.1
        ocp.constraints.ush = np.ones(3) * 0.1
        ocp.constraints.idxsh = np.array(range(3))


        ocp.cost.zl = 100 * np.zeros((3,))
        ocp.cost.zu = 100 * np.zeros((3,))
        ocp.cost.Zl = 1 * np.zeros((3,))
        ocp.cost.Zu = 1 * np.zeros((3,))

        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"

        ocp.model.cost_expr_ext_cost = cost_expr # Control rate weighing
        ocp.model.cost_expr_ext_cost_e = Je # Contouring cost

        ocp.constraints.x0 = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])

        # set QP solver and integration
        ocp.solver_options.tf = Tf
        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'

        ocp.solver_options.N_horizon = N # N = Tf * 12 because we want 12 Hz

        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.hessian_approx = "EXACT"
        ocp.solver_options.regularize_method = "CONVEXIFY"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 3
        ocp.solver_options.nlp_solver_max_iter = 100
        ocp.solver_options.qp_warm_start = 1  # Enable warm-starting
        ocp.solver_options.tol = 1e-4
        # ocp.solver_options.nlp_solver_tol_comp = 1e-1
        
        # create solver
        acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
        return acados_solver, model, cost_func

if __name__ == "__main__":
    acados = AcadosMPC()

    solver, model, cost_func = acados.create_ocp_solver()


    state = np.array([-0.2635, -0.1, 0.1, 0.1, 0., 0., 0.24, 0., 0., 0.05])
    u0 = np.array([0.,0.,0.])

    [_,x_spline, y_spline, _, _] = getTrack(acados.track)

    i = 0
    j = 0

    x_l = []
    y_l = []

    vx_plot = []
    vtheta_plot = []

    e_c_plot = []
    e_l_plot = []


    while j != 1:
        i+=1
        solver.set(0, "lbx", state)
        solver.set(0, "ubx", state)

        status = solver.solve()

        x_solution = np.array([solver.get(i, "x") for i in range(solver.acados_ocp.solver_options.N_horizon)])
        u_solution = np.array([solver.get(i, "u") for i in range(solver.acados_ocp.solver_options.N_horizon)])

        if status != 4:
            u0 = solver.get(0, "u")
        else:
            u0 = np.array([0.,0.,0.])

        state, e_c2, e_l2 = acados.tt02.integration(state,u0)


        x,y,theta,vx,vy,omega,thetaA,delta,D,vtheta = state

        print(f"{u0}")
        print(f"vx: {vx}, vtheta: {vtheta}")

        print(f"acados returned status {status}")


        x_l.append(x)
        y_l.append(y)
        vx_plot.append(vx)
        vtheta_plot.append(vtheta)
        e_c_plot.append(float(e_c2))
        e_l_plot.append(float(e_l2))
    
        if j == 5 and (status ==0 or status == 2) and i%200:
            continue

        if j == 3 and status == 0:
            continue

        if j == 4 and status == 2:
            continue

        print(f"Number of SQP iterations: {solver.get_stats('sqp_iter')}")
        print(f"Final cost value: {solver.get_cost()}")

        # Check QP solver status
        print(f"QP solver residuals: {solver.get_stats('qp_stat')}")  
        print(f"Residuals: {solver.get_stats('residuals')}")  
        print(f"QP solver iterations: {solver.get_stats('qp_iter')}")
        # print(f"QP solver statistics: {solver.get_stats('statistics')}")


        print(f"\nStep {i/12.}:")
        print(f"   Control: {u0}")
        total_cost = 0
        for index in range(20):
            x_i = solver.get(index, "x")
            u_i = solver.get(index, "u")
            _,_,_,_, cost = cost_func(x_i, u_i)
            total_cost += cost

        print("Total cost : ", cost)


        index = len(x_solution)-1
       
        print(f" acados returned status {status}")

        x_pred = np.array(x_solution[:])[1:,0]
        y_pred = np.array(x_solution[:])[1:,1]


        import matplotlib.pyplot as plt        

        try:
            j = int(input("1 to stop, 2 to plot, 3 to skip to next non 0 status, 4 to skip to next non 2 status\n\n"))

            if j == 2:
                fig, axes = plt.subplots(1, 3, figsize=(15, 5))

                # 1) XY-plane
                axes[0].plot(x_spline, y_spline, 'b-o', label="Spline Track")
                axes[0].plot(x_l, y_l, 'r-x', label="Car pos")
                axes[0].plot(x_pred, y_pred, 'g-x', label="MPC x(Tf), y(Tf)")
                axes[0].set_aspect('equal', adjustable='datalim')
                axes[0].set_xlabel("X [m]")
                axes[0].set_ylabel("Y [m]")
                axes[0].set_title("Track in XY-plane")
                axes[0].legend()


                axes[1].plot(vtheta_plot, label="Vtheta")
                axes[1].plot(vx_plot, label="vx")
                axes[1].set_aspect('equal', adjustable='datalim')
                axes[1].set_title("Vtheta and Vxplot")
                axes[1].legend()

                axes[2].plot(e_c_plot, label="e_c")
                axes[2].plot(e_l_plot, label="e_l")
                # axes[2].set_aspect('equal', adjustable='datalim')
                axes[2].set_title("E_c and E_l")
                axes[2].legend()

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
