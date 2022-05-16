from curses.ascii import SO
from time import time

from matplotlib.image import imread

import casadi as cs
import casadi.tools as cst
import matplotlib.pyplot as plt
import numpy as np
from icecream import ic
from liecasadi import SE3, SO3, SO3Tangent

from planner import Contact, CustomCallback, Robot, Visualizer
from planner.integrator import generic_integrator, robot_integrator


class Problem:
    def __init__(self, model: Robot, steps: int, time_horizon: float = None) -> None:
        self.model = model
        self.N = steps
        self.time_horizon = time_horizon
        self.joints_lb, self.joints_ub = self.model.get_joints_limits()

    def create(
        self,
        initial_condition=None,
        path_costs=None,
        path_constraints=None,
        terminal_costs=None,
        terminal_constraints=None,
        horizon=None,
    ):

        opti = cs.Opti()

        # horizon
        if self.time_horizon is None:
            Time, dt = self.set_optimized_horizon(opti)
        else:
            dt = self.time_horizon / self.N

        # centroidal momementum quantities
        R, dR, ddR, L, dL, F, dF, C = self.get_cm_quantities(opti)
        # dC = opti.variable(self.model.contact_forces_dimension(), self.N + 1)
        # opti.set_initial(dC, 0)

        init_CoM = np.array([0, 0, 0.57])
        final_CoM = np.array([0.0, 0, 0.80])

        # initial CoM constr
        opti.subject_to(R[:, 0] == init_CoM)
        # opti.subject_to(dR[:, 0] == 0)
        opti.subject_to(opti.bounded(-1e-3, dR[:, 0], 1e-3))
        # opti.subject_to(L[:, 0] == 0)
        opti.subject_to(opti.bounded(-1e-3, L[:, 0], 1e-3))
        # opti.subject_to(ddR[:, 0] == 0)
        opti.subject_to(opti.bounded(-1e-3, ddR[:, 0], 1e-3))
        # opti.subject_to(dL[:, 0] == 0)
        opti.subject_to(opti.bounded(-1e-3, dL[:, 0], 1e-3))

        P, Q, S, dS, dP, Om = self.get_kinematics_quantities(initial_condition, opti)

        opti.subject_to(opti.bounded(-1e-3, Q[:, 0] - initial_condition[3:7], 1e-3))
        opti.subject_to(opti.bounded(-1e-3, Q[:, -1] - [0, 0, 0, 1], 1e-3))
        # opti.subject_to(Q[:, 0] == initial_condition[3:7])

        T = 100 * opti.variable(4, self.N + 1)
        [opti.set_initial(T[:, k], 30 * np.ones(4)) for k in range(self.N + 1)]
        dT = opti.variable(4, self.N + 1)
        [opti.set_initial(dT[:, k], np.zeros(4)) for k in range(self.N + 1)]
        ddT = opti.variable(4, self.N + 1)
        [opti.set_initial(ddT[:, k], np.zeros(4)) for k in range(self.N + 1)]
        U = opti.variable(4, self.N + 1)
        [opti.set_initial(U[:, k], np.zeros(4)) for k in range(self.N + 1)]
        # C_jet = opti.variable(4 * 3, self.N + 1)

        # initial thrust constraints
        opti.subject_to(opti.bounded(-1e-3, T[:, 0], 1e-3))
        opti.subject_to(opti.bounded(-1e-3, dT[:, 0], 1e-3))
        # self.add_cost(cs.sumsqr((SO3(Q[:, 0]) - SO3.Identity()).vec))

        # final COM constraint
        opti.subject_to(R[:, -1] == final_CoM)
        # opti.subject_to(R[2, self.N / 2] > 0.80)

        # opti.subject_to(opti.bounded(-1e-1, dR[:, self.N / 2], 1e-1))
        # opti.subject_to(opti.bounded(-1e-1, L[:, self.N / 2], 1e-1))
        # opti.subject_to(opti.bounded(-1e-1, ddR[:, self.N / 2], 1e-1))
        # opti.subject_to(opti.bounded(-1e-1, dL[:, self.N / 2], 1e-1))
        # opti.subject_to(R[2, -1] >= 0.8)
        # opti.subject_to(R[:, -1] == final_CoM)
        # opti.subject_to(opti.bounded(-1e-2, SO3(Q[:, 0]).as_matrix()[1, 1] - 1, 1e-2))
        # opti.subject_to(opti.bounded(-1e-3, dR[:, -1], 1e-3))
        # opti.subject_to(opti.bounded(-1e-3, L[:, -1], 1e-3))
        # opti.subject_to(U[:, 0] == 0)
        # opti.subject_to(dF[:, 0] == 0)

        # opti.subject_to(opti.bounded(-1e-3, ddR[:, -1], 1e-3))
        # opti.subject_to(opti.bounded(-1e-3, dL[:, -1], 1e-3))

        for k in range(self.N + 1):
            pos = P[:, k]
            quat = Q[:, k]
            s = S[:, k]
            w_H_b = SE3(pos, quat).transform()

            m = self.model.kinDyn.get_total_mass()
            ddot_rxm = (m * self.model.kinDyn.g)[:3]
            dot_L = (m * self.model.kinDyn.g)[3:]
            r = R[:, k]
            opti.set_initial(
                R[:, k], init_CoM * (1 - k / self.N) + final_CoM * k / self.N
            )
            # box constraint on the point-com distance
            opti.subject_to(r[2] > 0)
            opti.subject_to(opti.bounded(-1.5, r[0], 1.5))
            opti.subject_to(opti.bounded(-1.5, r[1], 1.5))
            for frame, contact in self.model.contacts_dict.items():
                f = F[contact.idx, k]
                c = C[contact.idx, k]

                opti.set_initial(
                    C[contact.idx, k][:2],
                    init_CoM[:2] * (1 - k / self.N) + final_CoM[:2] * k / self.N,
                )

                opti.subject_to(
                    opti.bounded([-0.3, -0.3, -0.6], c - r, [0.3, 0.3, 0.6])
                )
                ddot_rxm, dot_L = self.add_force(opti, ddot_rxm, dot_L, r, f, c)

                opti.subject_to(c[2] >= 0)
                opti.subject_to(f[2] >= 0)
                self.set_friction_cone(opti, f)
                self.set_complementarity(opti, f, c)
                if k < self.N:
                    c_next = C[contact.idx, k + 1]
                    self.set_planar_complementarity(opti, f, c, c_next)

            for frame, jet in self.model.jets_dict.items():
                R_j = self.model.kinDyn.forward_kinematics_fun(frame)(w_H_b, s)[:3, 2]
                # c = opti.variable(3)
                c = self.model.kinDyn.forward_kinematics_fun(frame)(w_H_b, s)[:3, 3]
                # opti.subject_to(
                #     c
                #     == self.model.kinDyn.forward_kinematics_fun(frame)(w_H_b, s)[:3, 3]
                # )
                opti.subject_to(
                    ddT[jet.idx, k]
                    == jet.get_acceleration()(
                        T[jet.idx, k], dT[jet.idx, k], U[jet.idx, k]
                    )
                )
                # f = opti.variable(3)
                # opti.set_initial(f, [0, 0, 0])
                # opti.subject_to(f == R_j @ -T[jet.idx, k])
                f = R_j @ -T[jet.idx, k]
                opti.subject_to(opti.bounded(0, T[jet.idx, k], 220))
                # opti.subject_to(opti.bounded(-50, dT[jet.idx, k], 50))
                opti.subject_to(opti.bounded(0, U[jet.idx, k], 100))
                opti.subject_to(opti.bounded(-0.5, c - r, 0.5))
                ddot_rxm, dot_L = self.add_force(opti, ddot_rxm, dot_L, r, f, c)

            opti.subject_to(ddR[:, k] == ddot_rxm / m)
            opti.subject_to(dL[:, k] == dot_L)

        for k in range(self.N + 1):
            pos = P[:, k]
            quat = Q[:, k]
            s = S[:, k]
            w_H_b = SE3(pos, quat).transform()
            v = cs.vertcat(dP[:, k], Om[:, k], dS[:, k])
            opti.subject_to(R[:, k] == self.model.kinDyn.CoM_position_fun()(w_H_b, s))

            cmm = (self.model.kinDyn.centroidal_momentum_matrix_fun()(w_H_b, s))[3:, :]
            opti.subject_to(L[:, k] == cmm @ v)

            # opti.subject_to(cs.sumsqr(quat) == 1)
            opti.set_initial(Q[:, k], [0, 0, 0, 1])
            for frame, contact in self.model.contacts_dict.items():
                # c = C[contact.idx, k]
                opti.subject_to(
                    C[contact.idx, k]
                    == self.model.kinDyn.forward_kinematics_fun(frame)(w_H_b, s)[:3, 3]
                )

        # integration
        for k in range(self.N):
            # com integration
            r_next = R[:, k] + (dR[:, k + 1] + dR[:, k]) * dt / 2
            dr_next = dR[:, k] + ddR[:, k + 1] * dt
            opti.subject_to(R[:, k + 1] == r_next)
            opti.subject_to(dR[:, k + 1] == dr_next)
            # angular momentum integration
            l_next = L[:, k] + dL[:, k + 1] * dt
            opti.subject_to(L[:, k + 1] == l_next)
            # force integration
            f_next = F[:, k] + dF[:, k + 1] * dt
            opti.subject_to(F[:, k + 1] == f_next)
            # joints integration
            s_next = S[:, k] + dS[:, k + 1] * dt
            opti.subject_to(S[:, k + 1] == s_next)
            # base pose integration
            p_next = P[:, k] + dP[:, k + 1] * dt
            opti.subject_to(P[:, k + 1] == p_next)
            # base orientation integration
            # dot_q = self.get_dot_q(Q[:, k + 1], Om[:, k + 1])
            # q_next = Q[:, k] + dot_q * dt
            q_next = (SO3Tangent(Om[:, k + 1] * dt) + SO3(Q[:, k])).as_quat().coeffs()
            opti.subject_to(Q[:, k + 1] == q_next)
            t_next = T[:, k] + dT[:, k + 1] * dt
            opti.subject_to(T[:, k + 1] == t_next)
            dt_next = dT[:, k] + (ddT[:, k] + ddT[:, k + 1]) * dt / 2
            opti.subject_to(dT[:, k + 1] == dt_next)

        self.add_cost(cs.sumsqr(S[:, 0] - initial_condition[7:]) * 1e6)
        # self.add_cost(cs.sumsqr(S[:, -1] - initial_condition[7:]) * 1e3)

        self.add_cost(cs.sumsqr(dR[:, -1] * 1e3))
        self.add_cost(cs.sumsqr(L[:, -1] * 1e3))
        self.add_cost(cs.sumsqr(ddR[:, -1] * 1e3))
        self.add_cost(cs.sumsqr(dL[:, -1] * 1e3))
        # self.add_cost(cs.sumsqr(F[:, 0]) * 1e1 * dt)
        # self.add_cost(cs.sumsqr(F[:, -1]) * 1e-1 * dt)
        self.add_cost(cs.sumsqr(T[:, -1]) * 1e2)
        # opti.subject_to(
        #     opti.bounded(-1e-1, (SO3(Q[:, 0]) - SO3(initial_condition[3:7])).vec, 1e-1)
        # )
        self.add_cost(1e5 * cs.sumsqr((SO3(Q[:, 0]) - SO3(initial_condition[3:7])).vec))

        for k in range(self.N + 1):
            self.add_cost(cs.sumsqr(ddR[:, k]) * 1e5 * dt)
            self.add_cost(cs.sumsqr(dR[:, k]) * 1e7 * dt)
            self.add_cost(cs.sumsqr(dL[:, k]) * 1e4 * dt)
            self.add_cost(cs.sumsqr(L[:, k]) * 1e4 * dt)
            self.add_cost(cs.sumsqr(dS[:12, k]) * 5e5 * dt)
            self.add_cost(cs.sumsqr(dS[-12:, k]) * 1e4 * dt)
            self.add_cost(cs.sumsqr(S[:, k] - initial_condition[7:]) * 1e6 * dt)
            self.add_cost(cs.sumsqr(F[:, k]) * 5e1 * dt)
            self.add_cost(cs.sumsqr(dF[:, k]) * 1e0 * dt)
            self.add_cost(cs.sumsqr(U[:, k]) * 1e-1 * dt)
            # self.add_cost(cs.sumsqr(SO3(Q[:, k]).as_matrix()[1, 1] - 1) * dt)

        for k in range(self.N + 1):
            self.set_joint_constraint(opti, S[:, k], dS[:, k])

        #     if path_costs is not None:
        #         for cost in path_costs:
        #             self.add_cost(cost(X[:, k], V[:, k], F[:, k], T[:, k]) * dt)

        #     if path_constraints is not None:
        #         for constr in path_constraints:
        #             lb, g, ub = constr(X[:, k], V[:, k], F[:, k], T[:, k])
        #             opti.subject_to(opti.bounded(lb, g, ub))

        # if terminal_costs is not None:
        #     for cost in terminal_costs:
        #         self.add_cost(cost(X[:, -1], V[:, -1], F[:, -1], T[:, -1]) * dt)

        # if terminal_constraints is not None:
        #     for constr in terminal_constraints:
        #         lb, g, ub = constr(X[:, -1], V[:, -1], F[:, -1], T[:, -1])
        #         opti.subject_to(opti.bounded(lb, g, ub))

        opti.minimize(self.cost)

        self.set_solver(opti)

        try:
            sol = opti.solve()

        except:
            print("Not solved :(.")
            # opti.debug.show_infeasibilities()
        # print("CoM Position")
        # print(sol.value(R))
        print("Contact forces")
        print(sol.value(F))
        print("Time")
        print(sol.value(Time))
        print("Thrust")
        print(sol.value(T))
        print("Throttle")
        print(sol.value(U))

        viz = Visualizer(robot=self.model)
        running = viz.viz.run()

        import datetime
        import glob
        import os
        import tempfile
        from pathlib import Path

        import imageio

        temp_dir = Path(tempfile.TemporaryDirectory().name)
        os.mkdir(temp_dir)

        now = datetime.datetime.now()
        month = "{:02d}".format(now.month)
        day = "{:02d}".format(now.day)
        hour = "{:02d}".format(now.hour)
        minute = "{:02d}".format(now.minute)

        picture_dir = f"pictures/{month}-{day}-{hour}-{minute}"
        os.mkdir(picture_dir)

        input("Ashpett")
        for k in range(self.N + 1):
            pos = sol.value(P)[:, k]
            quat = sol.value(Q)[:, k]
            H = np.array(SE3(pos, quat).transform())
            s = sol.value(S)[:, k]
            t = sol.value(T)[:, k]
            # t = np.zeros(4)
            f = sol.value(F)[:, k]
            viz.viz.run()
            viz.update(H, s, t, f)
            viz.viz.drawToFile(f"{picture_dir}/{str(k).zfill(3)}.png")
            input()

        gif_name = f"pictures/{month}-{day}-{hour}-{minute}.gif"

        import pickle

        dataset = {
            "com-pos": sol.value(R),
            "com-vel": sol.value(dR),
            "com-acc": sol.value(ddR),
            "ang-mom": sol.value(L),
            "ang-mom-der": sol.value(dL),
            "position": sol.value(P),
            "orientation": sol.value(Q),
            "joint_pos": sol.value(S),
            "lin-vel": sol.value(dP),
            "ang-vel": sol.value(Om),
            "joint_vel": sol.value(dS),
            "contact-forces": sol.value(F),
            "contact-forces-der": sol.value(dF),
            "thrust": sol.value(T),
            "thrust-der": sol.value(dT),
            "thrust-acc": sol.value(ddT),
            "throttle": sol.value(U),
            "Time": sol.value(Time),
            "knots": self.N,
        }

        with open(f"pictures/{month}-{day}-{hour}-{minute}.pickle", "wb") as output:
            pickle.dump(dataset, output)

        # file_list = glob.glob(f"{temp_dir}/*.png")
        # file_list = sorted(file_list, key=lambda x: x[-6:-4])
        # images = []
        # for file in file_list:
        #     images += imageio.imread(file)
        # imageio.mimsave(gif_name, images, loop=1, duration=sol.value(Time) / self.N)

        with imageio.get_writer(
            gif_name, loop=1, duration=sol.value(Time) / self.N, mode="I"
        ) as writer:
            file_list = glob.glob(f"{picture_dir}/*.png")
            for filename in sorted(file_list, key=lambda x: x[-7:-4]):
                image = imageio.imread(filename)
                writer.append_data(image)

        return opti

    def get_kinematics_quantities(self, initial_condition, opti):
        P = opti.variable(3, self.N + 1)
        [opti.set_initial(P[:, k], initial_condition[:3]) for k in range(self.N + 1)]
        Q = opti.variable(4, self.N + 1)
        opti.subject_to(opti.bounded([-1, -1, -1, -1], Q, [1, 1, 1, 1]))
        S = 2 * opti.variable(self.model.ndofs, self.N + 1)
        [opti.set_initial(S[:, k], initial_condition[7:]) for k in range(self.N + 1)]
        dS = 7 * opti.variable(self.model.ndofs, self.N + 1)
        [
            opti.set_initial(dS[:, k], np.zeros(self.model.ndofs))
            for k in range(self.N + 1)
        ]
        dP = opti.variable(3, self.N + 1)
        [opti.set_initial(dP[:, k], np.zeros(3)) for k in range(self.N + 1)]
        opti.subject_to(opti.bounded(-2, dP, 2))
        Om = opti.variable(3, self.N + 1)
        [opti.set_initial(Om[:, k], np.zeros(3)) for k in range(self.N + 1)]
        opti.subject_to(opti.bounded(-2, Om, 2))
        return P, Q, S, dS, dP, Om

    def get_dot_q(self, quat, omega):
        A = 0.5 * cs.vertcat(
            cs.horzcat(0, -omega[0], -omega[1], -omega[2]),
            cs.horzcat(omega[0], 0, omega[2], -omega[1]),
            cs.horzcat(omega[1], -omega[2], 0, omega[0]),
            cs.horzcat(omega[2], omega[1], -omega[0], 0),
        )
        A[1:, 1:] = A[1:, 1:].T
        quat = cs.vertcat(quat[3], quat[0], quat[1], quat[2])
        dot_q = A @ quat
        dot_q = cs.vertcat(dot_q[1], dot_q[2], dot_q[3], dot_q[0])
        return dot_q

    def get_cm_quantities(self, opti: cs.Opti):
        R = opti.variable(3, self.N + 1)
        # opti.set_initial(R, np.transpose([np.array([0, 0, 0.57])] * (self.N + 1)))
        dR = opti.variable(3, self.N + 1)
        opti.set_initial(dR, 0)
        ddR = opti.variable(3, self.N + 1)
        opti.set_initial(ddR, 0)
        L = opti.variable(3, self.N + 1)
        opti.set_initial(L, 0)
        dL = opti.variable(3, self.N + 1)
        opti.set_initial(dL, 0)
        F = 50 * opti.variable(self.model.contact_forces_dimension(), self.N + 1)
        opti.set_initial(F, 0)
        dF = 50 * opti.variable(self.model.contact_forces_dimension(), self.N + 1)
        opti.set_initial(dF, 0)
        C = opti.variable(self.model.contact_forces_dimension(), self.N + 1)
        opti.set_initial(C, 0)
        return R, dR, ddR, L, dL, F, dF, C

    def set_optimized_horizon(self, opti):
        print("Optimizing also the horizon length.")
        Time = opti.variable(1)
        min_Time = 0.05 * self.N
        max_Time = 0.1 * self.N
        opti.subject_to(opti.bounded(min_Time, Time, max_Time))
        opti.set_initial(Time, max_Time)
        dt = Time / self.N
        self.add_cost(Time)
        return Time, dt

    def add_force(self, opti, ddot_rxm, dot_L, r, f, c):
        ddot_rxm += f
        dot_L += cs.skew(c - r) @ f
        return ddot_rxm, dot_L

    def set_planar_complementarity(self, opti, f, c, c_next):
        panik = opti.variable(1)
        self.add_cost(panik * 1e5)
        # opti.subject_to(opti.bounded(0, panik, 1e-1))
        opti.subject_to(opti.bounded(-panik, f[2] * (c[0] - c_next[0]), panik))
        opti.subject_to(opti.bounded(-panik, f[2] * (c[1] - c_next[1]), panik))

    def set_complementarity(self, opti, f, c):
        panik = opti.variable(1)
        self.add_cost(panik * 1e5)
        # opti.subject_to(opti.bounded(0, panik, 1e-1))
        opti.subject_to(f[2] * c[2] <= panik)
        # # DCC
        # chi = dc[2] * f[2] + c[2] * df[2]
        # K_bs = 20
        # opti.subject_to(chi <= -K_bs * f[2] * c[2])
        # delta_p = 1 / cs.cosh(500 * c[2])
        # Mf = 100
        # Kf = 250
        # opti.subject_to(
        #     opti.bounded(-Mf, df[2], -Kf * (1 - delta_p) * f[2] + delta_p * Mf)
        # )

    def set_friction_cone(self, opti, f):
        mu = 0.33
        opti.subject_to(opti.bounded(-mu * f[2], f[0], mu * f[2]))
        opti.subject_to(opti.bounded(-mu * f[2], f[1], mu * f[2]))

    def set_solver(self, opti):
        if use_sqpmethod := False:
            options = {
                "print_time": False,
                "expand": True,
                "qpsol": "qrqp",
                "qpsol_options": {"print_iter": False, "print_header": False},
                "print_iteration": False,
                "print_header": False,
                "print_status": False,
            }

            opti.solver("sqpmethod", options)
        if use_ipopt := True:
            optiSettings = {"expand": True}
            solvSettings = {
                "linear_solver": "ma97",
                "tol": 0.001,
                "acceptable_tol": 1,  # 0.01
                "max_iter": 10000,
                "hessian_approximation": "limited-memory",
                "alpha_for_y": "dual-and-full",
                "dual_inf_tol": 10.0,
                # "iteration_callback": CustomCallback("callback", opti),
                # "print_time": False,
                "print_timing_statistics": "yes",
                # "ipopt": {"print_level": 1},
                # "warm_start_init_point": "yes",
            }

            solvSettings = {
                # "print_level": 0,
                "linear_solver": "ma97",
                "ma57_pivtol": 1e-6,
                "nlp_scaling_max_gradient": 100.0,
                "nlp_scaling_min_value": 1e-6,
                "tol": 1e-3,
                "dual_inf_tol": 1000.0,
                "compl_inf_tol": 1e-2,
                "constr_viol_tol": 1e-4,
                "acceptable_tol": 1e0,
                "acceptable_iter": 2,
                "acceptable_compl_inf_tol": 1,
                "alpha_for_y": "dual-and-full",
                "max_iter": 4000,
                "ma97_print_level": -10,
                "warm_start_bound_frac": 1e-2,
                "warm_start_bound_push": 1e-2,
                "warm_start_mult_bound_push": 1e-2,
                "warm_start_slack_bound_frac": 1e-2,
                "warm_start_slack_bound_push": 1e-2,
                "warm_start_init_point": "yes",
                "required_infeasibility_reduction": 0.8,
                "perturb_dec_fact": 0.1,
                "max_hessian_perturbation": 100.0,
                "fast_step_computation": "yes",
                "hessian_approximation": "limited-memory",
            }

            opti.solver("ipopt", optiSettings, solvSettings)

    def print_stats(self, opti=None, X=None, V=None, F=None, T=None, U=None, Time=None):
        ic(self.model.get_com_position(opti.debug.value(X))[:, -1])
        ic(opti.debug.value(T)[:, -1])
        ic(opti.debug.value(U)[:, -1])

    def add_cost(self, cost):
        """Adding cost

        Args:
            cost (cs.MX): single cost
        """
        if not hasattr(self, "cost"):
            self.cost = cost
        else:
            self.cost += cost

    def set_joint_constraint(self, opti, s, ds):
        """Set joint position boundaries and joint velocity boundaries"""
        opti.subject_to(
            opti.bounded(
                np.transpose([self.joints_lb]),
                s,
                np.transpose([self.joints_ub]),
            )
        )
        opti.subject_to(opti.bounded(-7, ds, 7))


if __name__ == "__main__":

    import pathlib

    import toml

    config_path = "src/planner/robot/ironcub_optimized_initial_position.toml"
    config = toml.load(config_path)
    initial_condition = cs.vertcat(
        config["optimized"]["position"],
        config["optimized"]["orientation"],
        config["optimized"]["joint_position"],
    )

    robot = Robot()
    problem = Problem(robot, 70)

    def desired_com_pos_cost(x, v, f, t):
        return cs.sumsqr(robot.get_com_position(x) - [0, 0, 2]) * 1e1

    def increase_thrust_cost(x, v, f, t):
        return -cs.sumsqr(t) * 1e0 * 0

    def decrease_contact_cost(x, v, f, t):
        return cs.sumsqr(f) * 1e0 * 0

    def minimize_variation_cost(x, v, f, t):
        return cs.sumsqr(v) * 2e2

    def postural_cost(x, v, f, t):
        return cs.sumsqr(x - initial_condition) * 1e1

    def thrust_constraint(x, v, f, t):
        return cs.vertcat(0, 0, 0, 0), t, cs.vertcat(160, 160, 220, 220)

    def detach_feet_constraint(x, v, f, t):
        g = []
        lb = []
        ub = []
        # set contact z > zero
        for frame, contact in robot.contacts_dict.items():
            g = cs.vertcat(g, contact.origin(x)[2])
            lb = cs.vertcat(lb, 0.03)
            ub = cs.vertcat(ub, cs.np.inf)
        #  set contact forces to zero
        g = cs.vertcat(g, f)
        lb = cs.vertcat(lb, 0 * cs.DM.ones(f.shape[0]))
        ub = cs.vertcat(ub, 0 * cs.DM.ones(f.shape[0]))
        return lb, g, ub

    path_costs = [
        increase_thrust_cost,
        decrease_contact_cost,
        minimize_variation_cost,
        postural_cost,
    ]

    path_constraints = [thrust_constraint]

    terminal_costs = [
        desired_com_pos_cost,
        increase_thrust_cost,
        decrease_contact_cost,
    ]

    terminal_constraints = [detach_feet_constraint]

    problem.create(
        initial_condition=initial_condition,
        path_costs=path_costs,
        path_constraints=path_constraints,
        terminal_costs=terminal_costs,
        terminal_constraints=terminal_constraints,
    )

    # problem.solve()

    # problem.show_result()
