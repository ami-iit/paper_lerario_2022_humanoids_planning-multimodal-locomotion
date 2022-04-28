import casadi as cs
import numpy as np
from liecasadi import SE3, SO3, SO3Tangent
from robot import Robot
from visualization import Visualizer


class Problem:
    def __init__(self, model: Robot, knots: int, time_horizon: float = None) -> None:
        self.model = model
        self.N = knots
        self.time_horizon = time_horizon
        self.joints_lb, self.joints_ub = self.model.get_joints_limits()
        self.zero_initial_thrust = False

    def set_initial_com(self, value: np.ndarray):
        self.initial_com = value

    def set_final_com(self, value: np.ndarray):
        self.final_com = value

    def set_zero_intial_thrust(self):
        self.zero_initial_thrust = True

    def set_middle_com_constraint(self, value: float):
        self.com_mid_constraint = value

    def create(self, initial_condition=None):

        opti = cs.Opti()

        if self.time_horizon is None:
            Time, dt = self.set_optimized_horizon(opti)
        else:
            dt = self.time_horizon / self.N

        # centroidal momementum quantities
        R, dR, ddR, L, dL, F, dF, C = self.get_cm_quantities(opti)

        # initial CoM constr
        opti.subject_to(R[:, 0] == self.initial_com)
        # initial equilibrium
        opti.subject_to(opti.bounded(-1e-3, dR[:, 0], 1e-3))
        opti.subject_to(opti.bounded(-1e-3, L[:, 0], 1e-3))
        opti.subject_to(opti.bounded(-1e-3, ddR[:, 0], 1e-3))
        opti.subject_to(opti.bounded(-1e-3, dL[:, 0], 1e-3))

        P, Q, S, dS, dP, Om = self.get_kinematics_quantities(initial_condition, opti)

        opti.subject_to(opti.bounded(-1e-3, Q[:, 0] - initial_condition[3:7], 1e-3))
        opti.subject_to(opti.bounded(-1e-3, Q[:, -1] - initial_condition[3:7], 1e-3))
        [opti.subject_to(C[i * 3 - 1, 0] == 0) for i in range(8)]

        T = 100 * opti.variable(4, self.N + 1)
        [opti.set_initial(T[:, k], 30 * np.ones(4)) for k in range(self.N + 1)]
        dT = opti.variable(4, self.N + 1)
        [opti.set_initial(dT[:, k], np.zeros(4)) for k in range(self.N + 1)]
        ddT = opti.variable(4, self.N + 1)
        [opti.set_initial(ddT[:, k], np.zeros(4)) for k in range(self.N + 1)]
        U = opti.variable(4, self.N + 1)
        [opti.set_initial(U[:, k], np.zeros(4)) for k in range(self.N + 1)]

        # initial thrust constraints
        if self.zero_initial_thrust:
            opti.subject_to(opti.bounded(-1e-3, T[:, 0], 1e-3))
            opti.subject_to(opti.bounded(-1e-3, dT[:, 0], 1e-3))

        # final COM constraint
        opti.subject_to(R[:, -1] == self.final_com)

        if hasattr(self, "com_mid_constraint"):
            opti.subject_to(R[2, self.N / 2] > self.com_mid_constraint)

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
                R[:, k],
                self.initial_com * (1 - k / self.N) + self.final_com * k / self.N,
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
                    self.initial_com[:2] * (1 - k / self.N)
                    + self.final_com[:2] * k / self.N,
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
                c = self.model.kinDyn.forward_kinematics_fun(frame)(w_H_b, s)[:3, 3]
                opti.subject_to(
                    ddT[jet.idx, k]
                    == jet.get_acceleration()(
                        T[jet.idx, k], dT[jet.idx, k], U[jet.idx, k]
                    )
                )

                f = R_j @ -T[jet.idx, k]
                opti.subject_to(opti.bounded(0, T[jet.idx, k], 220))
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
            opti.set_initial(Q[:, k], [0, 0, 0, 1])
            for frame, contact in self.model.contacts_dict.items():
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
            q_next = (SO3Tangent(Om[:, k + 1] * dt) + SO3(Q[:, k])).as_quat().coeffs()
            opti.subject_to(Q[:, k + 1] == q_next)
            t_next = T[:, k] + dT[:, k + 1] * dt
            opti.subject_to(T[:, k + 1] == t_next)
            dt_next = dT[:, k] + (ddT[:, k] + ddT[:, k + 1]) * dt / 2
            opti.subject_to(dT[:, k + 1] == dt_next)

        self.add_cost(
            cs.sumsqr(S[:, 0] - initial_condition[7:]) * self.intitial_joints_w
        )
        # self.add_cost(cs.sumsqr(S[:, -1] - initial_condition[7:]) * 1e3)

        self.add_cost(cs.sumsqr(dR[:, -1] * 1e3))
        self.add_cost(cs.sumsqr(L[:, -1] * 1e3))
        self.add_cost(cs.sumsqr(ddR[:, -1] * 1e3))
        self.add_cost(cs.sumsqr(dL[:, -1] * 1e3))
        self.add_cost(cs.sumsqr(F[:, 0]) * self.initial_force_w)
        # self.add_cost(cs.sumsqr(F[:, -1]) * 1e-1 * dt)
        # self.add_cost(cs.sumsqr(T[:, -1]) * 1e2)
        self.add_cost(
            cs.sumsqr((SO3(Q[:, 0]) - SO3(initial_condition[3:7])).vec)
            * self.initial_orientation_w
        )

        for k in range(self.N + 1):
            self.add_cost(cs.sumsqr(ddR[:, k]) * self.running_com_acc_w * dt)
            self.add_cost(cs.sumsqr(dR[:, k]) * self.running_com_vel_w * dt)
            self.add_cost(cs.sumsqr(dL[:, k]) * self.running_mom_der_w * dt)
            self.add_cost(cs.sumsqr(L[:, k]) * self.running_mom_w * dt)
            self.add_cost(cs.sumsqr(dS[:12, k]) * self.running_joints_vel_upper_w * dt)
            self.add_cost(cs.sumsqr(dS[-12:, k]) * self.running_joints_vel_lower_w * dt)
            self.add_cost(
                cs.sumsqr(S[:, k] - initial_condition[7:])
                * self.running_postural_w
                * dt
            )
            self.add_cost(cs.sumsqr(F[:, k]) * self.running_force_w * dt)
            self.add_cost(cs.sumsqr(dF[:, k]) * self.running_force_der_w * dt)
            self.add_cost(cs.sumsqr(U[:, k]) * self.throtte_w * dt)
            self.add_cost(
                cs.sumsqr(Q[:, k] - [0, 0, 0, 1]) * self.running_orientation_w * dt
            )
        for k in range(self.N + 1):
            self.set_joint_constraint(opti, S[:, k], dS[:, k])

        opti.minimize(self.cost)

        self.set_solver(opti)

        sol = opti.solve()

        viz = Visualizer(robot=self.model)

        import datetime
        import glob
        import os
        import imageio

        now = datetime.datetime.now()
        month = "{:02d}".format(now.month)
        day = "{:02d}".format(now.day)
        hour = "{:02d}".format(now.hour)
        minute = "{:02d}".format(now.minute)

        picture_dir = f"runs/{month}-{day}-{hour}-{minute}"
        os.makedirs(f"{picture_dir}/pictures")

        input("Press to go on with frames.")
        for k in range(self.N + 1):
            pos = sol.value(P)[:, k]
            quat = sol.value(Q)[:, k]
            H = np.array(SE3(pos, quat).transform())
            s = sol.value(S)[:, k]
            t = sol.value(T)[:, k]
            f = sol.value(F)[:, k]
            viz.viz.run()
            viz.update(H, s, t, f)
            viz.viz.drawToFile(f"{picture_dir}/pictures/{str(k).zfill(3)}.png")
            input()

        gif_name = f"{picture_dir}/gif.gif"

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

        with open(f"{picture_dir}/data.pickle", "wb") as output:
            pickle.dump(dataset, output)

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

    def get_cm_quantities(self, opti: cs.Opti):
        R = opti.variable(3, self.N + 1)
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
        self.add_cost(panik * 1e7)
        # opti.subject_to(opti.bunded(0, panik, 1e-1))
        opti.subject_to(opti.bounded(-panik, f[2] * (c[0] - c_next[0]), panik))
        opti.subject_to(opti.bounded(-panik, f[2] * (c[1] - c_next[1]), panik))

    def set_complementarity(self, opti, f, c):
        panik = opti.variable(1)
        self.add_cost(panik * 1e7)
        # opti.subject_to(opti.bounded(0, panik, 1e-1))
        opti.subject_to(f[2] * c[2] <= panik)

    def set_friction_cone(self, opti, f):
        mu = 0.33
        opti.subject_to(opti.bounded(-mu * f[2], f[0], mu * f[2]))
        opti.subject_to(opti.bounded(-mu * f[2], f[1], mu * f[2]))

    def set_solver(self, opti):
        optiSettings = {"expand": True}

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
    import toml

    config_path = "config/ironcub_optimized_initial_position.toml"
    config = toml.load(config_path)
    initial_condition = cs.vertcat(
        config["optimized"]["position"],
        config["optimized"]["orientation"],
        config["optimized"]["joint_position"],
    )

    robot = Robot()
    problem = Problem(robot, 50)

    problem.set_initial_com(np.array([0, 0, 0.57]))
    problem.set_final_com(np.array([-0.1, 0, 0.57]))
    problem.set_zero_intial_thrust()
    problem.set_middle_com_constraint(0.7)

    problem.initial_force_w = 1e1
    problem.intitial_joints_w = 1e6
    problem.initial_orientation_w = 1e5
    problem.running_com_acc_w = 1e5
    problem.running_com_vel_w = 1e7
    problem.running_mom_der_w = 1e4
    problem.running_mom_w = 1e4
    problem.running_joints_vel_upper_w = 5e5
    problem.running_joints_vel_lower_w = 1e5
    problem.running_postural_w = 1e6
    problem.running_force_w = 1e0
    problem.running_force_der_w = 1e-3
    problem.throtte_w = 1e3
    problem.running_orientation_w = 1e4

    problem.create(
        initial_condition=initial_condition,
    )
