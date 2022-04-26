import dataclasses
from ast import Slice
from typing import Dict, List

import casadi as cs
import numpy as np
import toml
import yarp
from adam.casadi.computations import KinDynComputations


@dataclasses.dataclass
class Contact:
    idx: Slice


@dataclasses.dataclass
class Jet:
    idx: Slice
    coefficients: List[float]

    # def get_dynamics(self):
    #     x = cs.SX.sym("T-dotT", 2, 1)
    #     T = x[0]
    #     dT = x[1]
    #     u = cs.SX.sym("throttle", 1)
    #     c = self.coefficients
    #     ddT = (
    #         c[0] * T
    #         + c[1] * cs.np.multiply(T, T)
    #         + c[2] * dT
    #         + c[3] * cs.np.multiply(dT, dT)
    #         + c[4] * cs.np.multiply(T, dT)
    #         + cs.np.multiply(
    #             (c[5] + c[6] * T + c[7] * dT),
    #             (u + c[8] * cs.np.multiply(u, u)),
    #         )
    #         + c[9]
    #     )
    #     dxdt = cs.vertcat(dT, ddT)
    #     return cs.Function(
    #         "jet_dynamics",
    #         [x, u],
    #         [dxdt],
    #         ["jet_state", "throttle"],
    #         ["jet_state_derivative"],
    #     )

    def get_acceleration(self):
        T = cs.SX.sym("T", 1, 1)
        dT = cs.SX.sym("dotT", 1, 1)
        u = cs.SX.sym("throttle", 1)
        c = self.coefficients
        ddT = (
            c[0] * T
            + c[1] * cs.np.multiply(T, T)
            + c[2] * dT
            + c[3] * cs.np.multiply(dT, dT)
            + c[4] * cs.np.multiply(T, dT)
            + cs.np.multiply(
                (c[5] + c[6] * T + c[7] * dT),
                (u + c[8] * cs.np.multiply(u, u)),
            )
            + c[9]
        )
        return cs.Function("jet_acceleration", [T, dT, u], [ddT])


class Robot:
    def __init__(self) -> None:
        self.kinDyn = self._load_kinDyn()
        self.ndofs = self.kinDyn.NDoF
        self.define_inputs()
        print("Robot loaded.")

    def _load_kinDyn(self):
        config_path = "config/ironcub.toml"
        self.config = toml.load(config_path)

        rf = yarp.ResourceFinder()
        self.model_path = rf.findFileByName("model.urdf")
        self.joint_list = self.config["Joints"]["List"]
        self.jets_list = self.config["Jets"]["Frames"]
        self.jets_coefficients = self.config["Jets"]["Coefficients"]
        self.contacts_list = self.config["Contacts"]["Frames"]
        self.joint_limits = self.config["Joints"]["Limits"]
        self.joint_initial_position = self.config["Joints"]["Initial_position"]

        print(f"Importing joint list...\n{self.joint_list}")
        print(f"Importing jets list...\n{self.jets_list}")
        print(f"Importing contact points...\n{self.contacts_list}")

        return KinDynComputations(
            urdfstring=self.model_path,
            joints_name_list=self.joint_list,
            root_link="root_link",
        )

    def define_inputs(self):
        self.v = []
        self.v_idx = {}
        self.add_feet_contacts()
        self.add_jets()

    def get_joints_limits(self):
        lb = [lim[0] * np.pi / 180 for lim in self.joint_limits]
        ub = [lim[1] * np.pi / 180 for lim in self.joint_limits]
        return lb, ub

    def add_feet_contacts(self):
        self.fc = []
        self.contacts_dict = {}
        self.add_contacts(self.contacts_list)

    def add_contacts(self, frames: List[str]):
        [self.add_single_contact(frame) for frame in frames]

    def add_single_contact(self, frame: str):
        f = cs.SX.sym(f"{frame}_contact_force", 3, 1)
        self.fc = cs.vertcat(
            self.fc,
            f,
        )
        self.contacts_dict[frame] = Contact(
            idx=slice(
                self.fc.shape[0] - 3,
                self.fc.shape[0],
            ),
        )

    def add_jets(self):
        self.jets_dict: Dict[Jet] = {}
        self.t_jets = []
        [self.add_single_jet(frame, idx) for idx, frame in enumerate(self.jets_list)]

    def add_single_jet(self, frame: str, idx: int):
        T = cs.SX.sym(f"{frame}_jet_int", 1, 1)
        dT = cs.SX.sym(f"{frame}_jet_int", 1, 1)
        self.t_jets = cs.vertcat(self.t_jets, T)
        self.jets_dict[frame] = Jet(
            idx=idx,
            coefficients=self.jets_coefficients[idx],
        )

    def jets_dimension(self):
        return len(self.jets_list)

    def get_contact(self, frame: str):
        return self.fc[self.contacts_dict[frame].idx]

    def contact_forces_dimension(self):
        return self.fc.shape[0]


if __name__ == "__main__":
    robot = Robot()
    lb, ub = robot.get_joints_limits()
    print(robot.joint_initial_position)
    print(robot.v)
    print(robot.fc)
    print(robot.t_jets)
    # print(robot.get_momentum_dynamics())
