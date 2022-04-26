import idyntree.swig as idyntree
from robot import Robot
import numpy as np


class Visualizer:
    def __init__(self, robot: Robot) -> None:
        robot_idyn = idyntree.ModelLoader()
        robot_idyn.loadReducedModelFromFile(robot.model_path, robot.joint_list)
        self.ndof = robot.ndofs
        self.robot = robot

        viz_options = idyntree.VisualizerOptions()
        viz_options.winHeight = 600
        viz_options.winWidth = 800

        self.viz = idyntree.Visualizer()
        self.viz.init(viz_options)
        self.viz.addModel(robot_idyn.model(), "iRonCub")

        self.set_environment()
        self.set_light()
        self.set_camera()

        # idyntree viz quantities
        self.H = idyntree.Transform.Identity()
        self.R = idyntree.Rotation()
        self.p = idyntree.Position()
        self.s = idyntree.VectorDynSize(self.ndof)

        self.prepare_jets()
        self.prepare_contact_forces()

    def set_environment(self):
        self.viz.enviroment().setElementVisibility("floor_grid", True)
        self.viz.enviroment().setElementVisibility("world_frame", True)
        self.viz.camera().animator().enableMouseControl(True)

    def set_camera(self):
        camera_pos_idyn = idyntree.Position()
        camera_pos = [-2, 0.5, 0.4]
        camera_pos_idyn.FromPython(np.array(camera_pos))
        self.viz.camera().setPosition(idyntree.Position.FromPython(camera_pos))
        target_pos_idyn = idyntree.Position()
        target_pos = [0, 0.0, 0.8]
        target_pos_idyn.FromPython(target_pos)
        self.viz.camera().setTarget(target_pos_idyn)

    def set_light(self):
        # additional lights
        self.viz.enviroment().addLight("sun1")
        self.viz.enviroment().lightViz("sun1").setType(idyntree.DIRECTIONAL_LIGHT)
        self.viz.enviroment().lightViz("sun1").setDirection(
            idyntree.Direction(-5, 0, 0)
        )
        self.viz.enviroment().addLight("sun2")
        self.viz.enviroment().lightViz("sun2").setType(idyntree.DIRECTIONAL_LIGHT)
        self.viz.enviroment().lightViz("sun2").setDirection(idyntree.Direction(5, 0, 0))

    def update(self, H, s, T, f) -> bool:
        self.R = idyntree.Rotation.FromPython(H[:3, :3])
        self.p = idyntree.Position.FromPython(H[:3, 3])
        self.H.setRotation(self.R)
        self.H.setPosition(self.p)
        self.s = idyntree.VectorDynSize.FromPython(s)
        self.viz.modelViz("iRonCub").setPositions(self.H, self.s)
        self.update_jets(T)
        self.update_contact_forces(f)
        self.viz.draw()
        return True

    def prepare_jets(self) -> bool:
        # setting jets
        self.jet_int_iDyn = idyntree.VectorDynSize(4)
        self.max_jets_int = 220
        orange = idyntree.ColorViz(1.0, 0.6, 0.1, 0.0)
        self.viz.modelViz("iRonCub").jets().setJetsFrames(self.robot.jets_list)
        self.viz.modelViz("iRonCub").jets().setJetsDimensions(0.02, 0.1, 0.3)
        for idx, jet in enumerate(self.robot.jets_list):
            self.viz.modelViz("iRonCub").jets().setJetColor(idx, orange)
            self.viz.modelViz("iRonCub").jets().setJetDirection(
                idx, idyntree.Direction(0, 0, 1.0)
            )
        return True

    def update_jets(self, jet_intensities) -> bool:
        self.jet_int_iDyn = idyntree.VectorDynSize.FromPython(
            jet_intensities / self.max_jets_int
        )
        self.viz.modelViz("iRonCub").jets().setJetsIntensity(self.jet_int_iDyn)
        return True

    def prepare_contact_forces(self):
        force = idyntree.Direction()
        for frame in self.robot.contacts_list:
            linkTransform = self.viz.modelViz("iRonCub").getWorldFrameTransform(frame)
            force.FromPython([0, 0, 0])
            self.viz.vectors().addVector(linkTransform.getPosition(), force)

    def update_contact_forces(self, f):
        force = idyntree.Direction()
        for idx, [frame, contact] in enumerate(self.robot.contacts_dict.items()):
            linkTransform = self.viz.modelViz("iRonCub").getWorldFrameTransform(frame)
            force = idyntree.Direction.FromPython(f[contact.idx] * 0.02)
            self.viz.vectors().updateVector(idx, linkTransform.getPosition(), force)


if __name__ == "__main__":
    robot = Robot()
    viz = Visualizer(robot=robot)
    H = np.eye(4)
    s = np.zeros(robot.ndofs)
    T = np.ones(4) * 100
    f = np.random.rand(len(robot.contacts_list) * 3) * 100

    # while viz.viz.run():
    #     viz.update(H, s, T, f)

    # Play a sample trajectory in a loop
    def play_sample_trajectory():

        update_rate = 60
        cycle_time = 3
        traj = np.repeat(
            np.zeros(robot.ndofs).reshape(-1, 1), cycle_time * update_rate, axis=1
        )
        beta = np.linspace(0, 1, traj.shape[1])
        traj[[2, 9, 10, 11, 22, 15, 16, 17]] = (
            0.39 + 0.685 * np.cos(beta),
            -beta,
            2.0 * beta,
            -beta,
            0.1 + beta * 1.56,
            -beta,
            2.0 * beta,
            -beta,
        )

        running = True

        while running:
            for k in range(traj.shape[1]):
                running = viz.viz.run()
                viz.update(H, traj[:, k], T, f)
            traj = np.flip(traj, 1)

    try:
        play_sample_trajectory()
    except:
        # an exception will be thrown when the window is closed
        pass
