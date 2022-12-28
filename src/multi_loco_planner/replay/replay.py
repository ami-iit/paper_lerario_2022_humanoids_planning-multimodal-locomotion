import typer
import pickle
from pathlib import Path
from multi_loco_planner import Robot, Visualizer
import numpy as np
from liecasadi import SE3

app = typer.Typer()


landing_str = Path("datasets/pickles/03-14-15-57.pickle")
transition_str = Path("datasets/pickles/04-28-18-32.pickle")
take_off_str = Path("datasets/pickles/03-15-09-54.pickle")


def load_viz():
    robot = Robot()
    return Visualizer(robot=robot)


def update(viz, dataset):
    for k in range(dataset["knots"] + 1):
        pos = dataset["position"][:, k]
        quat = dataset["orientation"][:, k]
        H = np.array(SE3(pos, quat).transform())
        s = dataset["joint_pos"][:, k]
        t = dataset["thrust"][:, k]
        f = dataset["contact-forces"][:, k]
        viz.update(H, s, t, f)


@app.command()
def landing():
    viz = load_viz()
    with open(landing_str, "rb") as f:
        dataset = pickle.load(f)
    update(viz, dataset=dataset)


@app.command()
def transition():
    viz = load_viz()
    with open(transition_str, "rb") as f:
        dataset = pickle.load(f)
    update(viz, dataset=dataset)


@app.command()
def take_off():
    viz = load_viz()
    with open(take_off_str, "rb") as f:
        dataset = pickle.load(f)
    update(viz, dataset=dataset)


if __name__ == "__main__":
    app()
