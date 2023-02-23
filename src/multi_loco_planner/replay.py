import os
import pickle
import time
from pathlib import Path

import numpy as np
import typer
from liecasadi import SE3

from multi_loco_planner import Robot, Visualizer

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
        time.sleep(dataset["Time"] / dataset["knots"])


@app.command(help="Visualize the landing trajectory")
def landing():
    os.system("python datasets/plot_data_landing.py")


@app.command(help="Visualize the walking to flying trajectory")
def transition():
    os.system("python datasets/plot_data_transition.py")


@app.command(help="Visualize the take off and landing trajectory")
def take_off_and_landing():
    os.system("python datasets/plot_data_take_off_and_landing.py")


@app.command(
    help="Visualize the take-off trajectory",
)
def take_off():
    os.system("python datasets/plot_data_take_off.py")


@app.command(
    help="Type `show_trajectory load-dataset my-pickle-path` to visualize a trajectory stored in a pickle",
)
def load_dataset(data_path: str):
    viz = load_viz()
    with open(data_path, "rb") as f:
        dataset = pickle.load(f)
    update(viz, dataset=dataset)


if __name__ == "__main__":
    app()
