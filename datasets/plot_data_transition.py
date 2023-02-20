import pickle
import time as tm

import matplotlib.pyplot as plt
import numpy as np
from liecasadi import SE3

from multi_loco_planner import Robot, Visualizer

if __name__ == "__main__":
    with open("datasets/pickles/04-28-18-32.pickle", "rb") as f:
        dataset = pickle.load(f)

    time = dataset["Time"]
    print(time)
    N = dataset["knots"]
    time = np.linspace(0.0, time, N + 1)

    plt.figure()
    plt.title(r"Throttle")
    [b, c, d, e] = plt.plot(
        time,
        np.transpose(dataset["throttle"]),
        label=["$u_1$", "$u_2$", "$u_3$", "$u_4$"],
    )
    plt.plot(time, np.ones(len(time)) * 100, "--", linewidth=2, color="r")
    plt.plot(time, np.ones(len(time)) * 0, "--", linewidth=2, color="r")
    plt.legend(
        [b, c, d, e],
        ["$u_1$", "$u_2$", "$u_3$", "$u_4$"],
        loc="best",
        fancybox=True,
        framealpha=0.5,
    )

    plt.axvspan(5.28, 5.5, alpha=0.2, color="red")

    plt.text(4.5, 100, "$u_{max}$", va="center", ha="center", backgroundcolor="w")
    plt.text(4.5, 0, "$u_{min}$", va="center", ha="center", backgroundcolor="w")
    plt.grid("True")
    plt.xlabel("Time [s]")
    plt.ylabel("Throttle [\%]")
    plt.savefig("throttle.pdf")

    fig = plt.figure()

    plt.title(r"Thrust")
    [b, c, d, e] = plt.plot(
        time,
        np.transpose(dataset["thrust"]),
        label=["Jet$_1$", "Jet$_2$", "Jet$_3$", "Jet$_4$"],
    )
    plt.plot(
        time,
        np.tile([160, 160, 220, 220], (len(time), 1)),
        "--",
        linewidth=2,
        color="r",
    )

    plt.axvspan(5.28, 5.5, alpha=0.2, color="red")

    t = plt.text(
        4.5, 160, "$T^{max}_{1,2}$", va="center", ha="center", backgroundcolor="w"
    )
    plt.text(4.5, 220, "$T^{max}_{3,4}$", va="center", ha="center", backgroundcolor="w")
    plt.text(4.5, 0, "$T^{min}$", va="center", ha="center", backgroundcolor="w")
    plt.plot(time, np.ones(len(time)) * 0, "--", linewidth=2, color="r")
    plt.legend(
        [b, c, d, e],
        ["$T_1$", "$T_2$", "$T_3$", "$T_4$"],
        loc="best",
        fancybox=True,
        framealpha=0.5,
    )
    plt.grid("True")
    plt.xlabel("Time [s]")
    plt.ylabel("Thrust [N]")

    plt.savefig("thrust.pdf")
    plt.figure()

    plt.title("Contact forces - vertical component")

    f1 = np.ones([1, 4]) @ dataset["contact-forces"][[2, 5, 8, 11], :]
    f2 = np.ones([1, 4]) @ dataset["contact-forces"][[14, 17, 20, 23], :]
    plt.plot(time, f1.squeeze(), label="left foot")
    plt.plot(time, f2.squeeze(), label="right foot")

    plt.axvspan(5.28, 5.5, alpha=0.2, color="red")
    plt.legend(loc="best", fancybox=True, framealpha=0.5)
    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.grid("True")
    plt.savefig("contact_forces.pdf")

    viz = Visualizer(robot=Robot())

    for k in range(dataset["knots"] + 1):
        pos = dataset["position"][:, k]
        quat = dataset["orientation"][:, k]
        H = np.array(SE3(pos, quat).transform())
        s = dataset["joint_pos"][:, k]
        t = dataset["thrust"][:, k]
        f = dataset["contact-forces"][:, k]
        viz.viz.run()
        viz.update(H, s, t, f)
        tm.sleep(dataset["Time"] / dataset["knots"])

    plt.show()
