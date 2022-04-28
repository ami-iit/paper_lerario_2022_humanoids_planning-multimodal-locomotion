import pickle
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":

    with open("datasets/pickles/03-14-15-57.pickle", "rb") as f:
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

    plt.axvspan(0, 0.3, alpha=0.2, color="red")

    plt.text(5.5, 100, "$u_{max}$", va="center", ha="center", backgroundcolor="w")
    plt.text(5.5, 0, "$u_{min}$", va="center", ha="center", backgroundcolor="w")
    plt.grid("True")
    plt.xlabel("Time [s]")
    plt.ylabel("Throttle [%]")
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

    plt.axvspan(0, 0.3, alpha=0.2, color="red")

    t = plt.text(
        5.5, 160, "$T^{max}_{1,2}$", va="center", ha="center", backgroundcolor="w"
    )
    plt.text(5.5, 220, "$T^{max}_{3,4}$", va="center", ha="center", backgroundcolor="w")
    plt.text(5.5, 0, "$T^{min}$", va="center", ha="center", backgroundcolor="w")
    plt.plot(time, np.ones(len(time)) * 0, "--", linewidth=2, color="r")
    plt.legend(
        [b, c, d, e],
        ["$T_1$", "$T_2$", "$T_3$", "$T_4$"],
        loc="best",
        fancybox=True,
        framealpha=0.5,
    )
    plt.grid("True")
    plt.xlabel("Time")
    plt.ylabel("Thrust [N]")
    plt.savefig("thrust.pdf")

    plt.figure()
    plt.title("Contact forces - vertical component")

    f1 = np.ones([1, 4]) @ dataset["contact-forces"][[2, 5, 8, 11], :]
    f2 = np.ones([1, 4]) @ dataset["contact-forces"][[14, 17, 20, 23], :]
    plt.plot(time, f1.squeeze(), label="left foot")
    plt.plot(time, f2.squeeze(), label="right foot")
    plt.axvspan(0, 0.3, alpha=0.2, color="red")
    plt.legend(loc="best", fancybox=True, framealpha=0.5)
    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.grid("True")

    plt.savefig("contact_forces.pdf")

    plt.show()
