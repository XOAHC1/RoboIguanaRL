import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt

# Define parameters
gait_data_dir = Path(__file__).parent / "Gait_data"

# declare attributes
gait_files = {}


def load_data():
    """Read gait data and store as a dictionary."""

    for csv_file in gait_data_dir.glob("*.csv"):
        file_name = csv_file.stem
        behaviour_name = file_name.split("_")[0]
        gait_files[behaviour_name] = pd.read_csv(csv_file)

    return gait_files


def plot_phases(behaviour="test"):
    """Plot all phase (p_XY) fields for the selected behaviour."""

    df = gait_files[behaviour]

    # Select columns whose names start with "p_"
    phase_columns = [col for col in df.columns if col.startswith("p_")]

    if not phase_columns:
        print(f"No phase fields found for behaviour '{behaviour}'.")
        return

    plt.figure(figsize=(10, 6))

    for col in phase_columns:
        # Use only the XY part, e.g. p_FL -> FL
        label = col[2:]
        plt.plot(df[col], label=label)

    plt.xlabel("Time")
    plt.ylabel("Phase")
    plt.title(f"Gait phases - {behaviour}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()



def plot_swimming(behaviour = "test"):

    df = gait_files[behaviour]

    swim_columns = [col for col in df.columns if ["T", "S"].__contains__(col[-1])]

    if not swim_columns:
        print(f"No tail fields found for behaviour '{behaviour}'.")
        return

    plt.figure(figsize=(10, 6))

    for col in swim_columns:
        # Use only the XY part, e.g. p_FL -> FL
        label = col
        plt.plot(df[col], label=label)

    plt.xlabel("Time")
    plt.ylabel("Phase")
    plt.title(f"Gait phases - {behaviour}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_walking(behaviour = "test"):

    df = gait_files[behaviour]

    walking_columns = [col for col in df.columns if ["L", "R"].__contains__(col[2:])]

    if not walking_columns:
        print(f"No tail fields found for behaviour '{behaviour}'.")
        return

    plt.figure(figsize=(10, 6))

    for col in walking_columns:
        # Use only the XY part, e.g. p_FL -> FL
        label = col
        plt.plot(df[col], label=label)

    plt.xlabel("Time")
    plt.ylabel("Phase")
    plt.title(f"Gait phases - {behaviour}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_everything(behaviour="test"):

    df = gait_files[behaviour]

    groups = [
        ("Leg phases",                  ["p_FL", "p_RL", "p_RR", "p_FR"]),
        ("Leg amplitudes",              ["r_FL", "r_RL", "r_RR", "r_FR"]),
        ("Foot trajectory rotations",   ["o_FL", "o_RL", "o_RR", "o_FR"]),
        ("Spine and tail phases",       ["p_S", "p_T"]),
        ("Spine and Tail amplitudes",   ["r_S", "r_T"]),
        ("SPine pitch and buoyancy",    ["a", "b"]),
    ]

    fig, axes = plt.subplots(
        len(groups), 1,
        figsize=(12, 16),
        sharex=True
    )

    for ax, (title, columns) in zip(axes, groups):

        for col in columns:
            if col in df.columns:
                ax.plot(df[col], label=col)

        ax.set_title(title)
        ax.set_ylabel("Value")
        ax.legend()
        ax.grid(True)

    axes[-1].set_xlabel("Time")

    # fig.suptitle(f"Gait data - {behaviour}", fontsize=16)

    plt.tight_layout()
    plt.show()


def plot_custom(groups, behaviour = "test"): 

    df = gait_files[behaviour]

    fig, axes = plt.subplots(
        len(groups), 1,
        figsize=(12, 16),
        sharex=True
    )

    for ax, (title, columns) in zip(axes, groups):

        for col in columns:
            if col in df.columns:
                ax.plot(df[col], label=col)

        ax.set_title(title)
        ax.set_ylabel("Value")
        ax.legend()
        ax.grid(True)

    axes[-1].set_xlabel("Time")

    fig.suptitle(f"Gait data - {behaviour}", fontsize=16)

    plt.tight_layout()
    plt.show()
