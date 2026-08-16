import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Define parameters
gait_data_dir = Path(__file__).parent / "Gait_data"

# declare attributes
gait_files = {}
drop_start_idcs = 20
drop_end_idcs = 0

start = 0
stop = 10000
idx = np.arange(start, stop)
time = idx / 200

switch = 7462

def split_episodes(df):
    """Split a dataframe at rows marked with 'New episode' in the Episode column."""
    if df.empty or "Episode" not in df.columns:
        return [df]

    episodes = []
    last_index = 0

    for idx, value in df["Episode"].items():
        if isinstance(value, str) and value.strip().lower() == "new episode":
            if idx > last_index:
                episodes.append(df.iloc[last_index+drop_start_idcs:idx].copy())
            last_index = idx + 1

    if last_index < len(df):
        episodes.append(df.iloc[last_index+drop_start_idcs:].copy())

    return episodes if episodes else [df]


def find_mode_change_indices(df):
    """Return the row indices where the 'mode' column changes value."""
    if df.empty or "mode" not in df.columns:
        return []

    mode = df["mode"]
    change_idx = np.flatnonzero(mode.ne(mode.shift(fill_value=mode.iloc[0])))
    return change_idx.tolist()


def get_behaviour_df(behaviour, episode_index=0):
    """Return a behaviour dataframe, or the selected episode if data was split."""
    data = gait_files.get(behaviour, pd.DataFrame())

    if isinstance(data, list):
        if not data:
            return pd.DataFrame()
        if episode_index < 0:
            episode_index = len(data) + episode_index
        episode_index = max(0, min(episode_index, len(data) - 1))
        ed = data[episode_index]
        switch = find_mode_change_indices(ed)
        print(f"switch: {switch}")
        return ed

    return


def load_data():
    """Read gait data and store as a dictionary."""

    for csv_file in gait_data_dir.glob("*.csv"):
        file_name = csv_file.stem
        behaviour_name = file_name.split("-")[0]
        print(behaviour_name)
        df = pd.read_csv(csv_file)
        gait_files[behaviour_name] = split_episodes(df) if "Episode" in df.columns else df

    return gait_files


def plot_phases(behaviour="test", tail=False):
    """Plot all phase (p_XY) fields for the selected behaviour."""

    df = get_behaviour_df(behaviour)

    # Select columns whose names start with "p_"
    phase_columns = [col for col in df.columns if col.startswith("p_")]
    if (not tail): phase_columns=phase_columns[:-1]

    if not phase_columns:
        print(f"No phase fields found for behaviour '{behaviour}'.")
        return

    plt.figure(figsize=(10, 6))

    for col in phase_columns:
        # Use only the XY part, e.g. p_FL -> FL
        label = col[2:]
        plt.plot(np.sin(df[col]), label=label)

    plt.xlabel("Time")
    plt.ylabel("Phase")
    plt.title(f"Gait phases - {behaviour}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()



def plot_swimming(behaviour = "test"):

    df = get_behaviour_df(behaviour)

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # Plot a and r_S in another plot
    if "a" in df.columns:
        axes[1].plot(time, (df["a"][start:stop])/20, label=r"$a$")    
    if "b" in df.columns:
        axes[1].plot(time, (df["b"][start:stop])/20, label=r"$b$")
    axes[1].set_title("Spine Pitch and Buoyancy")
    axes[1].set_ylabel("Value relative to max")
    axes[1].set_xlabel("Time")
    ticks = np.arange(start, stop, 200) / 200
    axes[1].set_xticks(ticks)
    axes[1].legend()
    axes[1].grid(True)

    mark_mode_switch(axes[0])
    mark_mode_switch(axes[1])


    # Plot r_T and r_S in separate plot
    if "p_T" in df.columns:
        plot_phase(axes[0], df["p_T"], r"$\sin$ Tail phase")
    if "r_T" in df.columns:
        axes[0].plot(time, df["r_T"][start:stop]/40, label=r"$r_T$")
    axes[0].set_title("Tail Activity")
    axes[0].set_ylabel("Phase and relative amplitude")
    axes[0].legend()
    axes[0].grid(True)

    fig.tight_layout()
    fig.show()

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    mark_mode_switch(axes[0])
    mark_mode_switch(axes[1])

    # Plot sin(p_S) and sin(p_T) in one plot
    if "p_S" in df.columns:
        plot_phase(axes[0], df["p_S"], r"$\sin$ Spine phase")
    if "r_S" in df.columns:
        axes[0].plot(time, df["r_S"][start:stop]-1, label=r"$r_s$")
    axes[0].set_title("Spine Activity")
    axes[0].set_ylabel("Phase and relative amplitude")
    axes[0].legend()
    axes[0].grid(True)

    if "pitch" in df.columns:
        axes[1].plot(time, (df["pitch"][start:stop]), label="pitch")
    if "roll" in df.columns:
        axes[1].plot(time, (df["roll"][start:stop]), label="roll")
        axes[0].set_title("Spine Pitch and Buoyancy")

    axes[1].set_ylabel("Angle")
    axes[1].set_xlabel("Time")
    ticks1= np.arange(start, stop, 200) / 200
    axes[1].set_xticks(ticks)
    axes[1].legend()
    axes[1].grid(True)



    plt.tight_layout()
    plt.show()




def plot_walking(behaviour = "test"):

    df = get_behaviour_df(behaviour)

    walking_columns = [col for col in df.columns if ["L", "R", "S"].__contains__(col[-1])]
    # walking_columns.remove([col for col in walking_columns if ["o"].__contains__(col[0])])

    if not walking_columns:
        print(f"No Walking fields found for behaviour '{behaviour}'.")
        return

    groups = [
        ("Phases", r"$\theta$", [col for col in walking_columns if ["p"].__contains__(col[0])]),
        ("Amplitudes", r"$r$", [col for col in walking_columns if ["r"].__contains__(col[0])]),
        ("GRF", "grf", [col for col in walking_columns if ["C"].__contains__(col[0])])
    ]

    fig, axes = plt.subplots(
        len(groups), 1,
        figsize=(12, 16),
        sharex=True
    )

    for ax, (title, label, columns) in zip(axes, groups):

        mark_mode_switch(ax)
        for col in columns:
            if title=="Phases":
                plot_phase(ax, df[col], rf"{label}_{col[2:]}")
            elif title=="GRF":
                plot_grf(ax, df[col][start:stop], rf"{label}_{col[2:]}", simple=True)
            else:
                ax.plot(time, df[col][start:stop], label=rf"{label}_{col[2:]}")

        ax.set_title(title)
        ax.set_ylabel(f"{label}")
        ax.legend()
        ax.grid(True)

    axes[-1].set_xlabel("Time")
    ticks = np.arange(start, stop, 200) / 200
    axes[-1].set_xticks(ticks)

    plt.tight_layout()
    plt.show()

def plot_phase(ax, data, label):

    ax.plot(time, np.sin(data)[start:stop], label=label)

def plot_grf(ax, data, label, simple):

    if simple:
        data = data > 0

    ax.plot(time, data, label=label)


def plot_everything(behaviour="test"):

    df = get_behaviour_df(behaviour)

    groups = [
        ("Leg phases",                  ["p_FL", "p_RL", "p_RR", "p_FR"]),
        ("Leg amplitudes",              ["r_FL", "r_RL", "r_RR", "r_FR"]),
        ("Foot trajectory rotations",   ["o_FL", "o_RL", "o_RR", "o_FR"]),
        ("Spine and tail phases",       ["p_S", "p_T"]),
        ("Spine and Tail amplitudes",   ["r_S", "r_T"]),
        ("Spine pitch and buoyancy",    ["a", "b"]),
        ("Target values",               ["xT", "yT"]),
        ("Velocities",                  ["vx", "vy"])
    ]

    fig, axes = plt.subplots(
        len(groups), 1,
        figsize=(12, 16),
        sharex=True
    )

    for ax, (title, columns) in zip(axes, groups):

        for col in columns:
            if col in df.columns:
                ax.plot(df[col][50:], label=col)

        ax.set_title(title)
        ax.set_ylabel("Value")
        ax.legend()
        ax.grid(True)

    axes[-1].set_xlabel("Time")

    # fig.suptitle(f"Gait data - {behaviour}", fontsize=16)

    plt.tight_layout()
    plt.show()


def plot_custom(groups, behaviour = "test"): 

    df = get_behaviour_df(behaviour)

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


def plot_target_errors(behaviour="test"):
    """Plot target vs actual x/y velocities over time in seconds."""

    df = get_behaviour_df(behaviour)

    if df.empty:
        print(f"No gait data found for behaviour '{behaviour}'.")
        return
    
    df_plot = df.iloc[start:stop]
    time = np.arange(len(df_plot)) / 200.0

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # x-direction: vx and Tx
    if "vx" in df_plot.columns:
        axes[0].plot(time, df_plot["vx"], label="vx")
    if "xT" in df_plot.columns:
        axes[0].plot(time, df_plot["xT"], linestyle="--", label="Tx")
    axes[0].set_title(f"X velocity tracking")
    axes[0].set_ylabel("Velocity")
    axes[0].grid(True)
    axes[0].legend()

    # y-direction: vy and Ty
    if "vy" in df_plot.columns:
        axes[1].plot(time, df_plot["vy"], label="vy")
    if "yT" in df_plot.columns:
        axes[1].plot(time, df_plot["yT"], linestyle="--", label="Ty")
    axes[1].set_title(f"Y velocity tracking")
    axes[1].set_ylabel("Velocity")
    axes[1].set_xlabel("Time (s)")
    axes[1].grid(True)
    axes[1].legend()

    fig.tight_layout()
    plt.show()

def mark_mode_switch(ax):
    ax.vlines(time[switch], 0, 1, label="mode switch")