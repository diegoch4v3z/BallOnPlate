import matplotlib.pyplot as plt

def plot_1(t, data_1, name_1, units_1, figure_name):
    fig, axs = plt.subplots(1, 1, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs.plot(t, data_1)
    axs.set_title(name_1)
    axs.set_xlabel("Time (s)")
    axs.set_ylabel(units_1)

    axs.grid(True)
    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/' + figure_name

    plt.savefig(file)

def plot_2(t, data_1, data_2, name_1, units_1, name_2, units_2, figure_name):
    fig, axs = plt.subplots(1, 2, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)

    axs[0].grid(True), axs[1].grid(True)
    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/' + figure_name

    plt.savefig(file)

def plot_3(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)

    # Plot Graph 2 (third column, first row)
    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_3)

    # plt.ylim(-0.01, 0.01)
    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()
    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/' + figure_name

    plt.savefig(file)



def plot_2_in_1(t, data_1, data_2, name_1, units_1, name_2, units_2, figure_name):
    fig, ax = plt.subplots(figsize=(10, 7))

    # Plot data_1
    ax.plot(t, data_1, label=name_1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(units_1)

    # Plot data_2
    ax.plot(t, data_2, label=name_2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(units_2)

    ax.grid(True)
    ax.legend()
    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/' + figure_name

    plt.savefig(file)