import matplotlib.pyplot as plt
from utilities import FileReader

def make_plots(files, title):
    fig, axes = plt.subplots(3,2, figsize=(14,14))
    fig.suptitle(title)

    # x-y, x-t, y-t, theta-t
    make_plot_row(axes, files[0], row=0, sp_title="X vs. Y", state_title="X, Y, and Theta vs. Time", create_points=True, point_file="/home/diego/MTE544_group7/Data/NewParabola/parabola_points.csv",
                  y1 = "X-Coordinate [m]", x1 = "Y-Coordinate [m]", y2 = "Robot Pose [m], [m], [rad]", x2 = "Time [ns]")

    # e-t, edot-t, e-edot (linear)
    make_plot_row(axes, files[1], row=1, sp_title="e vs. e_dot (Linear)", state_title="e, e_dot, and e_int vs. Time (Linear)",
                  y1 = "Linear Error [m]", x1 = "Linear Error Dot [m/s]", y2 = "Linear Errors [m], [m/s], [m*s]", x2 = "Time [ns]")

    # e-t, edot-t, e-edot (linear)
    make_plot_row(axes, files[2], row=2, sp_title="e vs. e_dot (Angular)", state_title="e, e_dot, and e_int vs. Time (Angular)",
                  y1 = "Angular Error [rad]", x1 = "Angular Error Dot [rad/s]", y2 = "Angular Errors [rad], [rad/s], [rad*s]", x2 = "Time [ns]")
    
    plt.tight_layout()
    plt.savefig(title + ".png", format='png', dpi=300)
    plt.show()
    plt.close()

def make_plot_row(axes, pose_file, row=0, sp_title="", state_title="", create_points=False, point_file="", x1 = "", y1 = "", x2 = "", y2 = ""):
    headers, values = FileReader(pose_file).read_file()
    time_list = []
    first_stamp=values[0][-1]

    for val in values:
        time_list.append(val[-1] - first_stamp)

    # State space (x vs. y)
    axes[row, 0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[row, 0].set_title(sp_title)
    axes[row, 0].grid()
    axes[row, 0].set_xlabel(x1)
    axes[row, 0].set_ylabel(y1)

    if create_points:
        _, p_values = FileReader(point_file).read_file()
        axes[row, 0].scatter([lin[0] for lin in p_values], [lin[1] for lin in p_values])

    # Individual states x, y, and theta vs t
    for i in range(0, len(headers) - 1):
        axes[row, 1].plot(time_list, [lin[i] for lin in values], label= headers[i])
    axes[row, 1].set_title(state_title)
    axes[row, 1].grid()
    axes[row, 1].legend(loc="upper right", bbox_to_anchor=(1, 1.05))
    axes[row, 1].set_xlabel(x2)
    axes[row, 1].set_ylabel(y2)

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument('--title', required=True, help="Figure title")
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    # Assumes the following order: robot_pose, linear, angular
    make_plots(args.files, args.title)
