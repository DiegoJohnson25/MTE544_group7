import matplotlib.pyplot as plt
from utilities import FileReader

def make_plots(files, title):
    fig, axes = plt.subplots(3,2, figsize=(14,6))
    fig.suptitle(title)

    # x-y, x-t, y-t, theta-t
    make_plot_row(axes, files[0], row=0, sp_title="x vs. y", state_title="x vs. t, y vs. t, theta vs. t", create_points=True, point_file="C:/Users/hench/Downloads/MTE544_Lab2_New_Data/MTE544_Lab2_New_Data/NewSigmoid/sigmoid_points.csv")

    # e-t, edot-t, e-edot (linear)
    make_plot_row(axes, files[1], row=1, sp_title="e vs. edot (linear)", state_title="e vs. edot vs. t (linear)")

    # e-t, edot-t, e-edot (linear)
    make_plot_row(axes, files[2], row=2, sp_title="e vs. edot (angular)", state_title="e vs. edot vs. t (angular)")

    plt.show()

def make_plot_row(axes, pose_file, row=0, sp_title="", state_title="", create_points=False, point_file=""):
    headers, values = FileReader(pose_file).read_file()
    time_list = []
    first_stamp=values[0][-1]

    for val in values:
        time_list.append(val[-1] - first_stamp)

    # State space (x vs. y)
    axes[row, 0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[row, 0].set_title(sp_title)
    axes[row, 0].grid()

    if create_points:
        _, p_values = FileReader(point_file).read_file()
        axes[row, 0].scatter([lin[0] for lin in p_values], [lin[1] for lin in p_values])

    # Individual states x, y, and theta vs t
    for i in range(0, len(headers) - 1):
        axes[row, 1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    axes[row, 1].set_title(state_title)
    axes[row, 1].grid()
    axes[row, 1].legend(loc="upper right", bbox_to_anchor=(1, 1.05))

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument('--title', required=True, help="Figure title")
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    # Assumes the following order: robot_pose, linear, angular
    make_plots(args.files, args.title)
