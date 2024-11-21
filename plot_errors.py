import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(2,1, figsize=(10,12))


    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values], label = "Kalman Filter")
    axes[0].plot([lin[4] for lin in values], [lin[5] for lin in values], label = "Odometery")
    axes[0].set_title("State Space")
    axes[0].legend()
    axes[0].grid()
    axes[0].set_xlabel("Position in X Direction (m)")
    axes[0].set_ylabel("Position in Y Direction (m)")

    
    axes[1].set_title("Measurements")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    box = axes[1].get_position()
    axes[1].set_position([box.x0, box.y0, box.width*0.8, box.height])

    axes[1].legend(loc='center left', bbox_to_anchor=(1, 0.5))
    axes[1].grid()
    axes[1].set_xlabel("ROS Header Timestamp (ns)")
    axes[1].set_ylabel("Value (Check legend for respective units)")

    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)


