import matplotlib.pyplot as plt
import matplotlib.image as img
from utilities import FileReader

import argparse
import yaml

def plot_errors(file, map_file, distance_fn):
    headers, values=FileReader(file).read_file()

    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    plt.title(f"{distance_fn}: Trajectory, Actual")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()

    plt.plot([lin[0] for lin in values], [lin[1] for lin in values])

    with open(map_file, "r") as file:
        metadata = yaml.safe_load(file)

        res, origin, path = metadata["resolution"], metadata["origin"], metadata["image"]
        map_image = img.imread(path)

        # Return extent of map in world coordinates
        extent = [origin[0], origin[0] + map_image.shape[1] * res, origin[1], origin[1] + map_image.shape[0] * res]
        plt.imshow(map_image, cmap="gray", origin="upper", extent=extent)

    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file",required=True, help="Pose CSV file")
    parser.add_argument("--map", required=True, help="Map YAML file")
    parser.add_argument("--dist", required=True, help="Distance function used (Manhattan or Euclidean)")

    args = parser.parse_args()

    plot_errors(args.file, args.map, args.dist)


if __name__=="__main__":
    main()
