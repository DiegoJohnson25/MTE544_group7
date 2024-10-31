from planner import planner
from utilities import Logger

planner_obj = planner(1) # TRAJECTORY_PLANNER=1
trajectory_points = planner_obj.trajectory_planner()

traj_log = Logger("sigmoid_points.csv", ["x", "y"])

for pair in trajectory_points:
    traj_log.log_values(pair)

traj_log.save_log()