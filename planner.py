from math import e

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

# Type of trajectory
TR_PARABOLA=0; TR_SIGMOID=1

# Use a global variable to select the trajectory, as recommended. Default = parabola
# Change this value to change the trajectory
tr_type = TR_PARABOLA

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]

        # Use a step value of 0.1 for the x values
        x_step = 0.1
        make_x_values = lambda upper_bound: [x * x_step for x in range(int(upper_bound / x_step) + 1)]

        if tr_type == TR_PARABOLA:
            # Upper bound of 1.5 for the parabolic trajectory
            x_values = make_x_values(1.5)
            return [(x, x ** 2) for x in x_values]
        else:
            # Upper bound of 2.5 for the sigmoid trajectory
            x_values = make_x_values(2.5)
            return [(x, 2 / (1 + e^(-2 * x)) - 1) for x in x_values]
