from shapely.geometry import Polygon
from prm import PRM

def get_input_a():
    n = 200
    r = 1
    start = (0, 0)
    goal = (10, 0)
    x_range = (-1, 11)
    y_range = (-3, 3)
    obs1 = Polygon([(3.5, 1.5), (4.5, 1.5), (4.5, 0.5), (3.5, 0.5)])
    obs2 = Polygon([(6.5, -0.5), (7.5, -0.5), (7.5, -1.5), (6.5, -1.5)])

    return n, r, start, goal, x_range, y_range, [obs1, obs2]

if __name__ == "__main__":
    n, r, start, goal, x_range, y_range, obstacles = get_input_a()
    prm = PRM(n, r, start, goal, x_range, y_range, obstacles)
    prm.path_planning()
    
    pass
