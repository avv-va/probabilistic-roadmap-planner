import matplotlib.pyplot as plt
import math
from main import get_input_a, get_input_b, get_input_c
from prm import PRM


def compute_path_lenght(path):
    i = 0
    length = 0
    while i < len(path):
        point = path[i].point
        if i == len(path) - 1:
            break
        next_point = path[i+1].point
        length += math.dist(point, next_point)
        i += 1
    return length


def get_a_n_r_list():
    return [(200, 0.5), (200, 1), (200, 1.5), (200, 2), (500, 0.5), (500, 1), (500, 1.5), (500, 2)]


def get_b_c_n_r_list():
    return [(200, 1), (200, 2), (500, 1), (500, 2), (1000, 1), (1000, 2)]


def benchmark():
    # to override n and r further
    _, _, start, goal, x_range, y_range, obstacles = get_input_a()
    n_r_list = get_a_n_r_list()

    solutions = {}
    for n_r in n_r_list:
        solutions[n_r] = {
            False: 0,
            True: 0,
        }


    box_plot_path = []
    box_plot_comp = []
    where = []
    data = []
    labels = []


    for n_r in n_r_list:
        n, r = n_r[0], n_r[1]
        path_lengths = []
        computation_times = []

        for _ in range(0, 100):
            prm = PRM(n, r, start, goal, x_range, y_range, obstacles)
            path, _, computation_time, _, _, _, found_solution = prm.path_planning()
            # path = prm.smooth_pathing()

            path_lengths.append(compute_path_lenght(path))
            computation_times.append(computation_time)
            solutions[n_r][found_solution] += 1

        labels.append(n_r)
        box_plot_comp.append(computation_times)
        box_plot_path.append(path_lengths)
        
        where.append(f'{n_r}-s')
        where.append(f'{n_r}-f')
        data.append(solutions[n_r][True])
        data.append(solutions[n_r][False])


    plt.boxplot(box_plot_path, labels=labels)
    plt.title("path lengths")
    plt.savefig(f"plots_again/path_lengths_benchmark.png")
    plt.show()
    plt.clf()
    plt.cla()
    plt.close()


    plt.boxplot(box_plot_comp, labels=labels)
    plt.title("computation times")
    # plt.savefig(f"plots_again/computation_times_benchmark.png")
    plt.show()
    plt.clf()
    plt.cla()
    plt.close()

    plt.bar(where, data)
    # plt.savefig("plots_again/solutions_benchmark.png")
    plt.show()


if __name__ == "__main__":
    benchmark()
