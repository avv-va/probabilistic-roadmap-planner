import matplotlib.pyplot as plt

from main import get_input_a
from prm import PRM

def benchmark():
    _, _, start, goal, x_range, y_range, obstacles = get_input_a() # to override n and r further
    n_r_list = [(200, 0.5), (200, 1), (200, 1.5), (200, 2), (500, 0.5), (500, 1), (500, 1.5), (500, 2)]
    
    solutions = {}
    for n_r in n_r_list:
        solutions[n_r] = {
            False: 0,
            True: 0,
    }
    path_lengths = []
    computation_times = []
    
    for n_r in n_r_list:
        n, r = n_r[0], n_r[1]
        prm = PRM(n, r, start, goal, x_range, y_range, obstacles)
        path, _, computation_time, _, _, _, found_solution = prm.path_planning()

        path_lengths.append(len(path) - 1)
        computation_times.append(computation_time)
        solutions[n_r][found_solution] += 1
    
    plt.boxplot(path_lengths)
    plt.title("path lengths")
    plt.savefig("plots/path_lengths_benchmark.png")
    plt.clf()
    plt.cla()
    plt.close()
    # plt.show()

    plt.boxplot(computation_times)
    plt.title("computation times")
    # plt.show()
    plt.savefig("plots/computation_times_benchmark.png")
    plt.clf()
    plt.cla()
    plt.close()

    for n_r in n_r_list:
        plt.bar(solutions[n_r][True], solutions[n_r][False])
        plt.title(f'n={n_r[0]}, r={n_r[1]}')
    plt.savefig("plots/solutions_benchmark.png")
    # plt.show()



if __name__ == "__main__":
    benchmark()