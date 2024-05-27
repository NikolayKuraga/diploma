import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# need wrap this into function
stat_directory = Path.home().joinpath('Work').joinpath(
    'balloon_ws').joinpath('src').joinpath('stat')
nested_dirs = [d for d in os.listdir(stat_directory)
               if os.path.isdir(stat_directory.joinpath(d))]
sorted_dirs = sorted(nested_dirs, key=lambda x: os.path.getctime(
    stat_directory.joinpath(x)), reverse=True)[:1]

latest_stat_directory = None
if len(nested_dirs) > 0:
    latest_stat_directory = stat_directory.joinpath(sorted_dirs[0])


def load_and_plot_speed():
    if latest_stat_directory == None:
        return

    file = latest_stat_directory.joinpath('speed_approach.txt')
    array = np.loadtxt(file, delimiter=',')
    speed_arr = np.transpose(array)

    x = speed_arr[0]
    y = speed_arr[1]

    plt.plot(x, y)
    plt.title("Speed Approach between drone and ball")
    plt.xlabel("Time")
    plt.ylabel("Speed")
    plt.get_current_fig_manager().canvas.set_window_title(file)
    plt.show()


def load_and_plot_distance():
    if latest_stat_directory == None:
        return

    file = latest_stat_directory.joinpath('distance.txt')
    array = np.loadtxt(file, delimiter=',')
    speed_arr = np.transpose(array)

    x = speed_arr[0]
    y = speed_arr[1]

    plt.plot(x, y)
    plt.title("Distance between drone and ball")
    plt.xlabel("Time")
    plt.ylabel("Distance")
    plt.get_current_fig_manager().canvas.set_window_title(file)
    plt.show()


load_and_plot_speed()
# load_and_plot_distance()
