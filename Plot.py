#-------------------------------------------------------------#
#Plot.py file, part of the anti-flocking algorithm
#Lars Oerlemans, 4880110
# AE4350 Bio-Inspired Intelligence and Aerospace Applications
#-------------------------------------------------------------#
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import time, colorsys, subprocess
import numpy as np
import tkinter as tk
from Constants import Constants


def trajectory_patch(history_x, history_y, agent, agent_colors):
    string_path_data = [
        (mpath.Path.MOVETO, (history_x[agent][-2], history_y[agent][-2])),
        (mpath.Path.CURVE3, (history_x[agent][-2], history_y[agent][-2])),
        (mpath.Path.CURVE3, (history_x[agent][-1], history_y[agent][-1]))
    ]

    codes, verts = zip(*string_path_data)
    string_path = mpath.Path(verts, codes)
    # Ensure color is selected cyclically
    color = agent_colors[agent % len(agent_colors)]
    return mpatches.PathPatch(string_path, color=color, lw=2)

def get_screen_dimensions():
    # Create a hidden root window
    root = tk.Tk()
    root.update_idletasks()
    root.attributes('-fullscreen', True)  # Ensures the window is fullscreen
    root.state('iconic')  # Minimizes the window to prevent display
    
    # Extract geometry and DPI
    screen_geometry = root.winfo_geometry()
    screen_dpi = root.winfo_fpixels('1i')
    
    # Clean up by destroying the Tk root window
    root.destroy()
    
    # Parse the geometry string and calculate dimensions in inches
    screen_width_pixels = int(screen_geometry.split('x')[0])
    screen_height_pixels = int(screen_geometry.split('x')[1].split('+')[0])
    
    return screen_width_pixels / screen_dpi, screen_height_pixels / screen_dpi

# Example of how to use this function
width_inches, height_inches = get_screen_dimensions()


def assign_agent_colors():
    # Fixed list of distinct colors
    colors = ['red', 'blue', 'green', 'yellow', 'purple', 'orange', 'cyan', 'magenta', 'lime', 'olive']
    num_colors = len(colors)
    agent_colors = [colors[i % num_colors] for i in range(Constants.num_UAVS)]
    return agent_colors

def plot_simulation_map(ax_trajectories, history_x, history_y):
    """
    Plots the UAV trajectories and boundary/geo-fence on the provided axis.

    Parameters:
    ax_trajectories: The axis to plot the trajectories on.
    history_x: A list of lists containing the x-coordinates of the UAVs over time.
    history_y: A list of lists containing the y-coordinates of the UAVs over time.

    Returns:
    ax_trajectories: The axis with the plotted trajectories.
    """
    # Plot the trajectories of each UAV
    for i in range(Constants.num_UAVS):
        ax_trajectories.plot(history_x[i], history_y[i], linewidth=0.5)  # Plot each UAV's trajectory

    # Add boundary and geo-fence
    boundary = Rectangle((0, 0), Constants.area_width, Constants.area_length, linewidth=1, edgecolor='black', facecolor="gainsboro")
    geo_fence = Rectangle((Constants.Geo_Fence_width, Constants.Geo_Fence_width), Constants.area_width - 2 * Constants.Geo_Fence_width, Constants.area_length - 2 * Constants.Geo_Fence_width, linewidth=0.5, edgecolor='grey', facecolor="White")
    ax_trajectories.add_patch(boundary)
    ax_trajectories.add_patch(geo_fence)

    # Set plot limits and aspect ratio
    ax_trajectories.set_xlim(0, Constants.area_width)
    ax_trajectories.set_ylim(0, Constants.area_length)
    ax_trajectories.set_aspect('equal', adjustable='box')

    # Set title and remove ticks
    ax_trajectories.set_title("Trajectories plot")
    ax_trajectories.set_xticks([])
    ax_trajectories.set_yticks([])

    return ax_trajectories


def draw_obstacles(obstacles, ax_trajectories):
    for obs in obstacles:
        width = obs.ru[0]-obs.ld[0]
        height = obs.ru[1]-obs.ld[1]
        rect = Rectangle((obs.ld[0], obs.ld[1]), width, height, linewidth=1, edgecolor='black', hatch="////", facecolor="lightgrey")
        # Add the patch to the Axes
        if Constants.TRAJECTORY_PLOT: ax_trajectories.add_patch(rect)

