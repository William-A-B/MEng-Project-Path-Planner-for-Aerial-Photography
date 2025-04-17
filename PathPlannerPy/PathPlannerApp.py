import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageTk
import tkinter as tk
from time import sleep
import PathPlannerDataStorage as ppds
import PathPlannerGUI as ppgui
from TSPSolver import TSPSolver
from tkinter import messagebox


class PathPlannerApp:
    def __init__(self, root_tk, tsp_solver):
        self.data_handler = ppds.PathPlannerDataStorage("waypoints.xml")
        self.gui = ppgui.PathPlannerGUI(root_tk, tsp_solver, self.data_handler)
        self.uav_waypoints = []

    def draw_calculated_uav_path(self):
        self.uav_waypoints = self.data_handler.read_temp_coordinate_data("./test_data/east_lake_coords.csv")

        self.gui.plot_waypoints()


def main():
    print("Path Planner Application")
    # messagebox.showinfo("Path Planner Application", "Starting Path Planner application, press ok to continue")

    # Initialise TSPSolver Object
    my_tsp_solver = TSPSolver.initialize()

    root_tk = tk.Tk()
    app = PathPlannerApp(root_tk, my_tsp_solver)
    # app.draw_calculated_UAV_path()
    root_tk.mainloop()

    # Finished with tsp solver
    my_tsp_solver.terminate()




if __name__ == '__main__':
    main()
