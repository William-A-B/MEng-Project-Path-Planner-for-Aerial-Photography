import tkinter as tk
import PathPlannerDataStorage as ppds
import PathPlannerGUI as ppgui
from PathPlannerTSPSolver_2D import setupTSP
from PathPlannerTSPSolver_3D import PathPlannerTSPSolver_3D


class PathPlannerApp:
    def __init__(self, root_tk, tsp_solver_2d, tsp_solver_3d):
        self.data_handler = ppds.PathPlannerDataStorage("waypoints.xml")
        self.gui = ppgui.PathPlannerGUI(root_tk, tsp_solver_2d, tsp_solver_3d, self.data_handler)
        self.uav_waypoints = []


def main():
    print("Path Planner Application")

    # Initialise TSPSolver Object
    my_2d_tsp_solver = setupTSP.initialize()
    my_3d_tsp_solver = PathPlannerTSPSolver_3D.initialize()

    # Create root object for tkinter
    root_tk = tk.Tk()
    # Initialise path planner app
    app = PathPlannerApp(root_tk, my_2d_tsp_solver, my_3d_tsp_solver)
    # Start main GUI loop
    root_tk.mainloop()

    # Finished with tsp solver
    my_2d_tsp_solver.terminate()
    my_3d_tsp_solver.terminate()


if __name__ == '__main__':
    main()
