import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageTk
import tkinter as tk
from time import sleep
import PathPlannerDataStorage as ppds
import PathPlannerGUI as ppgui

class PathPlannerApp:
    def __init__(self, root_tk):
        self.gui = ppgui.PathPlannerGUI(root_tk)

def main():
    print("Path Planner Application")

    root_tk = tk.Tk()
    app = PathPlannerApp(root_tk)
    root_tk.mainloop()


if __name__ == '__main__':
    main()
