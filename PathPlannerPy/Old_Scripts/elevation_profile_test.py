# import tkinter as tk
#
# # Sample elevation data (in meters)
# elevations = [0, 10, 40, 100, 120, 135, 150, 165, 140, 130, 125, 145, 160, 155]
#
# # GUI setup
# root = tk.Tk()
# root.title("Elevation Profile")
#
# # Canvas size
# width = 800
# height = 400
# padding = 50  # space for axes and labels
#
# canvas = tk.Canvas(root, width=width, height=height, bg='white')
# canvas.pack()
#
# # Graph area (excluding padding)
# graph_width = width - 2 * padding
# graph_height = height - 2 * padding
#
# # Scaling
# max_elev = max(elevations)
# min_elev = min(elevations)
# num_points = len(elevations)
#
# x_step = graph_width / (num_points - 1)
# y_scale = graph_height / (max_elev - min_elev)
#
# # Draw axes
# canvas.create_line(padding, height - padding, width - padding, height - padding)  # X axis
# canvas.create_line(padding, padding, padding, height - padding)  # Y axis
#
# # Draw Y-axis labels and ticks
# for i in range(min_elev, max_elev + 1, 10):
#     y = height - padding - (i - min_elev) * y_scale
#     canvas.create_line(padding - 5, y, padding + 5, y)
#     canvas.create_text(padding - 25, y, text=str(i), anchor="e")
#
# # Draw X-axis labels and ticks
# for i in range(num_points):
#     x = padding + i * x_step
#     canvas.create_line(x, height - padding - 5, x, height - padding + 5)
#     canvas.create_text(x, height - padding + 20, text=str(i), anchor="n")  # distance or index
#
# # Draw the elevation profile line
# for i in range(num_points - 1):
#     x1 = padding + i * x_step
#     y1 = height - padding - (elevations[i] - min_elev) * y_scale
#     x2 = padding + (i + 1) * x_step
#     y2 = height - padding - (elevations[i + 1] - min_elev) * y_scale
#     canvas.create_line(x1, y1, x2, y2, fill="green", width=2)
#     canvas.create_oval(x1 - 2, y1 - 2, x1 + 2, y1 + 2, fill="green")  # point marker
#
# # Add last point
# x_last = padding + (num_points - 1) * x_step
# y_last = height - padding - (elevations[-1] - min_elev) * y_scale
# canvas.create_oval(x_last - 2, y_last - 2, x_last + 2, y_last + 2, fill="green")
#
# root.mainloop()


# ======================================================================================================================
import tkinter

from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

import numpy as np


root = tkinter.Tk()
root.wm_title("Embedding in Tk")

fig = Figure(figsize=(5, 4), dpi=100)
t = np.arange(0, 3, .01)
fig.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
canvas.draw()

toolbar = NavigationToolbar2Tk(canvas, root)
toolbar.update()


def on_key_press(event):
    print("you pressed {}".format(event.key))
    key_press_handler(event, canvas, toolbar)


canvas.mpl_connect("key_press_event", on_key_press)

button = tkinter.Button(master=root, text="Quit", command=root.quit)

# Packing order is important. Widgets are processed sequentially and if there
# is no space left, because the window is too small, they are not displayed.
# The canvas is rather flexible in its size, so we pack it last which makes
# sure the UI controls are displayed as long as possible.
button.pack(side=tkinter.BOTTOM)
canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

tkinter.mainloop()
