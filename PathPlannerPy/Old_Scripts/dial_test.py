# # import tkinter as tk
# # import math
# #
# # class WindDial(tk.Canvas):
# #     def __init__(self, master=None, size=200, **kwargs):
# #         super().__init__(master, width=size, height=size, bg='white', **kwargs)
# #         self.size = size
# #         self.center = size // 2
# #         self.radius = self.center - 10
# #         self.angle = 0  # in degrees
# #         self.indicator = None
# #
# #         self.draw_base()
# #         self.draw_indicator()
# #
# #         self.bind("<B1-Motion>", self.on_drag)
# #         self.bind("<Button-1>", self.on_drag)
# #
# #     def draw_base(self):
# #         # Draw dial circle
# #         self.create_oval(10, 10, self.size - 10, self.size - 10, outline='black', width=2)
# #         # Draw compass directions
# #         directions = ['N', 'E', 'S', 'W']
# #         for i, label in enumerate(directions):
# #             angle = math.radians(i * 90)
# #             x = self.center + math.cos(angle) * (self.radius + 10)
# #             y = self.center - math.sin(angle) * (self.radius + 10)
# #             self.create_text(x, y, text=label, font=("Arial", 10, "bold"))
# #
# #     def draw_indicator(self):
# #         # Draw the wind direction indicator arrow
# #         angle_rad = math.radians(self.angle)
# #         x = self.center + self.radius * math.sin(angle_rad)
# #         y = self.center - self.radius * math.cos(angle_rad)
# #         if self.indicator:
# #             self.delete(self.indicator)
# #         self.indicator = self.create_line(
# #             self.center, self.center, x, y,
# #             arrow=tk.LAST, fill="red", width=3
# #         )
# #
# #     def on_drag(self, event):
# #         # Calculate angle from center to cursor
# #         dx = event.x - self.center
# #         dy = self.center - event.y  # Invert Y axis for correct angle
# #         angle = math.degrees(math.atan2(dx, dy)) % 360
# #         self.angle = angle
# #         self.draw_indicator()
# #         print(f"Wind direction: {int(self.angle)}°")
# #
# # # Example usage in a Tkinter app
# # if __name__ == "__main__":
# #     root = tk.Tk()
# #     root.title("Wind Direction Dial")
# #
# #     dial = WindDial(root, size=250)
# #     dial.pack(padx=20, pady=20)
# #
# #     root.mainloop()
#
#
#
#
# import tkinter as tk
# import math
#
# class SimpleWindDial(tk.Canvas):
#     def __init__(self, master, size=200):
#         super().__init__(master, width=size, height=size, bg="white", highlightthickness=0)
#         self.size = size
#         self.center = size // 2
#         self.radius = self.center - 10
#         self.angle = 0  # Wind direction in degrees
#
#         # Draw base circle and directions
#         self.create_oval(10, 10, size - 10, size - 10, outline="black", width=2)
#         for i, label in enumerate(["N", "E", "S", "W"]):
#             a = math.radians(i * 90)
#             x = self.center + math.cos(a) * (self.radius + 10)
#             y = self.center - math.sin(a) * (self.radius + 10)
#             self.create_text(x, y, text=label, font=("Arial", 10, "bold"))
#
#         # Draw arrow
#         self.arrow = self.create_line(self.center, self.center,
#                                       self.center, self.center - self.radius,
#                                       arrow=tk.LAST, width=3, fill="red")
#
#         # Mouse interaction
#         self.bind("<Button-1>", self.update_arrow)
#         self.bind("<B1-Motion>", self.update_arrow)
#
#     def update_arrow(self, event):
#         dx = event.x - self.center
#         dy = self.center - event.y  # Flip Y for correct angle
#         angle = math.degrees(math.atan2(dx, dy)) % 360
#         self.angle = angle
#
#         x = self.center + self.radius * math.sin(math.radians(angle))
#         y = self.center - self.radius * math.cos(math.radians(angle))
#         self.coords(self.arrow, self.center, self.center, x, y)
#
#         print(f"Wind direction: {int(self.angle)}°")
#
# # Run the dial
# if __name__ == "__main__":
#     root = tk.Tk()
#     root.title("Simple Wind Direction Dial")
#     dial = SimpleWindDial(root, size=250)
#     dial.pack(padx=20, pady=20)
#     root.mainloop()


import tkinter as tk
import math

def on_select(direction):
    direction_map = {
        "N": 0,
        "NE": math.pi / 4,
        "E": math.pi / 2,
        "SE": 3 * math.pi / 4,
        "S": math.pi,
        "SW": 5 * math.pi / 4,
        "W": 3 * math.pi / 2,
        "NW": 7 * math.pi / 4
    }
    angle_rad = direction_map[direction]
    print(f"Selected wind direction: {direction} ({angle_rad:.2f} radians)")

root = tk.Tk()
root.title("Wind Direction (Dropdown)")

options = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
var = tk.StringVar(value="N")

menu = tk.OptionMenu(root, var, *options, command=on_select)
menu.pack(padx=20, pady=20)

root.mainloop()

