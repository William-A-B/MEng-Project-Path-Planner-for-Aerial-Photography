import numpy as np
import matplotlib.pyplot as plt
import folium
import tkinter
import tkintermapview


def left_click_event(coordinates_tuple):
    print("Left click event with coordinates:", coordinates_tuple)


def gui_func():
    # create tkinter window
    root_tk = tkinter.Tk()
    root_tk.geometry(f"{800}x{600}")
    root_tk.title("map_view_example.py")

    # create map widget
    map_widget = tkintermapview.TkinterMapView(root_tk, width=800, height=600, corner_radius=0)
    map_widget.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

    # set current widget position and zoom
    map_widget.set_position(53.946555, -1.029298)  # York, Campus East
    map_widget.set_zoom(15)

    # map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google satellite
    # map_widget.set_tile_server("https://wmts.geo.admin.ch/1.0.0/ch.swisstopo.pixelkarte-farbe/default/current/3857/{z}/{x}/{y}.jpeg")  # swisstopo map

    # set current widget position by address
    map_widget.set_address("colosseo, rome, italy")

    # set current widget position by address
    marker_1 = map_widget.set_address("colosseo, rome, italy", marker=True)

    # print(marker_1.position, marker_1.text)  # get position and text

    # marker_1.set_text("Colosseo in Rome")  # set new text
    # marker_1.set_position(48.860381, 2.338594)  # change position
    # marker_1.delete()

    map_widget.add_left_click_map_command(left_click_event)

    root_tk.mainloop()



def main():
    print("Path Planner Application")

    gui_func()


if __name__ == '__main__':
    main()