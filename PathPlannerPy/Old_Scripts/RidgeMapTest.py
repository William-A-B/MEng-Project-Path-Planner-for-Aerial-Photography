from ridge_map import RidgeMap
import matplotlib.pyplot as plt

# Define the bounding box coordinates for Mt. Shasta
mt_shasta_bbox = (-122.5, 41.25, -122.0, 41.5)
rm = RidgeMap(mt_shasta_bbox)
values = rm.get_elevation_data(num_lines=150)
# Preprocess data
values = rm.preprocess(values=values,
                       lake_flatness=.5,
                       water_ntile=0,
                       vertical_ratio=900)
rm.plot_map(values=values,
            label='Mt. Shasta',
            label_y=0.1,
            label_x=0.55,
            label_size=40,
            linewidth=1,
            line_color=plt.get_cmap('copper'),
            kind='elevation')