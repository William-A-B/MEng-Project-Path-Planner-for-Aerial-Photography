coords = [53.946555, -1.029298]
latlim = [coords(1)-0.004, coords(1)+0.004];
lonlim = [coords(2)-0.004, coords(2)+0.004];

fig = figure;
g = geoaxes(fig, Basemap="satellite")
geolimits(latlim, lonlim)