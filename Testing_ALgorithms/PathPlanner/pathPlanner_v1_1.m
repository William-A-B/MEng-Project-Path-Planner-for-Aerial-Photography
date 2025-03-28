coords = [53.946555, -1.029298, 0]
latlim = [coords(1)-0.004, coords(1)+0.004];
lonlim = [coords(2)-0.004, coords(2)+0.004];

fig = figure;
g = geoaxes(fig, Basemap="satellite")
geolimits(latlim, lonlim)

% Read latitude and longitude from a GPX file
% Specify the GPX file name
filename = 'campus_east_coordinates.gpx';

% Read the GPX file as text
gpxData = fileread(filename);

% Extract latitude and longitude values using regular expressions
latPattern = 'lat="([\d\.\-]+)"';
lonPattern = 'lon="([\d\.\-]+)"';

latTokens = regexp(gpxData, latPattern, 'tokens');
lonTokens = regexp(gpxData, lonPattern, 'tokens');

% Convert extracted tokens from cell arrays to numeric arrays
latitudes = cellfun(@(x) str2double(x{1}), latTokens);
longitudes = cellfun(@(x) str2double(x{1}), lonTokens);

% Transpose matrices
latitudes = latitudes';
longitudes = longitudes';


lat_list = [53.945826, 53.945649, 53.945346, 53.945055, 53.944929]';
lon_list = [-1.031784, -1.032028, -1.031942, -1.032168, -1.031481]';

lakePoly = [latitudes, longitudes];

coverageSpace = uavCoverageSpace(Polygons=lakePoly, UseLocalCoordinates=false, ReferenceLocation=coords);

ReferenceHeight = 25;
coverageSpace.UnitWidth = 20;
show(coverageSpace, Parent=g);

setCoveragePattern(coverageSpace, 1);
coveragePattern = uavCoveragePlanner(coverageSpace, Solver="Exhaustive");

takeoffLocation = [53.946555, -1.029298, 0];
[waypoints,soln] = planCustom(coveragePattern,takeoffLocation);
hold on
geoplot(waypoints(:,1), waypoints(:,2), LineWidth=1.5);
geoplot(takeoffLocation(1), takeoffLocation(2), MarkerSize=25, Marker=".")
legend("","","Path","Takeoff/Landing")
hold off