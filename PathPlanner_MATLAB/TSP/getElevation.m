function elevation = getElevation(latitude, longitude, batchsize)
% GETELEVATION retrieves elevation data for given GPS coordinates using the
%   Open-Elevation API.
% 
% Published under BSD-3-Clause License     copyright (c) 2024, Luboš Smolík
% v1.1.1 (built 23. July 2024)                 email: smolik(at)vzuplzen.cz 
%
% elevation = getElevation(latitude, longitude) takes arrays of latitude
%   and longitude values and returns the corresponding elevation values
%   using the SRTM 250 model hosted at https://open-elevation.com/. Input
%   arrays are fetched in batches to avoid server overload and potential
%   errors due to request size limits.
%   Latitude must be a numeric array of values between -90 and 90,
%   longitude must be a numeric array of values between -180 and 180.
%   NaNs as inputs are currently not supported because they results in
%   "Internal Server Error".
%
% elevation = getElevation(latitude, longitude, batchsize) also specifies
%   the batch size.

% EXAMPLE #1
% % GPS coordinates of the Death Valley
% latValley  =   36.250278;
% lonValley  = -116.825833;
% % GPS coordinates of Mt. Whitney
% latWhitney =   36.578581;
% lonWhitney = -118.291995;
%
% % Get elevations
% pts = 51;
% elevation = getElevation(linspace(latValley, latWhitney, pts), ...
%                          linspace(lonValley, lonWhitney, pts));
% % Plot results
%  plot(linspace(0, 130, pts), elevation);
%  title("Elevation profile from Death Valley to Mt. Whitney");
%  xlabel("Distance (km)");
%  ylabel("Elevation (m)");

% EXAPLE #2
% % GPS latitudes
% latMin = 36; % South of the Sequoia National Forest
% latMax = 37; % North of Mt. Whitney
% % GPS longitudes
% lonMin = -118.291995; % Mt. Whitney
% lonMax = -116.825833; % Death Valley
% 
% % Get elevations
% pts = 101;
% [lat, lon] = meshgrid(linspace(latMin, latMax, pts), ...
%                       linspace(lonMin, lonMax, pts));
% elevation = getElevation(lat, lon);
% 
% % Plot results
% contourf(lat, lon, elevation, 25);
% view([-90 90]);
% set(gca, "YDir", "reverse");
% colorbar;
% title("Ground elevation around Death Valley and Mt. Whitney");
% ylabel("Longitude (deg)");
% xlabel("Latitude (deg)");

% CHANGELOG
% v1.1.1 - Improved input validation, description and examples added.
% v1.1.0 - Long queries are now separated into shorted batches. The length
%          of a batch can be specified by user.

%% Validate inputs
% Validate the number of input parameters
narginchk(2, 3);
nargoutchk(0, 1);

% Latitudes has to be between -90 and 90
validateattributes( latitude, {'numeric'}, {'2d', 'real', '>=',  -90, '<=',  90}, ...
                              '', 'latitude', 1);

% Longitude has to be between -180 and 180
validateattributes(longitude, {'numeric'}, {'2d', 'real', '>=', -180, '<=', 180}, ...
                              '', 'longitude', 2);

% Check if latitude and longitude have the same number of rows
if size(latitude, 1) ~= size(longitude, 1)
    warning("Inputs latitude and longitude should have the same number of rows." ...
            + newline + "Excess rows will be omitted.");
    latitude = latitude(1:min(size(latitude, 1), size(longitude, 1)), :);
    longitude = longitude(1:min(size(latitude, 1), size(longitude, 1)), :);
end

% Check if latitude and longitude have the same number of columns
if size(latitude, 2) ~= size(longitude, 2)
    warning("Inputs latitude and longitude should have the same number of columns." ...
            + newline + "Excess columns will be omitted.");
    latitude = latitude(:, 1:min(size(latitude, 2), size(longitude, 2)));
    longitude = longitude(:, 1:min(size(latitude, 2), size(longitude, 2)));
end

if nargin == 2
    % Assume that 16384 locations is a safe size of a request for which the
    % server does not return status 413 ("Request Entity Too Large")
    batchsize = 16384;
else
    % batchsize has to be positive integer
    validateattributes(batchsize, {'numeric'}, {'scalar', 'integer', 'positive'}, ...
                                  '', 'batchsize', 3);        
end

%% Run function
% Create query structure
locations = struct([]);
for i = 1:numel(latitude)
    locations(i).latitude = latitude(i);
    locations(i).longitude = longitude(i);
end

% Define the API endpoint URL
url = "https://api.open-elevation.com/api/v1/lookup";

% Specify the options for the request
options = weboptions(ContentType = "json", ...
                     MediaType = "application/json", ...
                     RequestMethod = "post");

try
    % Compute the number of queries
    nqueries = ceil(length(locations) / batchsize);

    % Preallocate elevations
    elevation = zeros(size(latitude));

    % Prepare query strings and send them to the server
    for i = 1:nqueries
        inds  = batchsize*(i-1)+1:min(batchsize*i,length(locations));
        query = jsonencode(struct(locations = locations(inds)));

        % Catch a syntax exception if the batch contains only one location
        if isscalar(inds)
            query = [query(1:13) '[' query(14:end-1) ']' query(end)];
        end

        % Try to obtain a response from the server
        response = webwrite(url, query, options);

        % Parse response
        if isscalar(inds)
            elevation(inds) = response.results.elevation;
        else
            for j = 1:length(inds)
                elevation(inds(j)) = response.results{j}.elevation;
            end
        end

        % Wait to avoid server status 429 ("Too Many Requests")
        pause(0.1);
    end

catch response
    % Catch errors
    error(getReport(response));
end

% End of function
end