function PlotCSVGps(csvFileName)
    % PlotCSVGps — read csv file, stitch them together,
    % then smooth and build one continuous spline in the X–Z plane, and plot.
    %
    % Usage: PlotCSVGps('FSAE_gps_comp.csv')

    % ---------- Read CSV ----------
    try
        lines = readtable(csvFileName);
    catch ME
        error('Cannot open CSV file: %s', csvFileName);
    end

    latitude = lines{:, 2};
    longitude = lines{:, 3};
    
    % ---------- Plot ----------
    geoplot(latitude, longitude, '-r', 'linewidth', 2)

    % ---------- Convert to Feet ----------
    lat0 = mean(latitude);
    lon0 = mean(longitude);

    R = 6371000; % Earth radius in meters
    metersToFeet = 3.28084;

    X = R * cosd(lat0) .* deg2rad(longitude - lon0) * metersToFeet;
    Z = R * deg2rad(latitude - lat0) * metersToFeet;
    
    % ---------- Smooth GPS data ----------
    X_smooth = smoothdata(X, 'movmedian', 7);
    Z_smooth = smoothdata(Z, 'movmedian', 7);

    % ---------- Output ----------
    gps_points = [X_smooth, Z_smooth]';
    gps_points = removeDuplicates(gps_points, 0.05);

    trackSplineXZ = cscvn(gps_points);
    
    save('TrackSpline.mat', 'trackSplineXZ');
end
    
function Q = removeDuplicates(P, tol)
    keep = true(1, size(P,2));
    for j = 2:size(P,2)
        if norm(P(:,j) - P(:,j-1)) < tol
            keep(j) = false;
        end
    end
    Q = P(:, keep);
end


%PlotCSVGps('gps_comp.csv')