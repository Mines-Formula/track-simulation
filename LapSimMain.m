%{
This is the main lapsim. It iterates over spliced splines, 
and uses the cars acceleration, grip, braking power, and track curvature to calculate the fastest speed possible around the track.
%}

function lapTime = LapSimMain()

    % Load saved track spline
    load('TrackSpline.mat','trackSplineXZ');

    % Load parameters (assuming you made a params.m script)
    vehicleParams; %defines params struct


    % Step A: arc length sampling
    %generates each point on the track
    tt = linspace(trackSplineXZ.breaks(1), trackSplineXZ.breaks(end), 100000);
    pts = fnval(trackSplineXZ, tt);
    %velocity vector
    d1 = fnval(fnder(trackSplineXZ,1), tt);
    %curvature
    d2 = fnval(fnder(trackSplineXZ,2), tt);
    
    %rate of how arc length changes
    speed = sqrt(sum(d1.^2,1));
    %distance between each point
    ds = speed(1:end-1) .* diff(tt);
    %distance along the track
    s = [0, cumsum(ds)];

    % Step B: curvature κ
    num = abs(d1(1,:).*d2(2,:) - d1(2,:).*d2(1,:));
    denom = (d1(1,:).^2 + d1(2,:).^2).^(3/2);
    kappa = num ./ denom;

    %speeed limit based on lateral friction
    v_curve = sqrt(params.mu*(params.g) ./ max(kappa,1e-6));


    % Step C: forward accel
    v = zeros(size(s));
    for i = 2:length(s)
        
        v(i) = sqrt(v(i-1)^2 + 2*params.a_max*ds(i-1));
        v(i) = min(v(i), v_curve(i));
    end

    % Step D: backward brake - ensure it can brake in time for the next
    % segment
    for i = length(s)-1:-1:1
        v(i) = min(v(i), sqrt(v(i+1)^2 + 2*params.b_max*ds(i)));
    end

    % Step E: lap time
    dt = ds ./ v(2:end);
    lapTime = sum(dt);

    fprintf('Lap time = %.2f seconds\n', lapTime);

    % calculate brake bias
    brakeBias = calculateBrakeBias(v, dt);

    % Plot
    figure; hold on; axis equal;
    plot(pts(1,:), pts(2,:), 'k-');          % track
    scatter(pts(1,:), pts(2,:), 5, v, 'filled'); % color by speed
    colorbar; title('Velocity along track');

    figure; hold on; axis equal;
    plot(pts(1,:), pts(2,:), 'k-');          % track
    scatter(pts(1,:), pts(2,:), 5, brakeBias, 'filled'); % color by brakebias
    colorbar; title('Brake bias along track');
end

function downforce = getDownForce(v)
vehicleParams;
%disp("SPEED IS " + v)
downforce = 0.01677*(v.^2);
fprintf("%.2f\n", v)
end
