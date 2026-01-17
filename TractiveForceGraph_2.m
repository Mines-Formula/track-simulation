disp("Initializing Vehicle Parameters...")
vehicleParams


disp("Loading data...")
Tr  = readtable("9.30RearData.csv");
Tf = readtable("9.30FrontData.csv");
disp("Data Loaded")



 
%%%---Convert sensor names to categorical for fast filtering---%%%
sr = categorical(Tr.Sensor);
sf = categorical(Tf.Sensor);
vr = Tr.Value;
vf = Tf.Value;

%%%---Extract indices once---%%%
timeIdxr     = find(sr == "Time");
timeIdxf     = find(sf == "Time");
xAccelIdxf   = find(sf == "DAQXAccel");
gearIdxr     = find(sr == "Gear");
rpmIdxr      = find(sr == "EngineSpeed");
latIdxr      = find(sr == "Latitude");
lonIdxr      = find(sr == "Longitude");
throttleIdxr = find(sr == "ThrottlePedal");


%%%---Build per-signal (time, value) arrays (time in seconds)---%%%

% master time axis REAR
timeValsr = Tr{timeIdxr,1} ./ 1000;   % ms → s
timeValsr = timeValsr(:);

% master time axis FRONT
timeValsf = Tf{timeIdxf,1} ./ 1000;   % ms → s
timeValsf = timeValsf(:) + .015; % one avergae, front day DAQ is 150 ms ahead

% xAccel (analog) 
xAccelTimer = Tf{xAccelIdxf,1} ./ 1000;   % ms → s
xAccelValsf = vf(xAccelIdxf);        xAccelValsf = xAccelValsf(:);

% gear (discrete)
gearTimer = Tr{gearIdxr,1} ./ 1000;  gearTimer = gearTimer(:);
gearValsr = vr(gearIdxr);            gearValsr = gearValsr(:) + 1; % 0–5 → 1–6

% rpm (analog)
rpmTimer  = Tr{rpmIdxr,1} ./ 1000;   rpmTimer = rpmTimer(:);
rpmValsr  = vr(rpmIdxr);             rpmValsr = rpmValsr(:);

% latitude (analog)
latTimer  = Tr{latIdxr,1} ./ 1000;   latTimer = latTimer(:);
latValsr  = vr(latIdxr);             latValsr = latValsr(:);

% longitude (analog)
lonTimer  = Tr{lonIdxr,1} ./ 1000;   lonTimer = lonTimer(:);
lonValsr  = vr(lonIdxr);             lonValsr = lonValsr(:);

% throttle (analog)
thrTimer  = Tr{throttleIdxr,1} ./ 1000; thrTimer = thrTimer(:);
thrValsr  = vr(throttleIdxr);           thrValsr = thrValsr(:);

%%%---Make the master time strictly unique & increasing---%%%
[timeValsr, ~] = unique_pair(timeValsr, timeValsr, "first");
if numel(timeValsr) < 2
    error("Master Time stream (rear) has < 2 samples after cleanup; cannot compute speeds.");
end

[timeValsf, ~] = unique_pair(timeValsf, timeValsf, "first");
if numel(timeValsr) < 2
    error("Master Time stream (front) has < 2 samples after cleanup; cannot compute speeds.");
end

%%%---Interpolate all signals to the master time grid (safe)---%%%
% Discrete signal: nearest
gearInterp     = safe_interp1(gearTimer, gearValsr, timeValsr, 'nearest', 'extrap', true);
% Analogs: linear
rpmInterp      = safe_interp1(rpmTimer,  rpmValsr,  timeValsr, 'linear',  'extrap', false);
latInterp      = safe_interp1(latTimer,  latValsr,  timeValsr, 'linear',  'extrap', false);
lonInterp      = safe_interp1(lonTimer,  lonValsr,  timeValsr, 'linear',  'extrap', false);
throttleInterp = safe_interp1(thrTimer,  thrValsr,  timeValsr, 'linear',  'extrap', false);
xAccelInterp   = safe_interp1(xAccelTimer, xAccelValsf, timeValsr+0.015, 'linear', 'extrap', false);

%%%---Compute speeds from GPS---%%%
R = 6371000;      % Earth radius (m)
degToRad = pi/180;

dLat   = diff(latInterp);
dLon   = diff(lonInterp);
avgLat = (latInterp(1:end-1) + latInterp(2:end))/2 * degToRad;

distances = R * sqrt( (dLat*degToRad).^2 + (cos(avgLat).*(dLon*degToRad)).^2 );
deltaT    = diff(timeValsr);

% Guard against zero/negative/NaN dt
bad_dt = ~isfinite(deltaT) | deltaT <= 0;
deltaT(bad_dt) = NaN;

speeds = distances ./ deltaT;

% Align other streams (drop last sample to match diff length)
rpmInterp      = rpmInterp(1:end-1);
gearInterp     = gearInterp(1:end-1);
throttleInterp = throttleInterp(1:end-1);
xAccelInterp   = xAccelInterp(1:end-1);



%%%---Validity mask---%%%
validMask = isfinite(speeds) & speeds > 0 & speeds < 150 & throttleInterp == 1;
% Optional throttle filter:
validIdx  = find(validMask);

% validIdx already computed above
idx = validIdx(:);                       % ensure column index

% Recompute per-idx signals and force column orientation
         

gearIdxSafe = round(gearInterp(idx));
gearIdxSafe = max(1, min(numel(params.gear_ratios_complete), gearIdxSafe));
gearRatios = params.gear_ratios_complete(gearIdxSafe);
gearRatios = gearRatios(:);              % N×1

% Ensure these are scalars (or index them per-gear if you actually store vectors)
assert(isscalar(params.powertrain_efficiency), ...
    'powertrain_efficiency must be a scalar; if per-gear, index it like gearRatios.');
assert(isscalar(params.wheel_rad), ...
    'wheel_rad must be a scalar; if per-axle/tire, index it to N×1 first.');

% Now this is elementwise N×1 .* N×1 → N×1
%driveForce = (torque .* gearRatios) .* params.powertrain_efficiency ./ params.wheel_rad;
Accel  = xAccelInterp(idx);  
driveForce = Accel .* params.mass;
driveForce = driveForce(:);

figure
plot(xAccelValsf)


% Assemble table (keep shapes aligned)
Speed  = speeds(idx);         Speed  = Speed(:);
Gear   = gearInterp(idx);     Gear   = Gear(:);

saveTable = table(Speed, driveForce, Gear, ...
    'VariableNames', {'Speed','DriveForce','Gear'});

disp("Writing processed data...")
writetable(saveTable, "Speed_driveForce_Plot.csv");

%%%---Plot robustly (auto-detect columns)---%%%
disp("Plotting results...")
Tr = readtable("Speed_driveForce_Plot.csv");

if isempty(Tr) || height(Tr) == 0
    warning("No valid rows to plot");
else
    vars = lower(string(Tr.Properties.VariableNames));
    speedCol = find(contains(vars,"speed"),1);
    forceCol = find(contains(vars,["driveforce","force"]),1);
    gearCol  = find(contains(vars,"gear"),1);

    Speed      = Tr.(Tr.Properties.VariableNames{speedCol});
    DriveForce = Tr.(Tr.Properties.VariableNames{forceCol});
    if isempty(gearCol)
        Gear = ones(height(Tr),1);
    else
        Gear = Tr.(Tr.Properties.VariableNames{gearCol});
    end

    % Ensure numeric gear
    if ~isnumeric(Gear), Gear = double(string(Gear)); end
    Gear(isnan(Gear)) = 1;

    uniqueGears = unique(round(Gear(~isnan(Gear))));
    if isempty(uniqueGears), uniqueGears = 1; end
    nGears = numel(uniqueGears);

    figure
    scatter(Speed, DriveForce, 10, Gear, 'filled');
    cmap = turbo(nGears);                
    colormap(cmap);
    colorbar('Ticks', uniqueGears, 'TickLabels', compose('Gear %d', uniqueGears));

    xlabel('Speed [m/s]')
    ylabel('Drive Force [N]')
    title('Drive Force vs Speed (colored by gear)')
    grid on
end

hold on

% Define per-gear [a, b (turns out I didnt need it), c, d, v_min, v_max]
manual = [
    -6.72,   71.85, 245.13,  0,  10;    % Gear 1
    -4.65,   104.8, -273.24, 1,  16;    % Gear 2
    -2.08,   63.83, -225.44, 8,  22;    % Gear 3
    -1.18,   43.69, -175,    6,  25;    % Gear 4
    -0.8,    34.21, -160,    9,  25.5;  % Gear 5
    -0.0751, 4.59,  129,     21, 45;    % Gear 6
];

for row = 1:size(manual,1)
    a = manual(row,1);
    c = manual(row,2);
    d = manual(row,3);
    vmin = manual(row,4);
    vmax = manual(row,5);

    vr = linspace(vmin, vmax, 150);
    fr = a*vr.^2 + c*vr + d;

    %make curve color match scatter color for this gear index

    % Thick black outline
    plot(vr, fr, 'LineWidth', 3, 'Color', 'k');

    % Colored line on top
    plot(vr, fr, 'LineWidth', 2, 'Color', cmap(row,:));
end



disp("Processing complete.")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper: make (t,v) unique on t; mode = "first" (keep first) or "mean".
% - Removes non-finite entries.
% - Returns column vectors tu, vu with matching lengths.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [tu, vu] = unique_pair(t, v, mode)
    t = t(:); v = v(:);
    mask = isfinite(t) & isfinite(v);
    t = t(mask); v = v(mask);

    switch lower(mode)
        case "first"
            [tu, ia] = unique(t, 'stable'); % indices of first occurrences
            vu = v(ia);

        case "mean"
            % Group by identical t (stable), compute mean v per group
            [tu, ~, ic] = unique(t, 'stable');  % ic maps each input to group 1..numel(tu)
            sumv  = accumarray(ic, v, [numel(tu), 1], @sum, 0);
            count = accumarray(ic, 1, [numel(tu), 1], @sum, 0);
            vu = sumv ./ max(count, 1);

        otherwise
            error("unique_pair: unknown mode '%s'", mode);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper: safe interpolation with degenerate cases handled
% - If ≥2 points: interp1
% - If exactly 1 point: constant fill
% - If 0 points: NaNs
% - discrete=true uses nearest and rounds output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function yq = safe_interp1(t, x, tq, method, extrapMode, discrete)
    if nargin < 6, discrete = false; end
    t = t(:); x = x(:); tq = tq(:);

    % Remove non-finite pairs
    m = isfinite(t) & isfinite(x);
    t = t(m); x = x(m);

    % Deduplicate by time, keep-first for discrete, mean for analog
    mode = "mean"; if discrete, mode = "first"; end
    [tu, xu] = unique_pair(t, x, mode);

    n = numel(tu);
    if n >= 2
        yq = interp1(tu, xu, tq, method, extrapMode);
    elseif n == 1
        yq = repmat(xu, size(tq)); % constant signal
    else
        yq = NaN(size(tq));
    end

    if discrete
        yq = round(yq);
    end
end
