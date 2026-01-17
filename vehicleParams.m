%{
Used in most if not all of the following programs. It is ran at the start of a program, 
and creates a struct of data relative to the car. Makes it easy for car data to change across multiple programs.
%}

params.mass  = 208;   % kg (from (I believe) 460 lbs)
params.mu    = 1.8;   % grip 
params.a_max = 11;     % m/s^2
params.b_max = 16;    % m/s^2
params.g     = 9.81;  % gravity, m/2^2
params.wheel_rad = 0.2032; % radius of the wheel (hoosier 43070, 16" diameter - nominal) converted to m; 8in = 0.2032m
params.Crr = 0.015; %the rolling resistance of the wheel base -- REALISTIC ESTIMATION, NOT ACTUAL NUMBER

pg = 76/36; %primary ratio -- before transmission
fg = 11/35; %final ratio -- leaving transmission

params.gear_ratios = [2.75, 2, 1.666, 1.444, 1.304, 1.208]; %raw ratios of just each gear individually

params.gear_ratios_complete = params.gear_ratios*pg*fg;%ratios of entire gear train, per gear

params.powertrain_efficiency = 0.92;

params.redline = 13000;    % rpm (from RPM range)
params.rpm_min = 5000;     % rpm (usable lower bound for good torque)
params.shift_time = 0.10; %time for vehicle to coast while shifting
params.down_force_iterations = 3; %iterations for inclusiong of downforce




