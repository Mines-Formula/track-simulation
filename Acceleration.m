%{
set of equations to add into the lapsim once a stronger data sat is provided
%}

vehicleParams

T = readtable("torque_data.csv");%read table to get data %% BOGUS DATA, DO NOT USE FOR ANYTHING REAL

rpm = T.RPM; %pull rpm data
torque = T.Torque_ft_lbf;%pull torque data

rpm_smooth = linspace(min(rpm), max(rpm), 100); % generate smooth RPM values for interpolation


pTorque = polyfit(rpm, torque, 3);%fit a polynomial to the rpm/torque relation

function torque = torque_at_rpm(rpm) % get the torque produced by the engine for a certain rpm
torque = polyval(pTorque, rpm);
end



%%%---TRACTIVE FORCES---%%%
function rpm = rpm_from_speed(vel, gear_num)% pull rpm data from the speed of the car
rpm = (vel ./ params.wheel_rad) .* params.gear_ratios_complete(gear_num) .* (60 / (2*pi));
end
function tractive_force_eng = tractive_force_eng(rpm)% pull the tractive forces of the engine
    tractive_force_eng = torque(rpm);
end
function tractive_force_wheel = tractive_force_wheel(rpm, gearNum)% pull the tractive force of the wheel
    tractive_force_wheel = params.powertrain_efficiency.*tractive_force_eng(rpm).*params.gear_ratios_complete(gearNum);
end
function drive_force = drive_force(rpm, gearNum)%total drive force
    drive_force = (tractive_force_wheel(rpm, gearNum)/params.wheel_rad);
end



%%%---RESISTANCE---%%%
function aero_drag = aero_drag(v)%drag from air, proportional to v^2
%defined as Fd = 0.5 * air_density * drag_area * v^2
%the decimal below is pulled from a quadratic regression
aero_drag = (0.00855 * (v.^2));
end

function rolling_drag = rolling_drag()%drag from friction with the ground
%defined as Frr = Crr (rolling resistance) * m * g
%No tests are public, so for now, Crr is an estimation
rolling_drag = (params.Crr.*params.m.*params.g);
end

function gradient_drag = gradient_drag()
%set the angle for gradient
theta = 0; %default 0
gradient_drag = sin(theta).*params.m.*params.g;
end

function resistance = resistance(vel)% combined method for gathering all resistance forces, such as air drag, rolling drag etc
    resistance = aero_drag(vel) + rolling_drag() + gradient_drag();
end



%%%---DOWNFORCE and FRICTION ELLIPSE/CRICLE---%%%
function downforce = downforce(vel)
%defined as 0.5 * air_density * reference_area * coefficient_of_lift * vel^2
    downforce = (0.01677 * vel.^2);%0.01677 is pulled from quadratic regression from data points, constants otherwise unknown
end

function accel_y = accel_y(vel, curvature)
accel_y = (curvature * vel.^2);
end

function g_eff = g_eff(v)
    g_eff = (downforce(v) + params.g);
end

function accel_x_tire_cap = accel_x_tire_cap(vel, curvature)
accel_x_tire_cap = sqrt( max((params.mu.*g_eff(vel)).^2 - (accel_y(vel, curvature)).^2, 0));%implamentation of friction circle, so you don't "double dip" on fiction
end



%%%---POWERTRAIN MAX ACCELERATION---%%%
function powertrain_max_accel = powertrain_max_accel(vel, rpm, gearNum)
%defined as (drive_force-resistance)/equivelent_mass. 
%Since equivelent mass is hardly ever known, it is approximated with 1.06
   powertrain_max_accel = ((drive_force(rpm, gearNum))-(resistance(vel)))/(1.06.*params.mass);
end


function forward_accel_used_in_march = forward_accel_used_in_march(vel, curvature, gearNum, rpm)
%the capped acceleration function that can be used in teh sim's lap march
   forward_accel_used_in_march = min((powertrain_max_accel(vel, rpm, gearNum)),(accel_x_tire_cap(vel, curvature)));
end


