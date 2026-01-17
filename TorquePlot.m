T = readtable("torque_data.csv");

powerTable = table([], [], 'VariableNames', {'RPM','Torque_ft_lbf'});

rpm = T.RPM;  

torque = T.Torque_ft_lbf;
hp = (rpm.*torque/5252);

pTorque = polyfit(rpm, torque, 3);
pHP = polyfit(rpm, hp, 3);


rpm_smooth = linspace(min(rpm), max(rpm), 500);
%torque_smooth = spline(rpm, torque, rpm_smooth);
%hp_smooth = spline(rpm, hp, rpm_smooth);

torque_smooth = polyval(pTorque, rpm_smooth);
hp_smooth = polyval(pHP, rpm_smooth);

figure;
plot(rpm, torque, 'b', 'LineWidth',1, LineStyle='--')
hold on;
plot(rpm, hp, 'g', 'LineWidth',1, LineStyle='--')
hold on;
plot(rpm_smooth, torque_smooth, 'b', 'LineWidth', 2);
hold on;
plot(rpm_smooth, hp_smooth, 'g', 'LineWidth', 2)
hold off;
legend('Torque Raw', 'HorsePower Raw', 'Torque Smoothed', 'HorsePower Smoothed')
grid on;

xlabel('Engine Speed (RPM)');
ylabel('Torque/Horsepower');
title('Engine Torque Curve');
