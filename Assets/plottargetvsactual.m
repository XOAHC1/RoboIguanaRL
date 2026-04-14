%% Read CSV file
data = readtable('joint_angle_log.csv');

% Extract columns
time = data.Time;
target = data.TargetAngle;
actual = data.ActualAngle;

%% Plot target vs actual
figure;
plot(time, target, 'LineWidth', 1.5);
hold on;
plot(time, actual, 'LineWidth', 1.5);
xlim([20 30]);

xlabel('Time (s)');
ylabel('Angle (deg)');
title('Joint Target vs Actual Angle');

legend('Target Angle','Actual Angle');
grid on;