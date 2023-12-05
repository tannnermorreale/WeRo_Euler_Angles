%% Code to estimate ankle angle from shank and foot mounted IMUs
% Tanner Morreale & Edgar Bolivar - Sep. 2023

clc, clearvars, close all % Prefer a clear the workspace to have repeatable results

Table_s = readtable("Standing_Still_1-000_00B4A3D1.txt");
Roll_s = Table_s.Roll;
n = length(Roll_s);
Pitch_s = Table_s.Pitch;
Yaw_s = Table_s.Yaw;
Acc_X_s = Table_s.Acc_X;
Acc_Y_s = Table_s.Acc_Y;
Acc_Z_s = Table_s.Acc_Z;
Gyro_X_s = Table_s.Gyr_X;
Gyro_Y_s = Table_s.Gyr_Y;
Gyro_Z_s = Table_s.Gyr_Z;
R_s = reshape(table2array(Table_s(:, 23:31)).', [3, 3, n]); % Notice the transpose

Table_f = readtable("Standing_Still_1-000_00B4A3CC.txt");
Roll_f = Table_f.Roll;
Pitch_f = Table_f.Pitch;
Yaw_f = Table_f.Yaw;
Acc_X_f = Table_f.Acc_X;
Acc_Y_f = Table_f.Acc_Y;
Acc_Z_f = Table_f.Acc_Z;
Gyro_X_f = Table_f.Gyr_X;
Gyro_Y_f = Table_f.Gyr_Y;
Gyro_Z_f = Table_f.Gyr_Z;
R_f = reshape(table2array(Table_f(:, 23:31)).', [3, 3, n]);

%% Getting the joint angle from the rotation matrix

phiCal = zeros(n,1);
ankAng = zeros(n,1);
R_f_s  = zeros(3,3,n);

for i = 1:n
    % Calculate roll angle from rotation matrix as a sanity check
    theta = asin(-R_f(3, 1, i));
    phiCal(i) = atan2d(R_f(3,2,i)/cos(theta), R_f(3,3,i)/cos(theta));

    % Create rotation matrix from foot frame to shank frame - Notice the
    % transpose
    R_f_s(:,:,i) = R_f(:,:,i).'*R_s(:,:,i);
    % Calculate Angle from fixed axis - (Subject to change depending on
    % validation experiments)
    ankAng(i) = asind(-R_f_s(3,1,i));
end

% Plot Roll angle as a sanity check of the operation
figure, hold on, grid on, title('Sanity Check')
plot(phiCal);
plot(Roll_f, '--o');
legend('Calculated', 'XSENS')
xlabel('Sample')
ylabel('Roll angle [deg.]')

% Plot ankle angle from absolute rotation around the y-axis
figure, hold on, grid on, title('Estimation of Ankle Angle')
plot(ankAng-ankAng(1), '--o');
xlabel('Sample')
ylabel('Ankle angle [deg.]')

figure, hold on, grid on, title('Pitch')
plot(Pitch_s)

