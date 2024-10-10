% TODO
%   Get {Gyro,Acc,+maybe Compass} datas from IMU.

binary_log_file = "Research/fail.bin";

[time, Lat1, Lng1, Alt1, HDop1, NSats1, Lat2, Lng2, Alt2, HDop2, NSats2, GyroX_resampled, GyroY_resampled, GyroZ_resampled, accX_resampled, accY_resampled, accZ_resampled, yaw_resampled, baro_resampled] = get_data(binary_log_file);

figure;
% Gyro
subplot(5,1,1);
plot(time, GyroX_resampled, 'r', 'DisplayName', 'X'); hold on;
plot(time, GyroY_resampled, 'g', 'DisplayName', 'Y');
plot(time, GyroZ_resampled, 'b', 'DisplayName', 'Z');
title('Gyro');
ylabel('deg/s');
legend;
grid on;

% Acc
subplot(5,1,2);
plot(time, accX_resampled, 'r', 'DisplayName', 'X'); hold on;
plot(time, accY_resampled, 'g', 'DisplayName', 'Y');
plot(time, accZ_resampled, 'b', 'DisplayName', 'Z');
title('Accelerometer');
ylabel('(m/s²)');
legend;
grid on;

% Yaw
subplot(5,1,3);
plot(time, yaw_resampled, 'k', 'DisplayName', 'Yaw');
title('Yaw');
ylabel('deg');
legend;
grid on;

% HDop
subplot(5,1,4);
plot(time, HDop1, 'r', 'DisplayName', 'HDop1');hold on;
plot(time, HDop2, 'g', 'DisplayName', 'HDop2');
title('HDop');
legend;
grid on;

% Baro
subplot(5,1,5);
plot(time, baro_resampled, 'm', 'DisplayName', 'Barometer');
title('Barometer');
ylabel('Alt');
legend;
grid on;

%{

% Function to compute the rotation matrix from yaw, pitch, roll
function R = compute_rotation_matrix(yaw, pitch, roll)
    % Rotation matrix for yaw (rotation around Z-axis)
    R_yaw = [cos(yaw), -sin(yaw), 0;
             sin(yaw), cos(yaw), 0;
             0, 0, 1];
    
    % Rotation matrix for pitch (rotation around Y-axis)
    R_pitch = [cos(pitch), 0, sin(pitch);
               0, 1, 0;
               -sin(pitch), 0, cos(pitch)];
    
    % Rotation matrix for roll (rotation around X-axis)
    R_roll = [1, 0, 0;
              0, cos(roll), -sin(roll);
              0, sin(roll), cos(roll)];
    
    % Final rotation matrix (R = R_yaw * R_pitch * R_roll)
    R = R_yaw * R_pitch * R_roll;
end

% Main script
% Constants
g = 9.81;  % Gravity (m/s^2)
dt = mean(seconds(diff(time)));  % Time step in seconds

% Initialize velocity and position arrays
velX = zeros(size(accX_resampled));
velY = zeros(size(accY_resampled));
velZ = zeros(size(accZ_resampled));
posX = zeros(size(accX_resampled));
posY = zeros(size(accY_resampled));
posZ = zeros(size(accZ_resampled));

% Initialize orientation (assuming initial orientation is level)
roll = zeros(size(time));
pitch = zeros(size(time));
yaw = yaw_resampled;  % Assuming yaw_resampled is in radians

% Integrate gyroscope data to get roll and pitch (assuming small angles)
for i = 2:length(time)
    roll(i) = roll(i-1) + GyroX_resampled(i) * dt;
    pitch(i) = pitch(i-1) + GyroY_resampled(i) * dt;
end

% Correct accelerometer data for gravity and convert to world frame
for i = 1:length(time)
    % Compute rotation matrix from body frame to world frame
    R = compute_rotation_matrix(yaw(i), pitch(i), roll(i));
    
    % Body-frame accelerometer values
    acc_body = [accX_resampled(i); accY_resampled(i); accZ_resampled(i) - g];
    
    % Convert to world frame
    acc_world = R * acc_body;
    
    % Integrate accelerations to get velocity
    if i > 1
        velX(i) = velX(i-1) + acc_world(1) * dt;
        velY(i) = velY(i-1) + acc_world(2) * dt;
        velZ(i) = velZ(i-1) + acc_world(3) * dt;
    end
end

% Integrate velocity to get position
for i = 2:length(time)
    posX(i) = posX(i-1) + velX(i) * dt;
    posY(i) = posY(i-1) + velY(i) * dt;
    posZ(i) = posZ(i-1) + velZ(i) * dt;
end

% Plot 3D flight path from IMU data
figure;
plot3(posX, posY, posZ, 'r-', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Estimated Flight Path from IMU Data (Corrected with Orientation)');
grid on;

%}