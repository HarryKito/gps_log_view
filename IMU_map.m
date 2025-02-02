% TODO
%   Get {Gyro,Acc,+maybe Compass} datas from IMU.

binary_log_file = "gps_log_view/Research/fail.bin";

[time, Lat1, Lng1, Alt1, HDop1, NSats1, Lat2, Lng2, Alt2, HDop2, NSats2, GyroX_resampled, GyroY_resampled, GyroZ_resampled, accX_resampled, accY_resampled, accZ_resampled, yaw_resampled, baro_resampled] = get_data(binary_log_file);


figure;
% Gyro
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
plot(time, HDop1, 'g', 'DisplayName', 'HDop1');
plot(time, HDop2, 'g', 'DisplayName', 'HDop2');
title('HDops');
legend;
grid on;

% Baro
subplot(5,1,5);
plot(time, baro_resampled, 'm', 'DisplayName', 'Barometer');
title('Barometer');
ylabel('Alt');
legend;
grid on;
