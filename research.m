% Users...
Fs = 100;  % Assumed IMU sample rate (Hz)
Fc = 5;    % Cutoff frequency for low-pass filter (Hz)

binary_log_file = "Research/fail.bin";
bin = ardupilotreader(binary_log_file);
 % ----------------------------------- %
% IMU
gyroX = [];
gyroY = [];
gyroZ = [];
accX = [];
accY = [];
accZ = [];

% GPS1
lat1 = [];
lng1 = [];
alt1 = [];
HDop1 = [];
NSats1 = [];

% GPS2
lat2 = [];
lng2 = [];
alt2 = [];
HDop2 = [];
NSats2 = [];

msg = readMessages(bin);
GpsMsg = readMessages(bin,'MessageName',{'GPS'});
ImuMsg = readMessages(bin,'MessageName',{'IMU'});
AHRMsg = readMessages(bin, 'MessageName', {'AHR2'});

Gps1Data = GpsMsg.MsgData{1,1};
Gps2Data = GpsMsg.MsgData{2,1};
ImuData = ImuMsg.MsgData{1,1};
AhrsData = AHRMsg.MsgData{1,1};
% GpsData
% NSats = number of satellites available
% HDop  = Horizonal Dilution of Precision
% Lat   = Latitude
% Lng   = Longitude
% Alt   = Altitude (Based on GPS, not barometer)
IMU_acc = ImuData(:,{'GyrX','GyrY','GyrZ','AccX','AccY','AccZ'});
positions1 = Gps1Data(:, {'NSats','HDop','Lat','Lng','Alt'});
positions2 = Gps2Data(:, {'NSats','HDop','Lat','Lng','Alt'});
Yaw = AhrsData(:,{'Yaw'});

yaw = Yaw.Yaw;

Lat1 = positions1.Lat;
Lng1 = positions1.Lng;
Alt1 = positions1.Alt;
HDop1 = positions1.HDop;
NSats1 = positions1.NSats;

Lat2 = positions2.Lat;
Lng2 = positions2.Lng;
Alt2 = positions2.Alt;
HDop2 = positions2.HDop;
NSats2 = positions2.NSats;

accX = IMU_acc.AccX;
accY = IMU_acc.AccY;
accZ = IMU_acc.AccZ;

start_time = msg{5,3};
end_time = msg{5,4};
n_points = length(Lat1);
time = linspace(start_time, end_time, n_points);

time_gps = linspace(start_time, end_time, n_points);
time_imu = linspace(start_time, end_time, length(accX));

% lowpass filters
accX_filtered = lowpass(accX, Fc, Fs);
accY_filtered = lowpass(accY, Fc, Fs);
accZ_filtered = lowpass(accZ, Fc, Fs);
% resample to match GPS
accX_resampled = interp1(time_imu, accX_filtered, time_gps, 'linear');
accY_resampled = interp1(time_imu, accY_filtered, time_gps, 'linear');
accZ_resampled = interp1(time_imu, accZ_filtered, time_gps, 'linear');

yaw_resampled = interp1(linspace(start_time, end_time, length(yaw)), yaw, time_gps, 'linear');

% 1. Latitude
figure;
subplot(9,1,1);
plot(time, Lat1, 'DisplayName', 'GPS1 Lat');
hold on;
plot(time, Lat2, 'DisplayName', 'GPS2 Lat');
xlabel('Time (s)');
ylabel('Latitude');
title('Lat');
legend;

% 2. Longitude
subplot(9,1,2);
plot(time, Lng1, 'DisplayName', 'GPS1 Lng');
hold on;
plot(time, Lng2, 'DisplayName', 'GPS2 Lng');
xlabel('Time (s)');
ylabel('Longitude');
title('Lng');
legend;

% 3. Altitude
subplot(9,1,3);
plot(time, Alt1, '-o', 'DisplayName', 'GPS1 Alt');
hold on;
plot(time, Alt2, '-x', 'DisplayName', 'GPS2 Alt');
xlabel('Time (s)');
ylabel('Altitude');
title('Alt');
legend;

% 4. HDop
subplot(9,1,4);
plot(time, HDop1, 'DisplayName', 'GPS1 HDop');
hold on;
plot(time, HDop2, 'DisplayName', 'GPS2 HDop');
xlabel('Time (s)');
ylabel('HDop');
title('HDop');
legend;

% 5. Number of satellites
subplot(9,1,5);
plot(time, NSats1, '-o', 'DisplayName', 'GPS1 NSats');
hold on;
plot(time, NSats2, '-x', 'DisplayName', 'GPS2 NSats');
xlabel('Time (s)');
ylabel('NSats');
title('NSats');
legend;

% 6. Acceleration X-axis
subplot(9,1,6);
plot(time_gps, accX_resampled, 'DisplayName', 'IMU AccX');
xlabel('Time (s)');
ylabel('AccX (m/s^2)');
title('Acc X');
legend;

% 7. Acceleration Y-axis
subplot(9,1,7);
plot(time_gps, accY_resampled, 'DisplayName', 'IMU AccY');
xlabel('Time (s)');
ylabel('AccY (m/s^2)');
title('Acc Y');
legend;

% 8. Acceleration Z-axis
subplot(9,1,8);
plot(time_gps, accZ_resampled, 'DisplayName', 'IMU AccZ');
xlabel('Time (s)');
ylabel('AccZ (m/s^2)');
title('Acc Z');
legend;

subplot(9,1,9);
plot(time_gps, yaw_resampled, 'DisplayName', 'Yaw');
ylabel('Yaw');
title('Yaw');
legend;