% function description
% time_gps  |time

%   GPS No.1
% Lat1      |latitude of GPS No.1
% Lng1      |longitude of GPS No.1
% Alt1      |Altitude of GPS No.1
% HDop1     |HDop of GPS No.1
% NSats1    |Number of Satellite of GPS No.1

%   GPS No.2
% Lat2
% Lng2
% Alt2
% HDop2
% NSats2

%   IMU data.
% gyroX_resampled
% gyroY_resampled
% gyroZ_resampled
% accX_resampled
% accY_resampled
% accZ_resampled

%   AHRS data.
% yaw_resampled

%   Barometer data.
% baro_resampled


function [time_gps, Lat1, Lng1, Alt1, HDop1, NSats1, Lat2, Lng2, Alt2, HDop2, NSats2, GyroX_resampled, GyroY_resampled, GyroZ_resampled, accX_resampled, accY_resampled, accZ_resampled, yaw_resampled, baro_resampled] = get_data(binary_log_file)
    
    bin = ardupilotreader(binary_log_file);
    
    % GPS and IMU
    msg = readMessages(bin);
    GpsMsg = readMessages(bin, 'MessageName', {'GPS'});
    ImuMsg = readMessages(bin, 'MessageName', {'IMU'});
    AHRMsg = readMessages(bin, 'MessageName', {'AHR2'});
    BaroMsg = readMessages(bin, 'MessageName', {'BARO'});

    Gps1Data = GpsMsg.MsgData{1,1};
    Gps2Data = GpsMsg.MsgData{2,1};
    ImuData = ImuMsg.MsgData{1,1};
    AhrsData = AHRMsg.MsgData{1,1};
    BaroData = BaroMsg.MsgData{1,1};

    IMU = ImuData(:,{'GyrX','GyrY','GyrZ','AccX', 'AccY', 'AccZ'});
    positions1 = Gps1Data(:, {'NSats','HDop','Lat','Lng','Alt'});
    positions2 = Gps2Data(:, {'NSats','HDop','Lat','Lng','Alt'});
    Yaw = AhrsData(:,{'Yaw'});
    Baro = BaroData(:,{'Alt'});

    yaw = Yaw.Yaw;
    baro = Baro.Alt;

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
    
    gyroX = IMU.GyrX;
    gyroY = IMU.GyrY;
    gyroZ = IMU.GyrZ;
    accX = IMU.AccX;
    accY = IMU.AccY;
    accZ = IMU.AccZ;
    
    start_time = msg{5,3};
    end_time = msg{5,4};
    n_points = length(Lat1);
    time_gps = linspace(start_time, end_time, n_points);
    time_imu = linspace(start_time, end_time, length(accX));
    
    % Low-pass filter 고려...
    Fs = 100;
    Fc = 5;
    
    % 
    % accX_filtered = lowpass(accX, Fc, Fs);
    % accY_filtered = lowpass(accY, Fc, Fs);
    % accZ_filtered = lowpass(accZ, Fc, Fs);
    % 
    GyroX_resampled = interp1(time_imu, gyroX, time_gps, 'linear');
    GyroY_resampled = interp1(time_imu, gyroY, time_gps, 'linear');
    GyroZ_resampled = interp1(time_imu, gyroZ, time_gps, 'linear');

    accX_resampled = interp1(time_imu, accX, time_gps, 'linear');
    accY_resampled = interp1(time_imu, accY, time_gps, 'linear');
    accZ_resampled = interp1(time_imu, accZ, time_gps, 'linear');

    yaw_resampled = interp1(linspace(start_time, end_time, length(yaw)), yaw, time_gps, 'linear');
    baro_resampled = interp1(linspace(start_time, end_time, length(baro)), baro, time_gps, 'linear');
end
