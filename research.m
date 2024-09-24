hdop = [];
lat = [];
lng = [];
alt = [];

binary_log_file = "Research/fail.bin";

bin = ardupilotreader(binary_log_file);

msg = readMessages(bin);

% Read data between 2min 34sec and 20min.
% Test 1번 자료 비행 = 2분부터 기록되어있음...
d1 = duration([0 0 00],'Format','hh:mm:ss.SSSSSS');
d2 = d1 + duration([0 20 00],'Format','hh:mm:ss.SSSSSS');

GpsMsg = readMessages(bin,'MessageName',{'GPS'},'Time',[d1 d2]);
GpsData = GpsMsg.MsgData{1,1}

% GpsData 
positions = GpsData(:, {'HDop','Lat','Lng','Alt'});
hdop = GpsData.HDop;
Lat = positions.Lat;
Lng = positions.Lng;
Alt = positions.Alt;

% 
% HDop로 계산하기...
% 
hdop_normalized = (hdop - min(hdop)) / (max(hdop) - min(hdop));
cmap = [hdop_normalized, 1 - hdop_normalized, zeros(length(hdop_normalized), 1)];

figure;
% plot3(Lng, Lat, Alt, 'o-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
hold on;
grid on;
for i = 1:length(Lng)-1
    plot3(Lng(i:i+1), Lat(i:i+1), Alt(i:i+1), 'Color', cmap(i,:), 'LineWidth', 1.5);
end

scatter3(Lng(1), Lat(1), Alt(1), 100, 'g', 'filled');
scatter3(Lng(end), Lat(end), Alt(end), 100, 'r', 'x', 'LineWidth', 2);

xlabel('Longitude');
ylabel('Latitude');
zlabel('Altitude');
title('GPS Position with HDop');

view(45, 30);