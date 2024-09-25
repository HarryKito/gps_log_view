% Parameters control by User
hdop_threshold = 1.5;

hdop = [];
lat = [];
lng = [];
alt = [];

binary_log_file = "Research/fail.bin";

bin = ardupilotreader(binary_log_file);

msg = readMessages(bin);

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
% Expect with HDop...
% 
hdop_normalized = (hdop - min(hdop)) / (max(hdop) - min(hdop));
cmap = [hdop_normalized, 1 - hdop_normalized, zeros(length(hdop_normalized), 1)];

figure;
hold on;
grid on;
for i = 1:length(Lng)-1
    plot3(Lng(i:i+1), Lat(i:i+1), Alt(i:i+1), 'Color', cmap(i,:), 'LineWidth', 1.5);
end

% Take off (Green filled circle)
scatter3(Lng(1), Lat(1), Alt(1), 100, 'g', 'filled');
% Land (Red 'X')
scatter3(Lng(end), Lat(end), Alt(end), 100, 'r', 'x', 'LineWidth', 2);

for i = 1:length(high_hdop_change_indices)
    idx = high_hdop_change_indices(i);
    scatter3(Lng(idx), Lat(idx), Alt(idx), 100, 'm', 'filled');
    text(Lng(idx), Lat(idx), Alt(idx), sprintf('%d', i), 'FontSize', 10, 'Color', 'k', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
end

xlabel('Longitude');
ylabel('Latitude');
zlabel('Altitude');
title('GPS Position with HDop');

view(45, 30);

% Change the Datatip for this project
dcm = datacursormode(gcf);
set(dcm, 'UpdateFcn', @(obj, event_obj) GPS_log_view_Datatip(obj, event_obj, Lng, Lat, Alt, hdop));