function output_txt = GPS_log_view_Datatip(~, event_obj, Lng, Lat, Alt, hdop)
    % Function to generate a custom data tip showing Latitude, Longitude, Altitude, and HDop
    pos = get(event_obj, 'Position');
    idx = find(Lng == pos(1) & Lat == pos(2) & Alt == pos(3));

    output_txt = {
        ['Latitude: ', num2str(Lat(idx))], ...
        ['Longitude: ', num2str(Lng(idx))], ...
        ['Altitude: ', num2str(Alt(idx))], ...
        ['HDop: ', num2str(hdop(idx))]
    };
end