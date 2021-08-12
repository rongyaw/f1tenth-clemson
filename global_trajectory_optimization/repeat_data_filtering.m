function [x_filtered, y_filtered, yaw_filtered] = repeat_data_filtering(x_raw, y_raw, yaw_raw)
x_prev = x_raw(1);
y_prev = y_raw(1);
yaw_prev = yaw_raw(1);

x_filtered = [x_prev];
y_filtered = [y_prev];
yaw_filtered = [yaw_prev];

for i = 2:1:length(x_raw)
    if x_raw(i) == x_prev && y_raw(i) == y_prev && yaw_raw(i) == yaw_prev
        continue
    else
        x_filtered = [x_filtered, x_raw(i)];
        y_filtered = [y_filtered, y_raw(i)];
        yaw_filtered = [yaw_filtered, yaw_raw(i)];
        
        x_prev = x_raw(i);
        y_prev = y_raw(i);
        yaw_prev = yaw_raw(i);
    end
end

