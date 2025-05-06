clear; clc;

% Serial port configuration
serialPort = 'COM4'; % Adjust as needed
baudRate = 115200;
timeout = 10; % Timeout in seconds

% Open serial connection
serialObj = serialport(serialPort, baudRate, 'Timeout', timeout);
pause(2); % Allow time for initialization

% Flush any existing data in buffer
flush(serialObj);

% Open file for writing
filename = 'tof_scan_data.xyz';
fid = fopen(filename, 'w');
fprintf(fid, 'X Y Z\n'); % Write header

fprintf("Waiting for data...\n");

data = [];

while true
    try
        % Check if data is available to read
        if serialObj.NumBytesAvailable > 0
            line = readline(serialObj); % Read line from serial port
            line = strtrim(line); % Trim whitespace

            if isempty(line)
                continue; % Ignore empty lines
            end

            fprintf("Received: %s\n", line); % Debugging output

            % Check if the scan is complete
            if contains(line, "Scan complete") || contains(line, "Data acquisition stopped.")
                fprintf("Scan completed, stopping data collection.\n");
                break;
            end

            % Split the line into parts
            parts = strsplit(line, ',');
            if length(parts) ~= 5
                fprintf("Skipping invalid line: %s\n", line);
                continue;
            end

            try
                x = str2double(strtrim(parts{3})); % X coordinate
                y = str2double(strtrim(parts{4})); % Y coordinate
                z = str2double(strtrim(parts{5})); % Z coordinate

                if isnan(x) || isnan(y) || isnan(z)
                    error("Invalid numeric values");
                end

                % Save to XYZ file
                fprintf(fid, '%.6f %.6f %.6f\n', x, y, z);
                data = [data; x, y, z];
                fprintf("Recorded: %.6f, %.6f, %.6f\n", x, y, z);

            catch ME
                fprintf("Error parsing line: %s - %s\n", line, ME.message);
            end
        else
            % Wait briefly to avoid excessive CPU usage when no data is available
            pause(0.1); 
        end
    catch ME
        fprintf("Error reading from serial: %s\n", ME.message);
        break;
    end
end

% Close file and serial port
fclose(fid);
clear serialObj;

fprintf("Data saved to %s\n", filename);

% Plot 3D model with connected lines
figure;
plot3(data(:,1), data(:,2), data(:,3), '-o', 'LineWidth', 1.5, 'MarkerSize', 3);

xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D TOF Scan Data');
grid on;
axis equal;
